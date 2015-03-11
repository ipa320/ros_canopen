#include <canopen_master/canopen.h>

using namespace canopen;

#pragma pack(push) /* push current alignment to stack */
#pragma pack(1) /* set alignment to 1 byte boundary */


struct NMTcommand{
    enum Command{
        Start = 1,
        Stop = 2,
        Prepare = 128,
        Reset = 129,
        Reset_Com = 130
    };
    uint8_t command;
    uint8_t node_id;
    
    struct Frame: public FrameOverlay<NMTcommand>{
        Frame(uint8_t node_id, const Command &c) : FrameOverlay(can::Header()) {
            data.command = c;
            data.node_id = node_id;
        }
    };
};

#pragma pack(pop) /* pop previous alignment from stack */

Node::Node(const boost::shared_ptr<can::CommInterface> interface, const boost::shared_ptr<ObjectDict> dict, uint8_t node_id, const boost::shared_ptr<SyncCounter> sync)
: Layer("Node 301"), node_id_(node_id), interface_(interface), sync_(sync) , state_(Unknown), sdo_(interface, dict, node_id), emcy_(interface, getStorage()), pdo_(interface){
    getStorage()->entry(heartbeat_, 0x1017);
}
    
const Node::State Node::getState(){
    boost::timed_mutex::scoped_lock lock(mutex); // TODO: timed lock?
    return state_;
}

void Node::reset_com(){
    boost::timed_mutex::scoped_lock lock(mutex); // TODO: timed lock?
    getStorage()->reset();
    interface_->send(NMTcommand::Frame(node_id_, NMTcommand::Reset_Com));
    wait_for(BootUp, boost::chrono::seconds(10));
    state_ = PreOperational;
    heartbeat_.set(heartbeat_.desc().value().get<uint16_t>());

}
void Node::reset(){
    boost::timed_mutex::scoped_lock lock(mutex); // TODO: timed lock?
    getStorage()->reset();
    
    interface_->send(NMTcommand::Frame(node_id_, NMTcommand::Reset));
    wait_for(BootUp, boost::chrono::seconds(10));
    state_ = PreOperational;
    heartbeat_.set(heartbeat_.desc().value().get<uint16_t>());
}

void Node::prepare(){
    boost::timed_mutex::scoped_lock lock(mutex); // TODO: timed lock?
    if(state_ == BootUp){
        // ERROR
    }
    interface_->send(NMTcommand::Frame(node_id_, NMTcommand::Prepare));
    wait_for(PreOperational, boost::chrono::milliseconds(heartbeat_.get_cached() * 3));
}
void Node::start(){
    boost::timed_mutex::scoped_lock lock(mutex); // TODO: timed lock?
    if(state_ == BootUp){
        // ERROR
    }
    interface_->send(NMTcommand::Frame(node_id_, NMTcommand::Start));
    wait_for(Operational, boost::chrono::milliseconds(heartbeat_.get_cached() * 3));
}
void Node::stop(){
    boost::timed_mutex::scoped_lock lock(mutex); // TODO: timed lock?
    if(sync_) sync_->removeNode(this);
    if(state_ == BootUp){
        // ERROR
    }
    interface_->send(NMTcommand::Frame(node_id_, NMTcommand::Stop));
    wait_for(Stopped, boost::chrono::milliseconds(heartbeat_.get_cached() * 3));
}

void Node::switchState(const uint8_t &s){
    switch(s){
        case Operational:
            if(sync_) sync_->addNode(this);
            break;
        case BootUp:
        case PreOperational:
        case Stopped:
            if(sync_) sync_->removeNode(this);
            break;
        default:
            //error
            ;
    }
    state_ = (State) s;
    state_dispatcher_.dispatch(state_);
}
void Node::handleNMT(const can::Frame & msg){
    boost::mutex::scoped_lock cond_lock(cond_mutex);
    heartbeat_timeout_ = boost::chrono::high_resolution_clock::now() + boost::chrono::milliseconds(3*heartbeat_.get_cached());
    assert(msg.dlc == 1);
    switchState(msg.data[0]);
    cond_lock.unlock();
    cond.notify_one();
    
}
template<typename T> void Node::wait_for(const State &s, const T &timeout){
    boost::mutex::scoped_lock cond_lock(cond_mutex);
    time_point abs_time = get_abs_time(timeout);
    
    if(timeout == boost::chrono::milliseconds(0)){
        boost::this_thread::sleep_for(boost::chrono::milliseconds(100));
        switchState(s);
        boost::this_thread::yield();
    }
    
    while(s != state_)
    {
        if(cond.wait_until(cond_lock,abs_time) == boost::cv_status::timeout)
        {
            if(s != state_){
                BOOST_THROW_EXCEPTION( TimeoutException() );
            }
        }
   }
}
bool Node::checkHeartbeat(){
    if(!heartbeat_.get_cached()) return true; //disabled
    boost::mutex::scoped_lock cond_lock(cond_mutex);
    return heartbeat_timeout_ >= boost::chrono::high_resolution_clock::now();
}


void Node::read(LayerStatus &status){
    if(!checkHeartbeat()){
        status.error("heartbeat problem");
    } else if(getState() != Operational){
        status.error("not operational");
    } else{
        pdo_.read(status);
        emcy_.read(status);
    }
}
void Node::write(LayerStatus &status){
    if(getState() != Operational)  status.error("not operational");
    else if(! pdo_.write())  status.error("PDO write problem");
}


void Node::diag(LayerReport &report){
    State state = getState();
    if(state != Operational){
        report.error("Mode not operational");
        report.add("Node state", (int)state);
    }else if(!checkHeartbeat()){
        report.error("Heartbeat timeout");
    }
    if(state != Unknown) emcy_.diag(report);
}
void Node::init(LayerStatus &status){
    nmt_listener_ = interface_->createMsgListener( can::MsgHeader(0x700 + node_id_), can::CommInterface::FrameDelegate(this, &Node::handleNMT));

    sdo_.init();
    try{
        //reset();
        reset_com();
        emcy_.recover();
        prepare();
    }
    catch(const TimeoutException&){
        status.error(boost::str(boost::format("could not reset node '%1%'") % (int)node_id_));
        return;
    }
	emcy_.recover();

	pdo_.init(getStorage());
	getStorage()->init_all();
	sdo_.init(); // reread SDO paramters;
	// TODO: set SYNC data

    try{
        start();
    }
    catch(const TimeoutException&){
        status.error(boost::str(boost::format("could not start node '%1%'") %  (int)node_id_));
    }
	emcy_.init(status);
}
void Node::recover(LayerStatus &status){
	emcy_.recover();
	prepare();
    if(getState() != Operational){
        try{
            start();
        }
        catch(const TimeoutException&){
            status.error(boost::str(boost::format("could not start node '%1%'") %  (int)node_id_));
        }
    }
	emcy_.read(status);
}
void Node::shutdown(LayerStatus &status){
    stop();
    nmt_listener_.reset();
    switchState(Unknown);
}
void Node::halt(LayerStatus &status){
    // do nothing
}
void Node::pending(LayerStatus &status){
    // TODO
}
