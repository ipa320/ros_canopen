#ifndef H_CANOPEN
#define H_CANOPEN

#include <socketcan_interface/interface.h>
#include <socketcan_interface/dispatcher.h>
#include <socketcan_interface/reader.h>
#include "exceptions.h"
#include "layer.h"
#include "objdict.h"
#include "timer.h"
#include <stdexcept>
#include <boost/thread/condition_variable.hpp>
#include <boost/chrono/system_clocks.hpp>
#include <boost/lexical_cast.hpp>

namespace canopen{

typedef boost::chrono::high_resolution_clock::time_point time_point;
typedef boost::chrono::high_resolution_clock::duration time_duration;
inline time_point get_abs_time(const time_duration& timeout) { return boost::chrono::high_resolution_clock::now() + timeout; }
inline time_point get_abs_time() { return boost::chrono::high_resolution_clock::now(); }


template<typename T> struct FrameOverlay: public can::Frame{
    T &data;
    FrameOverlay(const Header &h) : can::Frame(h,sizeof(T)), data(* (T*) can::Frame::c_array()) {
        can::Frame::data.fill(0);
    }
    FrameOverlay(const can::Frame &f) : can::Frame(f), data(* (T*) can::Frame::c_array()) { }
};

class SDOClient{

    can::Header client_id;

    boost::timed_mutex mutex;

    can::BufferedReader reader_;
    bool processFrame(const can::Frame & msg);

    String buffer;
    size_t offset;
    size_t total;
    bool done;
    can::Frame last_msg;
    const canopen::ObjectDict::Entry * current_entry;

    void transmitAndWait(const canopen::ObjectDict::Entry &entry, const String &data, String *result);
    void abort(uint32_t reason);

    const can::CommInterfaceSharedPtr interface_;
protected:
    void read(const canopen::ObjectDict::Entry &entry, String &data);
    void write(const canopen::ObjectDict::Entry &entry, const String &data);
public:
    const ObjectStorageSharedPtr storage_;

    void init();

    SDOClient(const can::CommInterfaceSharedPtr interface, const ObjectDictSharedPtr dict, uint8_t node_id)
    : interface_(interface),
      storage_(std::make_shared<ObjectStorage>(dict, node_id,
                                               std::bind(&SDOClient::read, this, std::placeholders::_1, std::placeholders::_2),
                                               std::bind(&SDOClient::write, this, std::placeholders::_1, std::placeholders::_2))
              ),
      reader_(false, 1)
    {
    }
};

class PDOMapper{
    boost::mutex mutex_;

    class Buffer{
    public:
        bool read(uint8_t* b, const size_t len);
        void write(const uint8_t* b, const size_t len);
        void read(const canopen::ObjectDict::Entry &entry, String &data);
        void write(const canopen::ObjectDict::Entry &, const String &data);
        void clean() { dirty = false; }
        const size_t size;
        Buffer(const size_t sz) : size(sz), dirty(false), empty(true), buffer(sz) {}

    private:
        boost::mutex mutex;
        bool dirty;
        bool empty;
        std::vector<char> buffer;
    };
    typedef std::shared_ptr<Buffer> BufferSharedPtr;

    class PDO {
    protected:
        void parse_and_set_mapping(const ObjectStorageSharedPtr &storage, const uint16_t &com_index, const uint16_t &map_index, const bool &read, const bool &write);
        can::Frame frame;
        uint8_t transmission_type;
        std::vector<BufferSharedPtr>buffers;
    };

    struct TPDO: public PDO{
        typedef std::shared_ptr<TPDO> TPDOSharedPtr;
        void sync();
        static TPDOSharedPtr create(const can::CommInterfaceSharedPtr interface, const ObjectStorageSharedPtr &storage, const uint16_t &com_index, const uint16_t &map_index){
            TPDOSharedPtr tpdo(new TPDO(interface));
            if(!tpdo->init(storage, com_index, map_index))
                tpdo.reset();
            return tpdo;
        }
    private:
        TPDO(const can::CommInterfaceSharedPtr interface) : interface_(interface){}
        bool init(const ObjectStorageSharedPtr &storage, const uint16_t &com_index, const uint16_t &map_index);
        const can::CommInterfaceSharedPtr interface_;
        boost::mutex mutex;
    };

    struct RPDO : public PDO{
        void sync(LayerStatus &status);
        typedef std::shared_ptr<RPDO> RPDOSharedPtr;
        static RPDOSharedPtr create(const can::CommInterfaceSharedPtr interface, const ObjectStorageSharedPtr &storage, const uint16_t &com_index, const uint16_t &map_index){
            RPDOSharedPtr rpdo(new RPDO(interface));
            if(!rpdo->init(storage, com_index, map_index))
                rpdo.reset();
            return rpdo;
        }
    private:
        bool init(const ObjectStorageSharedPtr &storage, const uint16_t &com_index, const uint16_t &map_index);
        RPDO(const can::CommInterfaceSharedPtr interface) : interface_(interface), timeout(-1) {}
        boost::mutex mutex;
        const can::CommInterfaceSharedPtr interface_;

        can::FrameListenerConstSharedPtr listener_;
        void handleFrame(const can::Frame & msg);
        int timeout;
    };

    std::unordered_set<RPDO::RPDOSharedPtr> rpdos_;
    std::unordered_set<TPDO::TPDOSharedPtr> tpdos_;

    const can::CommInterfaceSharedPtr interface_;

public:
    PDOMapper(const can::CommInterfaceSharedPtr interface);
    void read(LayerStatus &status);
    bool write();
    bool init(const ObjectStorageSharedPtr storage, LayerStatus &status);
};

class EMCYHandler : public Layer {
    std::atomic<bool> has_error_;
    ObjectStorage::Entry<uint8_t> error_register_;
    ObjectStorage::Entry<uint8_t> num_errors_;
    can::FrameListenerConstSharedPtr emcy_listener_;
    void handleEMCY(const can::Frame & msg);
    const ObjectStorageSharedPtr storage_;

    virtual void handleDiag(LayerReport &report);

    virtual void handleInit(LayerStatus &status);
    virtual void handleRecover(LayerStatus &status);
    virtual void handleRead(LayerStatus &status, const LayerState &current_state);
    virtual void handleWrite(LayerStatus &status, const LayerState &current_state);
    virtual void handleHalt(LayerStatus &status);
    virtual void handleShutdown(LayerStatus &status);

public:
    EMCYHandler(const can::CommInterfaceSharedPtr interface, const ObjectStorageSharedPtr storage);
    void resetErrors(LayerStatus &status);
};

struct SyncProperties{
    const can::Header header_;
    const uint16_t period_ms_;
    const uint8_t overflow_;
    SyncProperties(const can::Header &h, const uint16_t  &p, const uint8_t &o) : header_(h), period_ms_(p), overflow_(o) {}
    bool operator==(const SyncProperties &p) const { return p.header_.key() == header_.key() && p.overflow_ == overflow_ && p.period_ms_ == period_ms_; }

};

class SyncCounter {
public:
    const SyncProperties properties;
    SyncCounter(const SyncProperties &p) : properties(p) {}
    virtual void addNode(void * const ptr)  = 0;
    virtual  void removeNode(void * const ptr) = 0;
    virtual ~SyncCounter() {}
};
typedef std::shared_ptr<SyncCounter> SyncCounterSharedPtr;

class Node : public Layer{
public:
    enum State{
        Unknown = 255, BootUp = 0, Stopped = 4, Operational = 5 , PreOperational = 127
    };
    const uint8_t node_id_;
    Node(const can::CommInterfaceSharedPtr interface, const ObjectDictSharedPtr dict, uint8_t node_id, const SyncCounterSharedPtr sync = SyncCounterSharedPtr());

    const State getState();
    void enterState(const State &s);

    const ObjectStorageSharedPtr getStorage() { return sdo_.storage_; }

    bool start();
    bool stop();
    bool reset();
    bool reset_com();
    bool prepare();

    using StateFunc = std::function<void(const State&)>;
    using StateDelegate [[deprecated("use StateFunc instead")]] = can::DelegateHelper<StateFunc>;

    typedef can::Listener<const StateFunc, const State&> StateListener;
    typedef StateListener::ListenerConstSharedPtr StateListenerConstSharedPtr;

    StateListenerConstSharedPtr addStateListener(const StateFunc & s){
        return state_dispatcher_.createListener(s);
    }

    template<typename T> T get(const ObjectDict::Key& k){
        return getStorage()->entry<T>(k).get();
    }

private:
    virtual void handleDiag(LayerReport &report);

    virtual void handleInit(LayerStatus &status);
    virtual void handleRecover(LayerStatus &status);
    virtual void handleRead(LayerStatus &status, const LayerState &current_state);
    virtual void handleWrite(LayerStatus &status, const LayerState &current_state);
    virtual void handleHalt(LayerStatus &status);
    virtual void handleShutdown(LayerStatus &status);

    template<typename T> int wait_for(const State &s, const T &timeout);

    boost::timed_mutex mutex;
    boost::mutex cond_mutex;
    boost::condition_variable cond;

    const can::CommInterfaceSharedPtr interface_;
    const SyncCounterSharedPtr sync_;
    can::FrameListenerConstSharedPtr nmt_listener_;

    ObjectStorage::Entry<ObjectStorage::DataType<ObjectDict::DEFTYPE_UNSIGNED16>::type> heartbeat_;

    can::SimpleDispatcher<StateListener> state_dispatcher_;

    void handleNMT(const can::Frame & msg);
    void switchState(const uint8_t &s);

    State state_;
    State wait_for_state_;
    SDOClient sdo_;
    PDOMapper pdo_;

    boost::chrono::high_resolution_clock::time_point heartbeat_timeout_;
    uint16_t getHeartbeatInterval() { return heartbeat_.valid()?heartbeat_.get_cached() : 0; }
    void setHeartbeatInterval() { if(heartbeat_.valid()) heartbeat_.set(heartbeat_.desc().value().get<uint16_t>()); }
    bool checkHeartbeat();
};
typedef std::shared_ptr<Node> NodeSharedPtr;

template<typename T> class Chain{
public:
    typedef std::shared_ptr<T> MemberSharedPtr;
    void call(void (T::*func)(void)){
        typename std::vector<MemberSharedPtr>::iterator it = elements.begin();
        while(it != elements.end()){
            ((**it).*func)();
            ++it;
        }
    }
    template<typename V> void call(void (T::*func)(const V&), const std::vector<V> &vs){
        typename std::vector<MemberSharedPtr>::iterator it = elements.begin();
        typename std::vector<V>::const_iterator it_v = vs.begin();
        while(it_v != vs.end() &&  it != elements.end()){
            ((**it).*func)(*it_v);
            ++it; ++it_v;
        }
    }
    template<typename V> void call(void (T::*func)(V&), std::vector<V> &vs){
        vs.resize(elements.size());

        typename std::vector<MemberSharedPtr>::iterator it = elements.begin();
        typename std::vector<V>::iterator it_v = vs.begin();
        while(it_v != vs.end() &&  it != elements.end()){
            ((**it).*func)(*it_v);
            ++it; ++it_v;
        }
    }
    void add(MemberSharedPtr t){
        elements.push_back(t);
    }
protected:
    std::vector<MemberSharedPtr> elements;
};

template<typename T> class NodeChain: public Chain<T>{
public:
    const std::vector<typename Chain<T>::MemberSharedPtr>& getElements() { return Chain<T>::elements; }
    void start() { this->call(&T::start); }
    void stop() { this->call(&T::stop); }
    void reset() { this->call(&T::reset); }
    void reset_com() { this->call(&T::reset_com); }
    void prepare() { this->call(&T::prepare); }
};

class SyncLayer: public Layer, public SyncCounter{
public:
    SyncLayer(const SyncProperties &p) : Layer("Sync layer"), SyncCounter(p) {}
};
typedef std::shared_ptr<SyncLayer> SyncLayerSharedPtr;

class Master{
    Master(const Master&) = delete; // prevent copies
    Master& operator=(const Master&) = delete;
public:
    Master() = default;
    virtual SyncLayerSharedPtr getSync(const SyncProperties &properties) = 0;
    virtual ~Master() {}

    typedef std::shared_ptr<Master> MasterSharedPtr;
    class Allocator {
    public:
        virtual MasterSharedPtr allocate(const std::string &name, can::CommInterfaceSharedPtr interface) = 0;
        virtual ~Allocator() {}
    };
};
typedef Master::MasterSharedPtr MasterSharedPtr;

using can::Settings;

} // canopen
#endif // !H_CANOPEN
