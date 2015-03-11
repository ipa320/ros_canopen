#ifndef H_CANOPEN_LAYER
#define H_CANOPEN_LAYER

#include <vector>
#include <boost/shared_ptr.hpp>
#include <boost/atomic.hpp>

namespace canopen{

class LayerStatus{
    mutable boost::mutex write_mutex_;
    enum State{
        OK = 0, WARN = 1, ERROR= 2, STALE = 3, UNBOUNDED = 3
    };
    boost::atomic<State> state;
    std::string reason_;

    virtual void set(const State &s, const std::string &r){
        boost::mutex::scoped_lock lock(write_mutex_);
        if(s > state) state = s;
        if(!r.empty()){
            if(reason_.empty())  reason_ = r;
            else reason_ += "; " + r;
        }
    }
public:
    struct Ok { static const State state = OK; private: Ok();};
    struct Warn { static const State state = WARN; private: Warn(); };
    struct Error { static const State state = ERROR; private: Error(); };
    struct Stale { static const State state = STALE; private: Stale(); };
    struct Unbounded { static const State state = UNBOUNDED; private: Unbounded(); };

    template<typename T> bool bounded() const{ return state <= T::state; }
    
    LayerStatus() : state(OK) {}
    
    int get() const { return state; }
    
    const std::string reason() const { boost::mutex::scoped_lock lock(write_mutex_); return reason_; }

    const void warn(const std::string & r = "") { set(WARN, r); }
    const void error(const std::string & r = "") { set(ERROR, r); }
    const void stale(const std::string & r = "") { set(STALE, r); }
};
class LayerReport : public LayerStatus {
    std::vector<std::pair<std::string, std::string> > values_;
public:
    const std::vector<std::pair<std::string, std::string> > &values() const { return values_; }
    template<typename T> void add(const std::string &key, const T &value) {
        std::stringstream str;
        str << value;
        values_.push_back(std::make_pair(key,str.str()));
    }
};
class Layer{
public:
    const std::string name;

    virtual void pending(LayerStatus &status) = 0;
    virtual void read(LayerStatus &status) = 0;
    virtual void write(LayerStatus &status) = 0;
    
    virtual void diag(LayerReport &report) = 0;
    
    virtual void init(LayerStatus &status) = 0;
    virtual void shutdown(LayerStatus &status) = 0;
    
    virtual void halt(LayerStatus &status) = 0;
    virtual void recover(LayerStatus &status) = 0;
    
    Layer(const std::string &n) : name(n) {}
    
    virtual ~Layer() {}
};

template<typename T> class VectorHelper{
protected:
    typedef std::vector<boost::shared_ptr<T> > vector_type ;
    vector_type layers;
    
    template<typename Bound, typename Iterator, typename Data> Iterator call(void(Layer::*func)(Data&), Data &status, const Iterator &begin, const Iterator &end){
        bool okay_on_start = status.template bounded<Bound>();

        for(Iterator it = begin; it != end; ++it){
            ((**it).*func)(status);
            if(okay_on_start && !status.template bounded<Bound>()){
                return it;
            }
        }
        return end;
    }
    template<typename Iterator, typename Data> Iterator call(void(Layer::*func)(Data&), Data &status, const Iterator &begin, const Iterator &end){
        return call<LayerStatus::Unbounded, Iterator, Data>(func, status, begin, end);
    }
    void destroy() { layers.clear(); }
public:
    void add(const boost::shared_ptr<T> &l) { layers.push_back(l); }
};
    
class LayerStack : public Layer, public VectorHelper<Layer>{
    boost::mutex end_mutex_;
    vector_type::iterator run_end_;
    
    void bringup(void(Layer::*func)(LayerStatus&), void(Layer::*func_fail)(LayerStatus&), LayerStatus &status){
        vector_type::iterator it = layers.begin();
        {
            boost::mutex::scoped_lock lock(end_mutex_);
            run_end_ = it;
        }
        for(; it != layers.end(); ++it){
            {
                boost::mutex::scoped_lock lock(end_mutex_);
                run_end_ = it;
            }
            ((**it).*func)(status);
            if(!status.bounded<LayerStatus::Warn>()) break;
        }
        if(it != layers.end()){
            LayerStatus omit;
            call(func_fail, omit, vector_type::reverse_iterator(it), layers.rend());
        }
        {
            boost::mutex::scoped_lock lock(end_mutex_);
            run_end_ = it;
        }
    }
public:
    virtual void read(LayerStatus &status){
        vector_type::iterator end;
        {
            boost::mutex::scoped_lock lock(end_mutex_);
            if(end == run_end_){
                run_end_ = layers.begin(); // reset
            }
            end = run_end_;
        }
        vector_type::iterator it = call<LayerStatus::Warn>(&Layer::read, status, layers.begin(), end);
        if(it != end){
            LayerStatus omit;
            call(&Layer::halt, omit, layers.rbegin(), vector_type::reverse_iterator(it));
            omit.error();
            call(&Layer::read, omit, it+1, end);
        }
    }
    virtual void pending(LayerStatus &status){
        vector_type::iterator end;
        {
            boost::mutex::scoped_lock lock(end_mutex_);
            end = run_end_;
        }
        if(end != layers.end()){
            (**end).pending(status);
        }
    }
    virtual void write(LayerStatus &status){
        vector_type::reverse_iterator begin;
        {
            boost::mutex::scoped_lock lock(end_mutex_);
            begin = vector_type::reverse_iterator(run_end_);
        }
        
        vector_type::reverse_iterator it = call<LayerStatus::Warn>(&Layer::write, status, begin, layers.rend());
        if(it != layers.rend()){
            LayerStatus omit;
            call(&Layer::halt, omit, begin, vector_type::reverse_iterator(it));
            omit.error();
            call(&Layer::write, omit, it+1, layers.rend());
        }
    }
    virtual void diag(LayerReport &report){
        vector_type::iterator end;
        {
            boost::mutex::scoped_lock lock(end_mutex_);
            if(end == run_end_) return;
            end = run_end_;
        }
        vector_type::iterator it = call(&Layer::diag, report, layers.begin(), end);
    }
    virtual void init(LayerStatus &status) {
        bringup(&Layer::init, &Layer::shutdown, status);
    }
    virtual void recover(LayerStatus &status) {
        bringup(&Layer::recover, &Layer::halt, status);
    }
    virtual void shutdown(LayerStatus &status){
        {
            boost::mutex::scoped_lock lock(end_mutex_);
            run_end_ = layers.begin();
        }
        call(&Layer::shutdown, status, layers.rbegin(), layers.rend());
    }
    virtual void halt(LayerStatus &status){
        call(&Layer::halt, status, layers.rbegin(), layers.rend());
    }

    LayerStack(const std::string &n) : Layer(n) {}
};

template<typename T> class LayerGroup : public Layer, public VectorHelper<T>{
    typedef VectorHelper<T> V;
public:
    virtual void pending(LayerStatus &status){
        this->template call<LayerStatus::Warn>(&Layer::pending, status, this->layers.begin(), this->layers.end());
    }
    virtual void read(LayerStatus &status){
        typename V::vector_type::iterator it = this->template call<LayerStatus::Warn>(&Layer::read, status, this->layers.begin(), this->layers.end());
        if(it != this->layers.end()){
            LayerStatus omit;
            this->template call(&Layer::halt, omit, this->layers.begin(), this->layers.end());
            omit.error();
            this->template call(&Layer::read, omit, it+1, this->layers.end());
        }
    }
    virtual void write(LayerStatus &status){
        typename V::vector_type::iterator it = this->template call<LayerStatus::Warn>(&Layer::write, status, this->layers.begin(), this->layers.end());
        if(it != this->layers.end()){
            LayerStatus omit;
            this->template call(&Layer::halt, omit, this->layers.begin(), this->layers.end());
            omit.error();
            this->template call(&Layer::write, omit, it+1, this->layers.end());
        }
    }
    virtual void diag(LayerReport &report){
        this->template call(&Layer::diag, report, this->layers.begin(), this->layers.end());
    }
    virtual void init(LayerStatus &status) {
        typename V::vector_type::iterator it = this->template call<LayerStatus::Warn>(&Layer::init, status, this->layers.begin(), this->layers.end());
        if(it != this->layers.end()){
            LayerStatus omit;
            this->template call(&Layer::shutdown, omit, this->layers.begin(), this->layers.end());
        }
    }
    virtual void recover(LayerStatus &status){
        typename V::vector_type::iterator it = this->template call<LayerStatus::Warn>(&Layer::recover, status, this->layers.begin(), this->layers.end());
        if(it != this->layers.end()){
            LayerStatus omit;
            this->template call(&Layer::halt, omit, this->layers.begin(), this->layers.end());
        }
    }
    virtual void shutdown(LayerStatus &status){
        this->template call(&Layer::shutdown, status, this->layers.begin(), this->layers.end());
    }
    virtual void halt(LayerStatus &status){
        this->template call(&Layer::halt, status, this->layers.begin(), this->layers.end());
    }
    LayerGroup(const std::string &n) : Layer(n) {}
};

template<typename T> class LayerGroupNoDiag : public LayerGroup<T>{
public:
    LayerGroupNoDiag(const std::string &n) : LayerGroup<T>(n) {}
    virtual void diag(LayerReport &report){
        // no report
    }
};

template<typename T> class DiagGroup : public VectorHelper<T>{
    typedef VectorHelper<T> V;
public:
    virtual void diag(LayerReport &report){
        this->template call(&Layer::diag, report, this->layers.begin(), this->layers.end());
    }
};



} // namespace canopen

#endif
