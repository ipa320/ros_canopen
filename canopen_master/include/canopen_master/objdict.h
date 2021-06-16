#ifndef H_OBJDICT
#define H_OBJDICT

#include <list>
#include <memory>
#include <unordered_map>
#include <unordered_set>

#include <socketcan_interface/delegates.h>

#include <boost/thread/mutex.hpp>
#include <functional>
#include <typeinfo>
#include <vector>
#include "exceptions.h"

namespace canopen{

class TypeGuard{
    const std::type_info& (*get_type)();
    size_t type_size;

    template<typename T> class TypeInfo{
    public:
        static const std::type_info& id() { return typeid(T); }
    };
    TypeGuard(const std::type_info& (*ti)(), const size_t s): get_type(ti), type_size(s) {}
public:

    template<typename T> bool is_type() const {
        return valid() && get_type() == typeid(T);
    }

    bool operator==(const TypeGuard &other) const {
        return valid() && other.valid() && (get_type() == other.get_type());
    }

    TypeGuard(): get_type(0), type_size(0) {}
    bool valid() const { return get_type != 0; }
    size_t get_size() const { return type_size; }
    template<typename T> static TypeGuard create() { return TypeGuard(TypeInfo<T>::id, sizeof(T)); }
};

class String: public std::vector<char>{
public:
    String() {}
    String(const std::string &str) : std::vector<char>(str.begin(), str.end()) {}
    operator const char * () const {
        return &at(0);
    }
    operator const std::string () const {
        return std::string(begin(), end());
    }
};

class HoldAny{
    String buffer;
    TypeGuard type_guard;
    bool empty;
public:
    HoldAny() : empty(true) {}

    const TypeGuard& type() const{ return type_guard; }

    template<typename T> HoldAny(const T &t) : type_guard(TypeGuard::create<T>()), empty(false){
        buffer.resize(sizeof(T));
        *(T*)&(buffer.front()) = t;
    }
    HoldAny(const std::string &t): type_guard(TypeGuard::create<std::string>()), empty(false){
        if(!type_guard.is_type<std::string>()){
            BOOST_THROW_EXCEPTION(std::bad_cast());
        }
        buffer = t;
    }
    HoldAny(const TypeGuard &t): type_guard(t), empty(true){ }

    bool is_empty() const { return empty; }

    const String& data() const {
        if(empty){
            BOOST_THROW_EXCEPTION(std::length_error("buffer empty"));
        }
        return buffer;
    }

    template<typename T> const T & get() const{
        if(!type_guard.is_type<T>()){
            BOOST_THROW_EXCEPTION(std::bad_cast());
        }else if(empty){
            BOOST_THROW_EXCEPTION(std::length_error("buffer empty"));
        }
        return *(T*)&(buffer.front());
    }
};

template<> const String & HoldAny::get() const;

struct DeviceInfo{
    std::string vendor_name;
    uint32_t vendor_number;
    std::string product_name;
    uint32_t product_number;
    uint32_t revision_number;
    std::string order_code;
    std::unordered_set<uint32_t> baudrates;
    bool simple_boot_up_master;
    bool simple_boot_up_slave;
    uint8_t granularity;
    bool dynamic_channels_supported;
    bool group_messaging;
    uint16_t nr_of_rx_pdo;
    uint16_t nr_of_tx_pdo;
    bool lss_supported;
    std::unordered_set<uint16_t> dummy_usage;
};

#define THROW_WITH_KEY(e,k) BOOST_THROW_EXCEPTION(boost::enable_error_info(e) << ObjectDict::key_info(k))

class ObjectDict{
protected:
public:
    class Key{
        static size_t fromString(const std::string &str);
    public:
        const size_t hash;
        Key(const uint16_t i) : hash((i<<16)| 0xFFFF) {}
        Key(const uint16_t i, const uint8_t s): hash((i<<16)| s) {}
        Key(const std::string &str): hash(fromString(str)) {}
        bool hasSub() const { return (hash & 0xFFFF) != 0xFFFF; }
        uint8_t sub_index() const { return hash & 0xFFFF; }
        uint16_t index() const { return hash  >> 16;}
        bool operator==(const Key &other) const { return hash == other.hash; }
        operator std::string() const;
    };
    struct KeyHash {
        std::size_t operator()(const Key& k) const { return k.hash; }
    };

    enum Code{
        NULL_DATA = 0x00,
        DOMAIN_DATA = 0x02,
        DEFTYPE = 0x05,
        DEFSTRUCT = 0x06,
        VAR = 0x07,
        ARRAY = 0x08,
        RECORD = 0x09
    };
    enum DataTypes{
        DEFTYPE_BOOLEAN  = 0x0001,
        DEFTYPE_INTEGER8 = 0x0002,
        DEFTYPE_INTEGER16 = 0x0003,
        DEFTYPE_INTEGER32 = 0x0004,
        DEFTYPE_UNSIGNED8 = 0x0005,
        DEFTYPE_UNSIGNED16 = 0x0006,
        DEFTYPE_UNSIGNED32 = 0x0007,
        DEFTYPE_REAL32 = 0x0008,
        DEFTYPE_VISIBLE_STRING = 0x0009,
        DEFTYPE_OCTET_STRING = 0x000A,
        DEFTYPE_UNICODE_STRING = 0x000B,
        DEFTYPE_DOMAIN = 0x000F,
        DEFTYPE_REAL64 = 0x0010,
        DEFTYPE_INTEGER64 = 0x0015,
        DEFTYPE_UNSIGNED64 = 0x001B
    };
    struct Entry{
        Code obj_code;
        uint16_t index;
        uint8_t sub_index;
        uint16_t data_type;
        bool constant;
        bool readable;
        bool writable;
        bool mappable;
        std::string desc;
        HoldAny def_val;
        HoldAny init_val;

        Entry() {}

        Entry(const Code c, const uint16_t i,  const uint16_t t, const std::string & d, const bool r = true, const bool w = true, bool m = false, const HoldAny def = HoldAny(), const HoldAny init = HoldAny()):
        obj_code(c), index(i), sub_index(0),data_type(t),readable(r), writable(w), mappable(m), desc(d), def_val(def), init_val(init) {}

        Entry(const uint16_t i, const uint8_t s, const uint16_t t, const std::string & d, const bool r = true, const bool w = true, bool m = false, const HoldAny def = HoldAny(), const HoldAny init = HoldAny()):
        obj_code(VAR), index(i), sub_index(s),data_type(t),readable(r), writable(w), mappable(m), desc(d), def_val(def), init_val(init) {}

        operator Key() const { return Key(index, sub_index); }
        const HoldAny & value() const { return !init_val.is_empty() ? init_val : def_val; }

    };
    typedef std::shared_ptr<const Entry> EntryConstSharedPtr;

    const Entry& operator()(uint16_t i) const{
        return *at(Key(i));
    }
    const Entry& operator()(uint16_t i, uint8_t s) const{
        return *at(Key(i,s));
    }
    const EntryConstSharedPtr& get(const Key &k) const{
        return at(k);
    }
    bool has(uint16_t i, uint8_t s) const{
        return dict_.find(Key(i,s)) != dict_.end();
    }
    bool has(uint16_t i) const{
        return dict_.find(Key(i)) != dict_.end();
    }
    bool has(const Key &k) const{
        return dict_.find(k) != dict_.end();
    }

    bool insert(bool is_sub, EntryConstSharedPtr e){
        return dict_.insert(std::make_pair(is_sub?Key(e->index,e->sub_index):Key(e->index),e)).second;
    }

    typedef std::unordered_map<Key, EntryConstSharedPtr, KeyHash>  ObjectDictMap;
    bool iterate(ObjectDictMap::const_iterator &it) const;
    typedef std::list<std::pair<std::string, std::string> > Overlay;
    typedef std::shared_ptr<ObjectDict> ObjectDictSharedPtr;
    static ObjectDictSharedPtr fromFile(const std::string &path, const Overlay &overlay = Overlay());
    const DeviceInfo device_info;

    ObjectDict(const DeviceInfo &info): device_info(info) {}
    typedef boost::error_info<struct tag_objectdict_key, ObjectDict::Key> key_info;
protected:
    const EntryConstSharedPtr& at(const Key &key) const{
        try{
            return dict_.at(key);
        }
        catch(const std::out_of_range &e){
            THROW_WITH_KEY(e, key);
        }
    }

    ObjectDictMap dict_;
};
typedef ObjectDict::ObjectDictSharedPtr ObjectDictSharedPtr;
typedef std::shared_ptr<const ObjectDict> ObjectDictConstSharedPtr;

[[deprecated]]
std::size_t hash_value(ObjectDict::Key const& k);

template<typename T> class NodeIdOffset{
    T offset;
    T (*adder)(const uint8_t &, const T &);

    static T add(const uint8_t &u, const T &t) {
        return u+t;
    }
public:
    NodeIdOffset(const T &t): offset(t), adder(add) {}

    static const T apply(const HoldAny &val, const uint8_t &u){
        if(!val.is_empty()){
            if(TypeGuard::create<T>() ==  val.type() ){
                return val.get<T>();
            }else{
                const NodeIdOffset<T> &no = val.get< NodeIdOffset<T> >();
                return no.adder(u, no.offset);
            }
        }else{
            BOOST_THROW_EXCEPTION(std::bad_cast());
        }

    }
};

template<typename T> std::ostream& operator<<(std::ostream& stream, const NodeIdOffset<T> &n) {
    //stream << "Offset: " << n.apply(0);
    return stream;
 }
std::ostream& operator<<(std::ostream& stream, const ObjectDict::Key &k);

class AccessException : public Exception{
public:
    AccessException(const std::string &w) : Exception(w) {}
};


class ObjectStorage{
public:
    using ReadFunc = std::function<void(const ObjectDict::Entry&, String &)>;
    using ReadDelegate [[deprecated("use ReadFunc instead")]] = can::DelegateHelper<ReadFunc>;

    using WriteFunc = std::function<void(const ObjectDict::Entry&, const String &)>;
    using WriteDelegate  [[deprecated("use WriteFunc instead")]] = can::DelegateHelper<WriteFunc>;

    typedef std::shared_ptr<ObjectStorage> ObjectStorageSharedPtr;

protected:
    class Data {
        Data(const Data&) = delete; // prevent copies
        Data& operator=(const Data&) = delete;

        boost::mutex mutex;
        String buffer;
        bool valid;

        ReadFunc read_delegate;
        WriteFunc write_delegate;

        template <typename T> T & access(){
            if(!valid){
                THROW_WITH_KEY(std::length_error("buffer not valid"), key);
            }
            return *(T*)&(buffer.front());
        }
        template <typename T> T & allocate(){
            if(!valid){
                buffer.resize(sizeof(T));
                valid = true;
            }
            return access<T>();
        }
    public:
        const TypeGuard type_guard;
        const ObjectDict::EntryConstSharedPtr entry;
        const ObjectDict::Key key;
        size_t size() { boost::mutex::scoped_lock lock(mutex); return buffer.size(); }

        template<typename T> Data(const ObjectDict::Key &k, const ObjectDict::EntryConstSharedPtr &e, const T &val, const ReadFunc &r, const WriteFunc &w)
        : valid(false), read_delegate(r), write_delegate(w), type_guard(TypeGuard::create<T>()), entry(e), key(k){
            assert(r);
            assert(w);
            assert(e);
            allocate<T>() = val;
        }
        Data(const ObjectDict::Key &k, const ObjectDict::EntryConstSharedPtr &e, const TypeGuard &t, const ReadFunc &r, const WriteFunc &w)
        : valid(false), read_delegate(r), write_delegate(w), type_guard(t), entry(e), key(k){
            assert(r);
            assert(w);
            assert(e);
            assert(t.valid());
            buffer.resize(t.get_size());
        }
        void set_delegates(const ReadFunc &r, const WriteFunc &w){
            boost::mutex::scoped_lock lock(mutex);
            if(r) read_delegate = r;
            if(w) write_delegate = w;
        }
        template<typename T> const T get(bool cached) {
            boost::mutex::scoped_lock lock(mutex);

            if(!entry->readable){
                THROW_WITH_KEY(AccessException("no read access"), key);

            }

            if(entry->constant) cached = true;

            if(!valid || !cached){
                allocate<T>();
                read_delegate(*entry, buffer);
            }
            return access<T>();
        }
        template<typename T>  void set(const T &val) {
            boost::mutex::scoped_lock lock(mutex);

            if(!entry->writable){
                if(access<T>() != val){
                    THROW_WITH_KEY(AccessException("no write access"), key);
                }
            }else{
                allocate<T>() = val;
                write_delegate(*entry, buffer);
            }
        }
        template<typename T>  void set_cached(const T &val) {
            boost::mutex::scoped_lock lock(mutex);
            if(!valid || val != access<T>() ){
                if(!entry->writable){
                    THROW_WITH_KEY(AccessException("no write access and not cached"), key);
                }else{
                    allocate<T>() = val;
                    write_delegate(*entry, buffer);
                }
            }
        }
        void init();
        void reset();
        void force_write();

    };
    typedef std::shared_ptr<Data> DataSharedPtr;
public:
    template<const uint16_t dt> struct DataType{
        typedef void type;
    };

    template<typename T> class Entry{
        DataSharedPtr data;
    public:
        typedef T type;
        bool valid() const { return data != 0; }
        const T get() {
            if(!data) BOOST_THROW_EXCEPTION( PointerInvalid("ObjectStorage::Entry::get()") );

            return data->get<T>(false);
        }
        bool get(T & val){
            try{
                val = get();
                return true;
            }catch(...){
                return false;
            }
        }
        const T get_cached() {
            if(!data) BOOST_THROW_EXCEPTION( PointerInvalid("ObjectStorage::Entry::get_cached()") );

            return data->get<T>(true);
        }
        bool get_cached(T & val){
            try{
                val = get_cached();
                return true;
            }catch(...){
                return false;
            }
        }
        void set(const T &val) {
            if(!data) BOOST_THROW_EXCEPTION( PointerInvalid("ObjectStorage::Entry::set(val)") );
            data->set(val);
        }
        bool set_cached(const T &val) {
            if(!data) return false;
            try{
	            data->set_cached(val);
				return true;
            }catch(...){
                return false;
            }
        }

        Entry() {}
        Entry(DataSharedPtr &d)
        : data(d){
            assert(data);
        }
        Entry(ObjectStorageSharedPtr storage, uint16_t index)
        : data(storage->entry<type>(index).data) {
            assert(data);
        }
        Entry(ObjectStorageSharedPtr storage, uint16_t index, uint8_t sub_index)
        : data(storage->entry<type>(index, sub_index).data) {
            assert(data);
        }
        Entry(ObjectStorageSharedPtr storage, const ObjectDict::Key &k)
        : data(storage->entry<type>(k).data) {
            assert(data);
        }
        const ObjectDict::Entry & desc() const{
            return *(data->entry);
        }
    };

    void reset();

protected:
    typedef std::unordered_map<ObjectDict::Key, DataSharedPtr, ObjectDict::KeyHash> ObjectStorageMap;
    ObjectStorageMap storage_;
    boost::mutex mutex_;

    void init_nolock(const ObjectDict::Key &key, const ObjectDict::EntryConstSharedPtr &entry);

    ReadFunc read_delegate_;
    WriteFunc write_delegate_;
    size_t map(const ObjectDict::EntryConstSharedPtr &e, const ObjectDict::Key &key, const ReadFunc & read_delegate, const WriteFunc & write_delegate);
public:
    template<typename T> Entry<T> entry(const ObjectDict::Key &key){
        boost::mutex::scoped_lock lock(mutex_);

        ObjectStorageMap::iterator it = storage_.find(key);

        if(it == storage_.end()){
            const ObjectDict::EntryConstSharedPtr e = dict_->get(key);

            DataSharedPtr data;
            TypeGuard type = TypeGuard::create<T>();

            if(!e->def_val.is_empty()){
                T val = NodeIdOffset<T>::apply(e->def_val, node_id_);
                data = std::make_shared<Data>(key, e,val, read_delegate_, write_delegate_);
            }else{
                if(!e->def_val.type().valid() ||  e->def_val.type() == type) {
                    data = std::make_shared<Data>(key,e,type, read_delegate_, write_delegate_);
                }else{
                    THROW_WITH_KEY(std::bad_cast(), key);
                }
            }

            std::pair<ObjectStorageMap::iterator, bool>  ok = storage_.insert(std::make_pair(key, data));
            it = ok.first;
        }

        if(!it->second->type_guard.is_type<T>()){
            THROW_WITH_KEY(std::bad_cast(), key);
        }
        return Entry<T>(it->second);
    }

    size_t map(uint16_t index, uint8_t sub_index, const ReadFunc & read_delegate, const WriteFunc & write_delegate);

    template<typename T> Entry<T> entry(uint16_t index){
        return entry<T>(ObjectDict::Key(index));
    }
    template<typename T> Entry<T> entry(uint16_t index, uint8_t sub_index){
        return entry<T>(ObjectDict::Key(index,sub_index));
    }

    template<typename T> void entry(Entry<T> &e, uint16_t index){ // TODO: migrate to bool
        e = entry<T>(ObjectDict::Key(index));
    }
    template<typename T> void entry(Entry<T> &e, uint16_t index, uint8_t sub_index){  // TODO: migrate to bool
        e = entry<T>(ObjectDict::Key(index,sub_index));
    }
    template<typename T> bool entry(Entry<T> &e, const ObjectDict::Key &k){
        try{
            e = entry<T>(k);
            return true;
        }catch(...){
            return false;
        }
    }
    typedef std::function<std::string()> ReadStringFuncType;
    ReadStringFuncType getStringReader(const ObjectDict::Key &key, bool cached = false);
    typedef std::function<void(const std::string &)>  WriteStringFuncType;
    WriteStringFuncType getStringWriter(const ObjectDict::Key &key, bool cached = false);

    const ObjectDictConstSharedPtr dict_;
    const uint8_t node_id_;

    ObjectStorage(ObjectDictConstSharedPtr dict, uint8_t node_id, ReadFunc read_delegate, WriteFunc write_delegate);

    void init(const ObjectDict::Key &key);
    void init_all();
};
typedef ObjectStorage::ObjectStorageSharedPtr ObjectStorageSharedPtr;

template<> String & ObjectStorage::Data::access();
template<> String & ObjectStorage::Data::allocate();

template<> struct ObjectStorage::DataType<ObjectDict::DEFTYPE_BOOLEAN>  { typedef bool type; };
template<> struct ObjectStorage::DataType<ObjectDict::DEFTYPE_INTEGER8> { typedef int8_t type;};
template<> struct ObjectStorage::DataType<ObjectDict::DEFTYPE_INTEGER16> { typedef int16_t type;};
template<> struct ObjectStorage::DataType<ObjectDict::DEFTYPE_INTEGER32> { typedef int32_t type;};
template<> struct ObjectStorage::DataType<ObjectDict::DEFTYPE_INTEGER64> { typedef int64_t type;};
template<> struct ObjectStorage::DataType<ObjectDict::DEFTYPE_UNSIGNED8> { typedef uint8_t type;};

template<> struct ObjectStorage::DataType<ObjectDict::DEFTYPE_UNSIGNED16> { typedef uint16_t type;};
template<> struct ObjectStorage::DataType<ObjectDict::DEFTYPE_UNSIGNED32> { typedef uint32_t type;};
template<> struct ObjectStorage::DataType<ObjectDict::DEFTYPE_UNSIGNED64> { typedef uint64_t type;};

template<> struct ObjectStorage::DataType<ObjectDict::DEFTYPE_REAL32> { typedef float type;};
template<> struct ObjectStorage::DataType<ObjectDict::DEFTYPE_REAL64> { typedef double type;};

template<> struct ObjectStorage::DataType<ObjectDict::DEFTYPE_VISIBLE_STRING> { typedef String type;};
template<> struct ObjectStorage::DataType<ObjectDict::DEFTYPE_OCTET_STRING> { typedef String type;};
template<> struct ObjectStorage::DataType<ObjectDict::DEFTYPE_UNICODE_STRING> { typedef String type;};
template<> struct ObjectStorage::DataType<ObjectDict::DEFTYPE_DOMAIN> { typedef String type;};

template<typename T, typename R> static R *branch_type(const uint16_t data_type){
    switch(ObjectDict::DataTypes(data_type)){
        case ObjectDict::DEFTYPE_BOOLEAN:  return T::template func< ObjectDict::DEFTYPE_BOOLEAN >;
        case ObjectDict::DEFTYPE_INTEGER8: return T::template func< ObjectDict::DEFTYPE_INTEGER8 >;
        case ObjectDict::DEFTYPE_INTEGER16: return T::template func< ObjectDict::DEFTYPE_INTEGER16 >;
        case ObjectDict::DEFTYPE_INTEGER32: return T::template func< ObjectDict::DEFTYPE_INTEGER32 >;
        case ObjectDict::DEFTYPE_INTEGER64: return T::template func< ObjectDict::DEFTYPE_INTEGER64 >;

        case ObjectDict::DEFTYPE_UNSIGNED8: return T::template func< ObjectDict::DEFTYPE_UNSIGNED8 >;
        case ObjectDict::DEFTYPE_UNSIGNED16: return T::template func< ObjectDict::DEFTYPE_UNSIGNED16 >;
        case ObjectDict::DEFTYPE_UNSIGNED32: return T::template func< ObjectDict::DEFTYPE_UNSIGNED32 >;
        case ObjectDict::DEFTYPE_UNSIGNED64: return T::template func< ObjectDict::DEFTYPE_UNSIGNED64 >;

        case ObjectDict::DEFTYPE_REAL32: return T::template func< ObjectDict::DEFTYPE_REAL32 >;
        case ObjectDict::DEFTYPE_REAL64: return T::template func< ObjectDict::DEFTYPE_REAL64 >;

        case ObjectDict::DEFTYPE_VISIBLE_STRING: return T::template func< ObjectDict::DEFTYPE_VISIBLE_STRING >;
        case ObjectDict::DEFTYPE_OCTET_STRING: return T::template func< ObjectDict::DEFTYPE_OCTET_STRING >;
        case ObjectDict::DEFTYPE_UNICODE_STRING: return T::template func< ObjectDict::DEFTYPE_UNICODE_STRING >;
        case ObjectDict::DEFTYPE_DOMAIN: return T::template func< ObjectDict::DEFTYPE_DOMAIN >;

        default:
            throw std::bad_cast();
    }
}

} // canopen

#endif // !H_OBJDICT
