#include <canopen_master/objdict.h>
#include <socketcan_interface/string.h>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/ini_parser.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/algorithm/string.hpp>

namespace canopen{
    size_t hash_value(ObjectDict::Key const& k)  { return k.hash;  }
    std::ostream& operator<<(std::ostream& stream, const ObjectDict::Key &k) { return stream << std::string(k); }
}

using namespace canopen;

template<> const String & HoldAny::get() const{
    return buffer;
}

template<> String & ObjectStorage::Data::access(){
    if(!valid){
        THROW_WITH_KEY(std::length_error("buffer not valid") , key);
    }
    return buffer;
}
template<> String & ObjectStorage::Data::allocate(){
    buffer.resize(0);
    valid = true;
    return buffer;
}

size_t ObjectDict::Key::fromString(const std::string &str){
    uint16_t index = 0;
    uint8_t sub_index = 0;
    int num = sscanf(str.c_str(),"%hxsub%hhx", &index, &sub_index);
    return (index << 16) | (num==2?sub_index:0xFFFF);
}
ObjectDict::Key::operator std::string() const{
    std::stringstream sstr;
    sstr << std::hex << index();
    if(hasSub()) sstr << "sub" << (int) sub_index();
    return sstr.str();
}
void ObjectStorage::Data::init(){
    boost::mutex::scoped_lock lock(mutex);

    if(entry->init_val.is_empty()) return;

    if(valid && !entry->def_val.is_empty() && buffer != entry->def_val.data()) return; // buffer was changed

    if(!valid || buffer != entry->init_val.data()){
        buffer = entry->init_val.data();
        valid = true;
        if(entry->writable && (entry->def_val.is_empty() || entry->init_val.data() != entry->def_val.data()))
            write_delegate(*entry, buffer);
    }
}
void ObjectStorage::Data::force_write(){
    boost::mutex::scoped_lock lock(mutex);

    if(!valid && entry->readable){
        read_delegate(*entry, buffer);
        valid = true;
    }
    if(valid) write_delegate(*entry, buffer);
}

void ObjectStorage::Data::reset(){
    boost::mutex::scoped_lock lock(mutex);
    if(!entry->def_val.is_empty() && entry->def_val.type() == type_guard){
        buffer = entry->def_val.data();
        valid = true;
    }else{
        valid = false;
    }
}

bool ObjectDict::iterate(ObjectDict::ObjectDictMap::const_iterator &it) const{
    if(it != ObjectDict::ObjectDictMap::const_iterator()){
        ++it;
    }else it = dict_.begin();
    return it != dict_.end();
}
void set_access( ObjectDict::Entry &entry, std::string access){
    boost::algorithm::to_lower(access);
    entry.constant = false;
    if(access == "ro"){
        entry.readable = true;
        entry.writable = false;
    }else if (access == "wo"){
        entry.readable = false;
        entry.writable = true;
    }else if (access == "rw"){
        entry.readable = true;
        entry.writable = true;
    }else if (access == "rwr"){
        entry.readable = true;
        entry.writable = true;
    }else if (access == "rww"){
        entry.readable = true;
        entry.writable = true;
    }else if (access == "const"){
        entry.readable = true;
        entry.writable = false;
        entry.constant = true;
    }else{
        THROW_WITH_KEY(ParseException("Cannot determine access"), ObjectDict::Key(entry));
    }
}

template<typename T> T int_from_string(const std::string &s);

template<> bool   int_from_string(const std::string &s){
    return strtol(s.c_str(), 0, 0 ) > 0;
}

template<> int8_t int_from_string(const std::string &s){
    return strtol(s.c_str(), 0, 0);
}
template<> uint8_t int_from_string(const std::string &s){
    return strtoul(s.c_str(), 0, 0);
}
template<> int16_t int_from_string(const std::string &s){
    return strtol(s.c_str(), 0, 0);
}
template<> uint16_t int_from_string(const std::string &s){
    return strtoul(s.c_str(), 0, 0);
}
template<> int32_t int_from_string(const std::string &s){
    return strtol(s.c_str(), 0, 0);
}
template<> uint32_t int_from_string(const std::string &s){
    return strtoul(s.c_str(), 0, 0);
}

template<> int64_t int_from_string(const std::string &s){
    return strtoll(s.c_str(), 0, 0);
}
template<> uint64_t int_from_string(const std::string &s){
    return strtoull(s.c_str(), 0, 0);
}

template<typename T> HoldAny parse_int(boost::property_tree::iptree &pt, const std::string &key){
    if(pt.count(key) == 0) return HoldAny(TypeGuard::create<T>());

    std::string str = boost::trim_copy(pt.get<std::string>(key));
    if(boost::istarts_with(str,"$NODEID")){
        return HoldAny(NodeIdOffset<T>(int_from_string<T>(boost::trim_copy(str.substr(str.find("+",7)+1)))));
    }else return HoldAny(int_from_string<T>(str));
}

template<typename T> HoldAny parse_octets(boost::property_tree::iptree &pt, const std::string &key){
    std::string out;
    if(pt.count(key) == 0 || can::hex2buffer(out,pt.get<std::string>(key), true)) return HoldAny(TypeGuard::create<T>());
    return HoldAny(T(out));
}

template<typename T> HoldAny parse_typed_value(boost::property_tree::iptree &pt, const std::string &key){
    if(pt.count(key) == 0) return HoldAny(TypeGuard::create<T>());
    return HoldAny(pt.get<T>(key));
}
template<> HoldAny parse_typed_value<String>(boost::property_tree::iptree &pt, const std::string &key){
    if(pt.count(key) == 0) return HoldAny(TypeGuard::create<String>());
    return HoldAny(String(pt.get<std::string>(key)));
}

struct ReadAnyValue{
    template<const ObjectDict::DataTypes dt> static HoldAny func(boost::property_tree::iptree &pt, const std::string &key);
    static HoldAny read_value(boost::property_tree::iptree &pt, uint16_t data_type, const std::string &key){
        return branch_type<ReadAnyValue, HoldAny (boost::property_tree::iptree &, const std::string &)>(data_type)(pt, key);
    }
};
template<> HoldAny ReadAnyValue::func<ObjectDict::DEFTYPE_BOOLEAN>(boost::property_tree::iptree &pt, const std::string &key){ return parse_int<bool>(pt, key); }
template<> HoldAny ReadAnyValue::func<ObjectDict::DEFTYPE_INTEGER8>(boost::property_tree::iptree &pt, const std::string &key){  return parse_int<int8_t>(pt,key); }
template<> HoldAny ReadAnyValue::func<ObjectDict::DEFTYPE_INTEGER16>(boost::property_tree::iptree &pt, const std::string &key){  return parse_int<int16_t>(pt,key); }
template<> HoldAny ReadAnyValue::func<ObjectDict::DEFTYPE_INTEGER32>(boost::property_tree::iptree &pt, const std::string &key){  return parse_int<int32_t>(pt,key); }
template<> HoldAny ReadAnyValue::func<ObjectDict::DEFTYPE_INTEGER64>(boost::property_tree::iptree &pt, const std::string &key){  return parse_int<int64_t>(pt,key); }

template<> HoldAny ReadAnyValue::func<ObjectDict::DEFTYPE_UNSIGNED8>(boost::property_tree::iptree &pt, const std::string &key){  return parse_int<uint8_t>(pt,key); }
template<> HoldAny ReadAnyValue::func<ObjectDict::DEFTYPE_UNSIGNED16>(boost::property_tree::iptree &pt, const std::string &key){  return parse_int<uint16_t>(pt,key); }
template<> HoldAny ReadAnyValue::func<ObjectDict::DEFTYPE_UNSIGNED32>(boost::property_tree::iptree &pt, const std::string &key){  return parse_int<uint32_t>(pt,key); }
template<> HoldAny ReadAnyValue::func<ObjectDict::DEFTYPE_UNSIGNED64>(boost::property_tree::iptree &pt, const std::string &key){  return parse_int<uint64_t>(pt,key); }

template<> HoldAny ReadAnyValue::func<ObjectDict::DEFTYPE_DOMAIN>(boost::property_tree::iptree &pt, const std::string &key)
{ return parse_octets<ObjectStorage::DataType<ObjectDict::DEFTYPE_DOMAIN>::type>(pt,key); }

template<> HoldAny ReadAnyValue::func<ObjectDict::DEFTYPE_OCTET_STRING>(boost::property_tree::iptree &pt, const std::string &key)
{ return parse_octets<ObjectStorage::DataType<ObjectDict::DEFTYPE_OCTET_STRING>::type>(pt,key); }

template<const ObjectDict::DataTypes dt> HoldAny ReadAnyValue::func(boost::property_tree::iptree &pt, const std::string &key){
    return parse_typed_value<typename ObjectStorage::DataType<dt>::type>(pt, key);
}

template<typename T> void read_optional(T& var, boost::property_tree::iptree &pt, const std::string &key){
    var = pt.get(key, T());
}

template<typename T> void read_integer(T& var, boost::property_tree::iptree &pt, const std::string &key){
    var = int_from_string<T>(pt.get<std::string>(key));
}

template<typename T> T read_integer(boost::property_tree::iptree &pt, const std::string &key){
    return int_from_string<T>(pt.get<std::string>(key, std::string()));
}


void read_var(ObjectDict::Entry &entry, boost::property_tree::iptree &object){
        read_integer<uint16_t>(entry.data_type, object, "DataType");
        entry.mappable = object.get<bool>("PDOMapping", false);
        try{
            set_access(entry, object.get<std::string>("AccessType"));
        }
        catch(...){
            THROW_WITH_KEY(ParseException("No AccessType") , ObjectDict::Key(entry));

        }
        entry.def_val = ReadAnyValue::read_value(object, entry.data_type, "DefaultValue");
        entry.init_val = ReadAnyValue::read_value(object, entry.data_type, "ParameterValue");
}

void parse_object(ObjectDictSharedPtr dict, boost::property_tree::iptree &pt, const std::string &name, const uint8_t* sub_index = 0){
    boost::optional<boost::property_tree::iptree&> object =  pt.get_child_optional(name.substr(2));
    if(!object) return;

    std::shared_ptr<ObjectDict::Entry> entry = std::make_shared<ObjectDict::Entry>();
    try{
        entry->index = int_from_string<uint16_t>(name);
        entry->obj_code = ObjectDict::Code(int_from_string<uint16_t>(object->get<std::string>("ObjectType", boost::lexical_cast<std::string>((uint16_t)ObjectDict::VAR))));
        entry->desc = object->get<std::string>("Denotation",object->get<std::string>("ParameterName"));

        // std::cout << name << ": "<< entry->desc << std::endl;
        if(entry->obj_code == ObjectDict::VAR || entry->obj_code == ObjectDict::DOMAIN_DATA || sub_index){
            entry->sub_index = sub_index? *sub_index: 0;
            read_var(*entry, *object);
            dict->insert(sub_index != 0, entry);
        }else if(entry->obj_code == ObjectDict::ARRAY || entry->obj_code == ObjectDict::RECORD){
            uint8_t subs = read_integer<uint8_t>(*object, "CompactSubObj");
            if(subs){ // compact
                dict->insert(true, std::make_shared<const canopen::ObjectDict::Entry>(entry->index, 0, ObjectDict::DEFTYPE_UNSIGNED8, "NrOfObjects", true, false, false, HoldAny(subs)));

                read_var(*entry, *object);

                for(uint8_t i=1; i< subs; ++i){
                    std::string subname = pt.get<std::string>(name.substr(2)+"Name." + boost::lexical_cast<std::string>((int)i),entry->desc + boost::lexical_cast<std::string>((int)i));
                    subname = pt.get<std::string>(name.substr(2)+"Denotation." + boost::lexical_cast<std::string>((int)i), subname);

                    dict->insert(true, std::make_shared<const canopen::ObjectDict::Entry>(entry->index, i, entry->data_type, name, entry->readable, entry->writable, entry->mappable, entry->def_val,
                       ReadAnyValue::read_value(pt, entry->data_type, name.substr(2)+"Value." + boost::lexical_cast<std::string>((int)i))));
                }
            }else{
                read_integer(subs, *object, "SubNumber");
                for(uint8_t i=0; i< subs; ++i){
                   std::stringstream buf;
                   buf << name << "sub" << std::hex << int(i);
                   // std::cout << "added " << buf.str() <<  "  " << int(i) << std::endl;
                   parse_object(dict, pt, buf.str(), &i);
                }
            }
        }else{
            THROW_WITH_KEY(ParseException("Object type not supported") , ObjectDict::Key(*entry));
        }
    }
    catch(const std::bad_cast &e){
        throw ParseException(std::string("Type of ") + name + " does not match or is not supported");
    }
    catch(const std::exception&e){
        throw ParseException(std::string("Cannot process ") + name + ": " + e.what());
    }
}
void parse_objects(ObjectDictSharedPtr dict, boost::property_tree::iptree &pt, const std::string &key){
    if(!pt.count(key)) return;

    boost::property_tree::iptree objects = pt.get_child(key);
    uint16_t count = read_integer<uint16_t>(objects, "SupportedObjects");
    for(uint16_t i=0; i < count; ++i){
        std::string name = objects.get<std::string>(boost::lexical_cast<std::string>(i+1));
        parse_object(dict, pt, name);
    }
}
ObjectDictSharedPtr ObjectDict::fromFile(const std::string &path, const ObjectDict::Overlay &overlay){
    DeviceInfo info;
    boost::property_tree::iptree pt;
    ObjectDictSharedPtr dict;

    boost::property_tree::read_ini(path, pt);

    boost::property_tree::iptree di = pt.get_child("DeviceInfo");

    read_optional(info.vendor_name, di, "VendorName");
    read_optional(info.vendor_number, di, "VendorNumber");
    read_optional(info.product_name, di, "ProductName");
    read_optional(info.product_number, di, "ProductNumber");
    read_optional(info.revision_number, di, "RevisionNumber");
    read_optional(info.order_code, di, "OrderCode");
    read_optional(info.simple_boot_up_master, di, "SimpleBootUpMaster");
    read_optional(info.simple_boot_up_slave, di, "SimpleBootUpSlave");
    read_optional(info.granularity, di, "Granularity");
    read_optional(info.dynamic_channels_supported, di, "DynamicChannelsSupported");
    read_optional(info.group_messaging, di, "GroupMessaging");
    read_optional(info.nr_of_rx_pdo, di, "NrOfRXPDO");
    read_optional(info.nr_of_tx_pdo, di, "NrOfTXPDO");
    read_optional(info.lss_supported, di, "LSS_Supported");

    std::unordered_set<uint32_t> baudrates;
    std::unordered_set<uint16_t> dummy_usage;

    for(boost::property_tree::iptree::value_type &v: di){
        if(v.first.find("BaudRate_") == 0){
            uint16_t rate = int_from_string<uint16_t>(v.first.substr(9));
            if(v.second.get_value<bool>())
                info.baudrates.insert(rate * 1000);
        }
    }

    if(pt.count("DummyUsage")){
        for(boost::property_tree::iptree::value_type &v: pt.get_child("DummyUsage")){
            if(v.first.find("Dummy") == 0){
                // std::cout << ("0x"+v.first.substr(5)) << std::endl;
                uint16_t dummy = int_from_string<uint16_t>("0x"+v.first.substr(5));
                if(v.second.get_value<bool>())
                    info.dummy_usage.insert(dummy);
            }
        }
    }

    dict = std::make_shared<ObjectDict>(info);

    for(Overlay::const_iterator it= overlay.begin(); it != overlay.end(); ++it){
        pt.get_child(it->first).put("ParameterValue", it->second);
    }

    parse_objects(dict, pt, "MandatoryObjects");
    parse_objects(dict, pt, "OptionalObjects");
    parse_objects(dict, pt, "ManufacturerObjects");

    return dict;
}

size_t ObjectStorage::map(const ObjectDict::EntryConstSharedPtr &e, const ObjectDict::Key &key, const ReadFunc & read_delegate, const WriteFunc & write_delegate){
    ObjectStorageMap::iterator it = storage_.find(key);

    if(it == storage_.end()){

        DataSharedPtr data;


        if(!e->def_val.type().valid()){
            THROW_WITH_KEY(std::bad_cast() , key);
        }

        data = std::make_shared<Data>(key, e,e->def_val.type(),read_delegate_, write_delegate_);

        std::pair<ObjectStorageMap::iterator, bool>  ok = storage_.insert(std::make_pair(key, data));
        it = ok.first;
        it->second->reset();

    }

    if(read_delegate && write_delegate){
        it->second->set_delegates(read_delegate_, write_delegate);
        it->second->force_write(); // update buffer
        it->second->set_delegates(read_delegate, write_delegate_);
    }else if(write_delegate) {
        it->second->set_delegates(read_delegate_, write_delegate);
        it->second->force_write(); // update buffer
    }else if(read_delegate){
        it->second->set_delegates(read_delegate, write_delegate_);
    }
    return it->second->size();
}

size_t ObjectStorage::map(uint16_t index, uint8_t sub_index, const ReadFunc & read_delegate, const WriteFunc & write_delegate){
    boost::mutex::scoped_lock lock(mutex_);

    try{
        ObjectDict::Key key(index,sub_index);
        const ObjectDict::EntryConstSharedPtr e = dict_->get(key);
        return map(e, key,read_delegate, write_delegate);
    }
    catch(std::out_of_range) {
        if(sub_index != 0) throw;

        ObjectDict::Key key(index);
        const ObjectDict::EntryConstSharedPtr e = dict_->get(key);
        return map(e, key, read_delegate, write_delegate);
    }
}

ObjectStorage::ObjectStorage(ObjectDictConstSharedPtr dict, uint8_t node_id, ReadFunc read_delegate, WriteFunc write_delegate)
:read_delegate_(read_delegate), write_delegate_(write_delegate), dict_(dict), node_id_(node_id){
    assert(dict_);
    assert(read_delegate_);
    assert(write_delegate_);
}

void ObjectStorage::init_nolock(const ObjectDict::Key &key, const ObjectDict::EntryConstSharedPtr &entry){

    if(!entry->init_val.is_empty()){
        ObjectStorageMap::iterator it = storage_.find(key);

        if(it == storage_.end()){
            DataSharedPtr data = std::make_shared<Data>(key,entry, entry->init_val.type(), read_delegate_, write_delegate_);
            std::pair<ObjectStorageMap::iterator, bool>  ok = storage_.insert(std::make_pair(key, data));
            it = ok.first;
            if(!ok.second){
                THROW_WITH_KEY(std::bad_alloc() , key);
            }
        }
        it->second->init();
    }
}
void ObjectStorage::init(const ObjectDict::Key &key){
    boost::mutex::scoped_lock lock(mutex_);
    init_nolock(key, dict_->get(key));
}
void ObjectStorage::init_all(){
    boost::mutex::scoped_lock lock(mutex_);

    ObjectDict::ObjectDictMap::const_iterator entry_it;
    while(dict_->iterate(entry_it)){
        init_nolock(entry_it->first, entry_it->second);
    }
}

void ObjectStorage::reset(){
    boost::mutex::scoped_lock lock(mutex_);
    for(ObjectStorageMap::iterator it = storage_.begin(); it != storage_.end(); ++it){
        it->second->reset();
    }
}

template<const ObjectDict::DataTypes dt, typename T> std::string formatValue(const T &value){
    std::stringstream sstr;
    sstr << value;
    return sstr.str();
}
template<> std::string formatValue<ObjectDict::DEFTYPE_DOMAIN>(const std::string &value){
    return can::buffer2hex(value, false);
}
template<> std::string formatValue<ObjectDict::DEFTYPE_OCTET_STRING>(const std::string &value){
    return can::buffer2hex(value, false);
}

struct PrintValue {
    template<const ObjectDict::DataTypes dt> static std::string func(ObjectStorage& storage, const ObjectDict::Key &key, bool cached){
        ObjectStorage::Entry<typename ObjectStorage::DataType<dt>::type> entry = storage.entry<typename ObjectStorage::DataType<dt>::type>(key);
        return formatValue<dt>(cached? entry.get_cached() : entry.get() );
    }
    static std::function<std::string()> getReader(ObjectStorage& storage, const ObjectDict::Key &key, bool cached){
        ObjectDict::DataTypes data_type = (ObjectDict::DataTypes) storage.dict_->get(key)->data_type;
        return std::bind(branch_type<PrintValue, std::string (ObjectStorage&, const ObjectDict::Key &, bool)>(data_type),std::ref(storage), key, cached);
    }
};

ObjectStorage::ReadStringFuncType ObjectStorage::getStringReader(const ObjectDict::Key &key, bool cached){
    return PrintValue::getReader(*this, key, cached);
}

struct WriteStringValue {
    typedef HoldAny (*reader_type)(boost::property_tree::iptree &, const std::string &);
    template<typename T> static void write(ObjectStorage::Entry<T> entry, bool cached, reader_type reader, const std::string &value){
        boost::property_tree::iptree pt;
        pt.put("value", value);
        HoldAny any = reader(pt, "value");
        if(cached){
            entry.set_cached(any.get<T>());
        } else {
            entry.set(any.get<T>());
        }
    }
    template<const ObjectDict::DataTypes dt> static std::function<void (const std::string&)> func(ObjectStorage& storage, const ObjectDict::Key &key, bool cached){
        ObjectStorage::Entry<typename ObjectStorage::DataType<dt>::type> entry = storage.entry<typename ObjectStorage::DataType<dt>::type>(key);
        reader_type reader = branch_type<ReadAnyValue, HoldAny (boost::property_tree::iptree &, const std::string &)>(dt);
        return std::bind(&WriteStringValue::write<typename ObjectStorage::DataType<dt>::type >, entry, cached, reader, std::placeholders::_1);
    }
    static std::function<void (const std::string&)> getWriter(ObjectStorage& storage, const ObjectDict::Key &key, bool cached){
        ObjectDict::DataTypes data_type = (ObjectDict::DataTypes) storage.dict_->get(key)->data_type;
        return branch_type<WriteStringValue,  std::function<void (const std::string&)> (ObjectStorage&, const ObjectDict::Key &, bool)>(data_type)(storage, key, cached);
    }
};

ObjectStorage::WriteStringFuncType ObjectStorage::getStringWriter(const ObjectDict::Key &key, bool cached){
    return WriteStringValue::getWriter(*this, key, cached);
}
