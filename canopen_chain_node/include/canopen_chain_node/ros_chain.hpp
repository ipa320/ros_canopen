#ifndef H_CANOPEN_ROS_CHAIN
#define H_CANOPEN_ROS_CHAIN

#include <memory>
#include <sys/stat.h>
#include <canopen_master/canopen.hpp>
#include <canopen_master/can_layer.hpp>
// #include <canopen_chain_node/msg/GetObject.hpp>
// #include <canopen_chain_node/msg/SetObject.hpp>
#include <socketcan_interface/string.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>
#include <boost/filesystem/path.hpp>
#include <pluginlib/class_loader.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

using namespace std::chrono_literals;

namespace canopen
{

typedef std::function<void ()> PublishFuncType;

class Logger : public DiagGroup<canopen::Layer>
{
  const canopen::NodeSharedPtr node_;

  std::vector<std::function<void(diagnostic_updater::DiagnosticStatusWrapper &)>> entries_;

  static void log_entry(
    diagnostic_updater::DiagnosticStatusWrapper & stat, uint8_t level,
    const std::string & name, std::function<std::string()> getter)
  {
    if (stat.level >= level) {
      try {
        stat.add(name, getter());
      } catch (...) {
        stat.add(name, "<ERROR>");
      }
    }
  }

public:
  Logger(canopen::NodeSharedPtr node)
  : node_(node) {add(node_);}

  bool add(uint8_t level, const std::string & key, bool forced)
  {
    try {
      ObjectDict::Key k(key);
      const ObjectDict::EntryConstSharedPtr entry = node_->getStorage()->dict_->get(k);
      std::string name = entry->desc.empty() ? key : entry->desc;
      entries_.push_back(std::bind(log_entry, std::placeholders::_1, level, name,
        node_->getStorage()->getStringReader(k, !forced)));
      return true;
    } catch (std::exception & e) {
      // RCLCPP_ERROR(this->get_logger(), boost::diagnostic_information(e));
      return false;
    }
  }

  template<typename T>
  void add(const std::shared_ptr<T> & n)
  {
    DiagGroup::add(std::static_pointer_cast<canopen::Layer>(n));
  }

  virtual void log(diagnostic_updater::DiagnosticStatusWrapper & stat)
  {
    if (node_->getState() == canopen::Node::Unknown) {
      stat.summary(stat.WARN, "Not initailized");
    } else {
      LayerReport r;
      diag(r);
      if (r.bounded<LayerStatus::Unbounded>()) { // valid
        stat.summary(r.get(), r.reason());
        for (std::vector<std::pair<std::string, std::string>>::const_iterator it =
          r.values().begin(); it != r.values().end(); ++it)
        {
          stat.add(it->first, it->second);
        }
        for (size_t i = 0; i < entries_.size(); ++i) {
          entries_[i](stat);
        }
      }
    }
  }

  virtual ~Logger() {}
};
typedef std::shared_ptr<Logger> LoggerSharedPtr;

class GuardedClassLoaderList
{
public:
  typedef std::shared_ptr<pluginlib::ClassLoaderBase> ClassLoaderBaseSharedPtr;
  static void addLoader(ClassLoaderBaseSharedPtr b)
  {
    guarded_loaders().push_back(b);
  }
  ~GuardedClassLoaderList()
  {
    guarded_loaders().clear();
  }

private:
  static std::vector<ClassLoaderBaseSharedPtr> & guarded_loaders()
  {
    static std::vector<ClassLoaderBaseSharedPtr> loaders;
    return loaders;
  }
};

template<typename T>
class GuardedClassLoader
{
  typedef pluginlib::ClassLoader<T> Loader;
  std::shared_ptr<Loader> loader_;

public:
  typedef std::shared_ptr<T> ClassSharedPtr;
  GuardedClassLoader(const std::string & package, const std::string & allocator_base_class)
  : loader_(new Loader(package, allocator_base_class))
  {
    GuardedClassLoaderList::addLoader(loader_);
  }
  ClassSharedPtr createInstance(const std::string & lookup_name)
  {
    return loader_->createUniqueInstance(lookup_name);
  }
};

template<typename T>
class ClassAllocator : public GuardedClassLoader<typename T::Allocator>
{
public:
  typedef std::shared_ptr<T> ClassSharedPtr;
  ClassAllocator(const std::string & package, const std::string & allocator_base_class)
  : GuardedClassLoader<typename T::Allocator>(package, allocator_base_class) {}
  template<typename T1>
  ClassSharedPtr allocateInstance(const std::string & lookup_name, const T1 & t1)
  {
    return this->createInstance(lookup_name)->allocate(t1);
  }
  template<typename T1, typename T2>
  ClassSharedPtr allocateInstance(const std::string & lookup_name, const T1 & t1, const T2 & t2)
  {
    return this->createInstance(lookup_name)->allocate(t1, t2);
  }
  template<typename T1, typename T2, typename T3>
  ClassSharedPtr allocateInstance(
    const std::string & lookup_name, const T1 & t1, const T2 & t2,
    const T3 & t3)
  {
    return this->createInstance(lookup_name)->allocate(t1, t2, t3);
  }
};

class RosChain : GuardedClassLoaderList, public canopen::LayerStack, public rclcpp::Node
{
  GuardedClassLoader<can::DriverInterface> driver_loader_;
  ClassAllocator<canopen::Master> master_allocator_;

protected:
  can::DriverInterfaceSharedPtr interface_;
  MasterSharedPtr master_;
  std::shared_ptr<canopen::LayerGroupNoDiag<canopen::Node>> nodes_;
  std::shared_ptr<canopen::LayerGroupNoDiag<canopen::EMCYHandler>> emcy_handlers_;
  std::map<std::string, canopen::NodeSharedPtr> nodes_lookup_;
  canopen::SyncLayerSharedPtr sync_;
  std::vector<LoggerSharedPtr> loggers_;
  std::vector<PublishFuncType> publishers_;

  can::StateListenerConstSharedPtr state_listener_;

  std::unique_ptr<boost::thread> thread_;

  diagnostic_updater::Updater diag_updater_;
  rclcpp::TimerBase::SharedPtr diag_timer_;

  boost::mutex mutex_;

  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_init_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_recover_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_halt_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_shutdown_;
  // ros::ServiceServer srv_get_object_;
  // ros::ServiceServer srv_set_object_;
  time_duration update_duration_;

  struct HeartbeatSender
  {
    can::Frame frame;
    can::DriverInterfaceSharedPtr interface;
    bool send()
    {
      return interface && interface->send(frame);
    }
  } hb_sender_;
  Timer heartbeat_timer_;

  std::atomic<bool> running_;
  boost::mutex diag_mutex_;

  bool reset_errors_before_recover_;

  void logState(const can::State & s);
  void run();

  virtual void handle_init(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  virtual void handle_recover(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  virtual void handleWrite(LayerStatus &status, const LayerState &current_state);
  virtual void handleShutdown(LayerStatus &status);

  virtual void handle_shutdown(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  virtual void handle_halt(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  // bool handle_get_object(canopen_chain_node::GetObject::Request  &req, canopen_chain_node::GetObject::Response &res);
  // bool handle_set_object(canopen_chain_node::SetObject::Request  &req, canopen_chain_node::SetObject::Response &res);

  bool setup_bus();
  bool setup_sync();
  bool setup_heartbeat();
  bool setup_nodes();
  virtual bool nodeAdded(const canopen::NodeSharedPtr & node, const LoggerSharedPtr & logger);

  void report_diagnostics(diagnostic_updater::DiagnosticStatusWrapper & stat);

  virtual bool setup_chain();

  PublishFuncType createPublishFunc(
    const std::string & name, canopen::NodeSharedPtr node,
    const std::string & key, bool force);

  template<typename Tpub, int dt>
  PublishFuncType createPublisher(
    const std::string & name, ObjectStorageSharedPtr storage,
    const std::string & key, const bool force);

public:
  RosChain();
  bool setup();
  virtual ~RosChain();
};

} //namespace canopen

#endif
