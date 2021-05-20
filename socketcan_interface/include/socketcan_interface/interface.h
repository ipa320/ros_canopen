#ifndef H_CAN_INTERFACE
#define H_CAN_INTERFACE

#include <array>
#include <memory>
#include <functional>

#include <boost/system/error_code.hpp>

#include "socketcan_interface/delegates.h"
#include "socketcan_interface/logging.h"
#include "socketcan_interface/settings.h"

namespace can{

/** Header for CAN id an meta data*/
struct Header{
    static const unsigned int ID_MASK = (1u << 29)-1;
    static const unsigned int ERROR_MASK = (1u << 29);
    static const unsigned int RTR_MASK = (1u << 30);
    static const unsigned int EXTENDED_MASK = (1u << 31);

    unsigned int id:29; ///< CAN ID (11 or 29 bits valid, depending on is_extended member
    unsigned int is_error:1; ///< marks an error frame (only used internally)
    unsigned int is_rtr:1; ///< frame is a remote transfer request
    unsigned int is_extended:1; ///< frame uses 29 bit CAN identifier
    bool is_fd; ///< frame is a CAN FD frame
    unsigned char flags; ///< additional flags for CAN FD
    /** check if frame header is valid*/
    bool isValid() const{
        return id < (is_extended?(1<<29):(1<<11));
    }
    unsigned int fullid() const { return id | (is_error?ERROR_MASK:0) | (is_rtr?RTR_MASK:0) | (is_extended?EXTENDED_MASK:0); }
    unsigned int key() const { return is_error ? (ERROR_MASK) : fullid(); }
    [[deprecated("use key() instead")]] explicit operator unsigned int() const { return key(); }

    /** constructor with default parameters
        * @param[in] i: CAN id, defaults to 0
        * @param[in] extended: uses 29 bit identifier, defaults to false
        * @param[in] rtr: is rtr frame, defaults to false
        */

    Header()
    : id(0),is_error(0),is_rtr(0), is_extended(0), is_fd(false), flags(0) {}

    Header(unsigned int i, bool extended, bool rtr, bool error, bool is_fd=false, unsigned char flags=0)
    : id(i),is_error(error?1:0),is_rtr(rtr?1:0), is_extended(extended?1:0), is_fd(is_fd), flags(flags) {}
};


struct MsgHeader : public Header{
    MsgHeader(unsigned int i=0, bool rtr = false) : Header(i, false, rtr, false) {}
};
struct ExtendedHeader : public Header{
    ExtendedHeader(unsigned int i=0, bool rtr = false) : Header(i, true, rtr, false) {}
};
struct ErrorHeader : public Header{
    ErrorHeader(unsigned int i=0) : Header(i, false, false, true) {}
};



/** representation of a CAN frame */
struct Frame: public Header{
    using value_type = unsigned char;
    std::array<value_type, 64> data; ///< array for 64 data bytes with bounds checking
    unsigned char dlc; ///< len of data

    /** check if frame header and length are valid*/
    bool isValid() const{
        return (dlc <= data.size())  &&  Header::isValid();
    }
    /**
     * constructor with default parameters
     * @param[in] i: CAN id, defaults to 0
     * @param[in] l: number of data bytes, defaults to 0
     * @param[in] extended: uses 29 bit identifier, defaults to false
     * @param[in] rtr: is rtr frame, defaults to false
     */
    Frame() : Header(), dlc(0) {}
    Frame(const Header &h, unsigned char l = 0) : Header(h), dlc(l) {}

    value_type * c_array() { return data.data(); }
    const value_type * c_array() const { return data.data(); }
};

/** extended error information */
class State{
public:
    enum DriverState{
        closed, open, ready
    } driver_state;
    boost::system::error_code error_code; ///< device access error
    unsigned int internal_error; ///< driver specific error

    State() : driver_state(closed), internal_error(0) {}
    virtual bool isReady() const { return driver_state == ready; }
    virtual ~State() {}
};

/** template for Listener interface */
template <typename T,typename U> class Listener{
    const T callable_;
public:
    using Type = U;
    using Callable = T;
    using ListenerConstSharedPtr = std::shared_ptr<const Listener>;

    Listener(const T &callable):callable_(callable){ }
    void operator()(const U & u) const { if(callable_) callable_(u); }
    virtual ~Listener() {}
};

class StateInterface{
public:
    using StateFunc = std::function<void(const State&)>;
    using StateDelegate [[deprecated("use StateFunc instead")]] = DelegateHelper<StateFunc>;
    using StateListener = Listener<const StateFunc, const State&>;
    using StateListenerConstSharedPtr = StateListener::ListenerConstSharedPtr;

    /**
     * acquire a listener for the specified delegate, that will get called for all state changes
     *
     * @param[in] delegate: delegate to be bound by the listener
     * @return managed pointer to listener
     */
    virtual StateListenerConstSharedPtr createStateListener(const StateFunc &delegate) = 0;
    template <typename Instance, typename Callable> inline StateListenerConstSharedPtr createStateListenerM(Instance inst, Callable callable) {
        return this->createStateListener(std::bind(callable, inst, std::placeholders::_1));
    }

    virtual ~StateInterface() {}
};
using StateInterfaceSharedPtr = std::shared_ptr<StateInterface>;
using StateListenerConstSharedPtr = StateInterface::StateListenerConstSharedPtr;

class CommInterface{
public:
    using FrameFunc = std::function<void(const Frame&)>;
    using FrameDelegate [[deprecated("use FrameFunc instead")]] = DelegateHelper<FrameFunc>;
    using FrameListener = Listener<const FrameFunc, const Frame&>;
    using FrameListenerConstSharedPtr = FrameListener::ListenerConstSharedPtr;

    /**
     * enqueue frame for sending
     *
     * @param[in] msg: message to be enqueued
     * @return true if frame was enqueued succesfully, otherwise false
     */
    virtual bool send(const Frame & msg) = 0;

    /**
     * acquire a listener for the specified delegate, that will get called for all messages
     *
     * @param[in] delegate: delegate to be bound by the listener
     * @return managed pointer to listener
     */
    virtual FrameListenerConstSharedPtr createMsgListener(const FrameFunc &delegate) = 0;
    template <typename Instance, typename Callable> inline FrameListenerConstSharedPtr createMsgListenerM(Instance inst, Callable callable) {
        return this->createMsgListener(std::bind(callable, inst, std::placeholders::_1));
    }

    /**
     * acquire a listener for the specified delegate, that will get called for messages with demanded ID
     *
     * @param[in] header: CAN header to restrict listener on
     * @param[in] delegate: delegate to be bound listener
     * @return managed pointer to listener
     */
    virtual FrameListenerConstSharedPtr createMsgListener(const Frame::Header&, const FrameFunc &delegate) = 0;
    template <typename Instance, typename Callable> inline FrameListenerConstSharedPtr createMsgListenerM(const Frame::Header& header, Instance inst, Callable callable) {
        return this->createMsgListener(header, std::bind(callable, inst, std::placeholders::_1));
    }

    virtual ~CommInterface() {}
};
using CommInterfaceSharedPtr = std::shared_ptr<CommInterface>;
using FrameListenerConstSharedPtr = CommInterface::FrameListenerConstSharedPtr;

class DriverInterface : public CommInterface, public StateInterface {
public:
    [[deprecated("provide settings explicitly")]]  virtual bool init(const std::string &device, bool loopback) = 0;

    /**
     * initialize interface
     *
     * @param[in] device: driver-specific device name/path
     * @param[in] loopback: loop-back own messages
     * @param[in] settings: driver-specific settings
     * @return true if device was initialized succesfully, false otherwise
     */
    virtual bool init(const std::string &device, bool loopback, SettingsConstSharedPtr settings) {
        ROSCANOPEN_ERROR("socketcan_interface", "Driver does not support custom settings");
        #pragma GCC diagnostic push
        #pragma GCC diagnostic ignored "-Wdeprecated-declarations"
        return init(device, loopback);
        #pragma GCC diagnostic pop
    }

    /**
     * Recover interface after errors and emergency stops
     *
     * @return true if device was recovered succesfully, false otherwise
     */
    virtual bool recover() = 0;

    /**
     * @return current state of driver
     */
    virtual State getState() = 0;

    /**
     * shutdown interface
     *
     * @return true if shutdown was succesful, false otherwise
     */
    virtual void shutdown() = 0;

    virtual bool translateError(unsigned int internal_error, std::string & str) = 0;

    virtual bool doesLoopBack() const = 0;

    virtual void run()  = 0;

    virtual ~DriverInterface() {}
};
using DriverInterfaceSharedPtr = std::shared_ptr<DriverInterface>;


} // namespace can

#endif
