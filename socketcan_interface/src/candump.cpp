#include <iostream>
#include <memory>
#include <unordered_set>

#include <boost/exception/diagnostic_information.hpp>
#include <class_loader/class_loader.hpp>
#include <socketcan_interface/socketcan.h>

using namespace can;

#include <iostream>

void print_frame(const Frame &f){

    if(f.is_error){
        std::cout << "E " << std::hex << f.id << std::dec;
    }else if(f.is_extended){
        std::cout << "e " << std::hex << f.id << std::dec;
    }else if(f.is_fd){
        std::cout << "f " << std::hex << f.id << std::dec;
    }else{
        std::cout << "s " << std::hex << f.id << std::dec;
    }

    std::cout << "\t";

    if(f.is_rtr){
        std::cout << "r";
    }else{
        std::cout << (int) f.dlc << std::hex;

        for(int i=0; i < f.dlc; ++i){
            std::cout << std::hex << " " << (int) f.data[i];
        }
    }

    std::cout << std::dec << std::endl;
}


std::shared_ptr<class_loader::ClassLoader> g_loader;
DriverInterfaceSharedPtr g_driver;

void print_state(const State & s){
    std::string err;
    g_driver->translateError(s.internal_error,err);
    std::cout << "STATE: driver_state=" << s.driver_state << " internal_error=" << s.internal_error << "('" << err << "') asio: " << s.error_code << std::endl;
}


int main(int argc, char *argv[]){

    if(argc != 2 && argc != 4){
        std::cout << "usage: "<< argv[0] << " DEVICE [PLUGIN_PATH PLUGIN_NAME]" << std::endl;
        return 1;
    }

    if(argc == 4 ){
        try
        {
            g_loader = std::make_shared<class_loader::ClassLoader>(argv[2]);
            g_driver = g_loader->createUniqueInstance<DriverInterface>(argv[3]);
        }

        catch(std::exception& ex)
        {
            std::cerr << boost::diagnostic_information(ex) << std::endl;;
            return 1;
        }
    }else{
        g_driver = std::make_shared<SocketCANInterface>();
    }



    FrameListenerConstSharedPtr frame_printer = g_driver->createMsgListener(print_frame);
    StateListenerConstSharedPtr error_printer = g_driver->createStateListener(print_state);

    if(!g_driver->init(argv[1], false, can::NoSettings::create())){
        print_state(g_driver->getState());
        return 1;
    }

    g_driver->run();

    g_driver->shutdown();
    g_driver.reset();
    g_loader.reset();

    return 0;

}
