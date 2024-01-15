#include <EIPScanner/MessageRouter.h>
#include <EIPScanner/utils/Logger.h>
#include <EIPScanner/utils/Buffer.h>
#include <iostream>

using eipScanner::SessionInfo;
using eipScanner::MessageRouter;
using namespace eipScanner::cip;
using namespace eipScanner::utils;

// class FanucEIPDriver {
//     public:
//     std::string robot_ip;
//     SessionInfo si = std::make_shared<SessionInfo>(robot_ip, 0xAF12);
//     MessageRouter messageRouter = std::make_shared<MessageRouter>();
    
//     FanucEIPDriver(int ip) {robot_ip = ip;}

//     void read_value(){
//         auto response = messageRouter->sendRequest(si, ServiceCodes::GET_ATTRIBUTE_SINGLE,
//                                                         EPath(0x7E, 0x01, 0x01));

//         if (response.getGeneralStatusCode() == GeneralStatusCodes::SUCCESS) {
//                 Buffer buffer(response.getData());
//                 CipUint vendorId;
//                 buffer >> vendorId;

//                 Logger(LogLevel::INFO) << "Vendor ID is " << vendorId;
//         }
//     };

//     };


int main() {
    Logger::setLogLevel(LogLevel::DEBUG);
    // FanucEIPDriver driver();

    std::string ip = "10.11.31.111";
    auto si = std::make_shared<SessionInfo>(ip, 0xAF12);
    MessageRouter messageRouter;
    
    auto response = messageRouter.sendRequest(si, ServiceCodes::GET_ATTRIBUTE_SINGLE, EPath(0x7E, 0x01, 0x01));

    std::vector<uint8_t> myList = response.getData();

    for (uint8_t l : myList)
    {
        std::cout << l << std::endl;
    }

    int numArrays = myList.size() / 4;

    std::vector<double> full;

    for (int j = 0; j < numArrays; ++j) 
    {
        unsigned char byteArray[4];
        for (int i = 0; i < 4; ++i) {
            byteArray[i] = myList[j * 4 + i];
        }
        float floatValue;
        std::memcpy(&floatValue, byteArray, sizeof(float));
        std::cout << "Valore convertito: " << floatValue << std::endl;
        full.push_back(floatValue);
    }

    std::vector<double> j_val(full.begin()+1, full.begin() + 7);
    std::cout << "j_val[ " << std::endl;
    for(double j:j_val)
        std::cout << ", " << j << std::endl;
    std::cout << "] " << std::endl;

    std::cout << full[1] << std::endl;
    full[1] += 25;
    std::cout << full[1] << std::endl;
    std::cout << full.size() << std::endl;



    std::vector<unsigned char> myByteArray;
    for(auto val : full)
    {
        float floatValue = val;
        std::vector<unsigned char> byteVector(sizeof(float));
        std::memcpy(byteVector.data(), &floatValue, sizeof(float));
        myByteArray.insert(myByteArray.end(), byteVector.begin(), byteVector.end());
    }

    
    std::cout << myByteArray.size() << std::endl;
    std::cout << response.getData().size() << std::endl;

    std::cout << response.getGeneralStatusCode() << std::endl;

    
    auto response2 = messageRouter.sendRequest(si, ServiceCodes::SET_ATTRIBUTE_SINGLE, EPath(0x7C, 0x01, 0x01), response.getData());

    std::cout << std::hex << response2.getGeneralStatusCode() << std::endl;

  if (response2.getGeneralStatusCode() == GeneralStatusCodes::SUCCESS) {
    Logger(LogLevel::INFO) << "Writing is successful";
  } else {
    Logger(LogLevel::ERROR) << "We got error=0x" << std::hex << response2.getGeneralStatusCode();
  }



    return 0;
}




