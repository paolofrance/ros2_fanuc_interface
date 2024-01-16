#include <ros2_fanuc_interface/fanuc_eth_ip.h>
#include <iostream>
#include <cassert>
fanuc_eth_ip::fanuc_eth_ip(std::string ip)
{ 
    Logger(LogLevel::ERROR) << "quiiiiiii: " ;
    ip_=ip;
    si_.reset( new eipScanner::SessionInfo( ip_, 0xAF12 ) );
    Logger(LogLevel::ERROR) << "quiiiiiii22222222222: " ;
}

fanuc_eth_ip::~fanuc_eth_ip()
{
}
std::vector<double> fanuc_eth_ip::get_current_joint_pos()
{
    auto response = messageRouter_->sendRequest(si_, ServiceCodes::GET_ATTRIBUTE_SINGLE, EPath(0x7E, 0x01, 0x01));

    std::vector<uint8_t> myList = response.getData();
    int numArrays = myList.size() / 4;

    std::vector<float> full;

    for (int j = 0; j < numArrays; ++j) 
    {
        unsigned char byteArray[4];
        for (int i = 0; i < 4; ++i) {
            byteArray[i] = myList[j * 4 + i];
        }
        float floatValue;
        std::memcpy(&floatValue, byteArray, sizeof(float));
        full.push_back(floatValue);
    }

    std::vector<double> j_val(full.begin()+1, full.begin() + 7);
    return j_val;
}

// WRITEs value on a REGULAR REGISTER 
void fanuc_eth_ip::write_register(int val, int reg)
{
    Buffer buffer;
    CipDint arg = val;
    buffer << arg;
    std::cout << buffer.data().size() << std::endl;

    auto response3 = messageRouter_->sendRequest(si_, ServiceCodes::SET_ATTRIBUTE_SINGLE, EPath(0x6B, 0x01, reg), buffer.data());

}


// WRITEs value on a POSITION REGISTER 
void fanuc_eth_ip::write_pos_register(std::vector<double> j_vals, int reg)
{
    
    Buffer buffer;
    buffer  << CipReal(  0.0  )
            << CipReal( static_cast<float>( j_vals[0] ) )
            << CipReal( static_cast<float>( j_vals[1] ) )
            << CipReal( static_cast<float>( j_vals[2] ) )
            << CipReal( static_cast<float>( j_vals[3] ) )
            << CipReal( static_cast<float>( j_vals[4] ) )
            << CipReal( static_cast<float>( j_vals[5] ) )
            << CipReal(  0.0  )
            << CipReal(  0.0  )
            << CipReal(  0.0  ) ;
    
    std::cout << "[fanuc_eth_ip::write_pos_register] buffer size: " << buffer.data().size() << std::endl;

    auto response2 = messageRouter_->sendRequest(si_, ServiceCodes::SET_ATTRIBUTE_SINGLE, EPath(0x7C, 0x01, reg), buffer.data());
}
// WRITEs value on a Digital Input
void fanuc_eth_ip::write_DI(const std::vector<int> vals)
{
    Buffer buffer;

    for(auto v:vals)
        buffer  << CipInt( v );

    std::cout << "[fanuc_eth_ip::write_DI] buffer size: " << buffer.data().size() << std::endl;
    auto response2 = messageRouter_->sendRequest(si_, ServiceCodes::SET_ATTRIBUTE_SINGLE, EPath(0x04, 151, 0x03), buffer.data());
}


int main() {
    
    // creates the driver object
    fanuc_eth_ip driver("10.11.31.111");

    // reads joint position
    std::vector<double> j_pos = driver.get_current_joint_pos();

    for(auto j:j_pos)
        Logger(LogLevel::ERROR) << "jpos: " << j;


    // writes on a register
    driver.write_register(55,7);

    // writes on a position register
    driver.write_pos_register(j_pos);

    // writes on a Digital Input
    std::vector<int> v = {1, 2, 3, 4};
    driver.write_DI(v);


    return 0;
}




