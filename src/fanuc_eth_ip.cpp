#include <ros2_fanuc_interface/fanuc_eth_ip.h>
#include <iostream>
#include <cassert>
#include <math.h>


fanuc_eth_ip::fanuc_eth_ip(std::string ip)
{ 
  ip_=ip;
  si_.reset( new eipScanner::SessionInfo( ip_, 0xAF12 ) );
  Logger::setLogLevel(LogLevel::ERROR);
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
    for (int i = 0; i < 4; ++i)
      byteArray[i] = myList[j * 4 + i];

    float floatValue;
    std::memcpy(&floatValue, byteArray, sizeof(float));
    full.push_back(floatValue);
  }

  std::vector<double> j_val(full.begin()+1, full.begin() + 7);
      
  for (int i=0;i<j_val.size();i++)
    j_val.at(i) *= M_PI/180.0;

  j_val.at(2) += ( j_val[1]);

  return j_val;
}

// WRITEs value on a REGULAR REGISTER 
void fanuc_eth_ip::write_register(const int val, const int reg)
{
  Buffer buffer;
  CipDint arg = val;
  buffer << arg;
  auto response = messageRouter_->sendRequest(si_, ServiceCodes::SET_ATTRIBUTE_SINGLE, EPath(0x6B, 0x01, reg), buffer.data());
}

// WRITEs value on a POSITION REGISTER 
void fanuc_eth_ip::write_pos_register(std::vector<double> j_vals, const int reg)
{
  for (int i=0;i<j_vals.size();i++)
    j_vals.at(i) *= 180.0/M_PI;

  j_vals.at(2) -= ( j_vals[1]);

  
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
  
  // std::cout << "[fanuc_eth_ip::write_pos_register] buffer size: " << buffer.data().size() << std::endl;

  auto response2 = messageRouter_->sendRequest(si_, ServiceCodes::SET_ATTRIBUTE_SINGLE, EPath(0x7C, 0x01, reg), buffer.data());
}

// WRITEs value on Digital Inputs - 151+, mapped with the DPM, activate DPM
bool fanuc_eth_ip::write_DI(const Buffer buffer)
{
  // std::cout << "[fanuc_eth_ip::write_DI] buffer size: " << buffer.data().size() << std::endl;
  auto response2 = messageRouter_->sendRequest(si_, ServiceCodes::SET_ATTRIBUTE_SINGLE, EPath(0x04, 151, 0x03), buffer.data());
}

// write DPM values
bool fanuc_eth_ip::writeDPM(const std::vector<int> vals)
{
  if(vals.size()!=6)
  {
    Logger(LogLevel::ERROR) << "expected vector of size 6! got: " << vals.size();
    return false;
  }   
  Buffer buffer;

  for(auto v:vals)
    buffer  << CipInt( v );
  buffer  << CipInt( 1 );
  write_DI(buffer);

  return true;
}
// Activate DPM
void fanuc_eth_ip::activateDPM(const bool activate)
{
  Buffer buffer;
  for(int i=0;i<6;i++)
    buffer  << CipInt( 0 );
  buffer  << CipInt( 0 );
  write_DI(buffer);
}
// Deactivate DPM
void fanuc_eth_ip::deactivateDPM(){activateDPM(false);}





