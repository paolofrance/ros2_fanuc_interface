#include <fanuc_eth_ip/fanuc_eth_ip.hpp>
#include <iostream>
#include <cassert>
#include <math.h>


fanuc_eth_ip::fanuc_eth_ip(std::string ip)
{ 
  ip_=ip;

  try
  {
    si_.reset( new eipScanner::SessionInfo( ip_, 0xAF12 ) );
  }
  catch(const std::exception& e)
  {
    std::cerr << e.what() << 'connection not available. check ip\n';
    Logger(LogLevel::ERROR) << "\033[1;31m \n\n\n connection not available. check ip \n\n\n\033[0m\n";
    return;
  }
  

  Logger::setLogLevel(LogLevel::ERROR);
}

fanuc_eth_ip::~fanuc_eth_ip()
{
}
std::vector<double> fanuc_eth_ip::get_current_joint_pos()
{
  std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
  auto response = messageRouter_->sendRequest(si_, ServiceCodes::GET_ATTRIBUTE_SINGLE, EPath(0x7E, 0x01, 0x01));

  std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
  // std::cout << "READ time:  = " << std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count() << "[microseconds]" << std::endl <<std::flush;

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
std::vector<double> fanuc_eth_ip::get_current_pose()
{
  std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
  auto response = messageRouter_->sendRequest(si_, ServiceCodes::GET_ATTRIBUTE_SINGLE, EPath(0x7D, 0x01, 0x01));

  std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
  // std::cout << "READ time:  = " << std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count() << "[microseconds]" << std::endl <<std::flush;

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

  std::vector<double> cart_val(full.begin()+1, full.begin() + 7);
      
  return cart_val;
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

  std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
  auto response2 = messageRouter_->sendRequest(si_, ServiceCodes::SET_ATTRIBUTE_SINGLE, EPath(0x7C, 0x01, reg), buffer.data());

  std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
  // std::cout << "WRITE time:  = " << std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count() << "[microseconds]"  << std::endl <<std::flush;
}

// WRITEs value on Digital Inputs - 151+, mapped with the DPM, activate DPM
bool fanuc_eth_ip::write_DI(const Buffer buffer)
{
  // std::cout << "[fanuc_eth_ip::write_DI] buffer size: " << buffer.data().size() << std::endl;
  auto response2 = messageRouter_->sendRequest(si_, ServiceCodes::SET_ATTRIBUTE_SINGLE, EPath(0x04, 151, 0x03), buffer.data());
  return true;
}

// WRITE FLAG
bool fanuc_eth_ip::write_flag(const bool val, const int pos)
{
  Buffer buffer;

  buffer  << CipBool( true );

  auto response = messageRouter_->sendRequest(si_, ServiceCodes::SET_ATTRIBUTE_SINGLE, EPath(0x04, 0x342, 0x03), buffer.data());
  // auto response = messageRouter_->sendRequest(si_, ServiceCodes::GET_ATTRIBUTE_SINGLE, EPath(0x04, 0x342, 0x03));
  // auto response = messageRouter_->sendRequest(si_, ServiceCodes::GET_ATTRIBUTE_SINGLE, EPath(0x04, 0x321, 0x03), buffer.data());



  if (response.getGeneralStatusCode() == GeneralStatusCodes::SUCCESS) {
    Buffer buffer(response.getData());
    CipUint vendorId;
    buffer >> vendorId;

    Logger(LogLevel::ERROR) << "size  " << buffer.size();
    Logger(LogLevel::ERROR) << "response " << vendorId;
  } else {
    Logger(LogLevel::ERROR) << __LINE__ <<"We____ got error=0x" << std::hex << response.getGeneralStatusCode();
  }

std::vector<uint8_t> data = response.getData();
  for (auto d:data)
    Logger(LogLevel::ERROR) << __LINE__ <<"data" << d ;


  return true;
}
  
// write DPM values
// TODO:: multiply by the scaling value to make it ok with the dpm mm/rad
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
  buffer  << CipInt( 0 );
  write_DI(buffer);

  return true;
}
bool fanuc_eth_ip::writeDPMScaled(const std::vector<double> vals, const double scale)
{
  if(vals.size()!=6)
  {
    Logger(LogLevel::ERROR) << "expected vector of size 6! got: " << vals.size();
    return false;
  }

  std::vector<int> v(6);
  for (int i=0;i<vals.size();i++)
    v.at(i)=vals.at(i)/scale;

  writeDPM(v);

  return true;
}
// Activate DPM
void fanuc_eth_ip::activateDPM(const bool activate)
{
  
  setCurrentPos();
  
  if(activate)
    write_register(2,1);
  else
    write_register(1,1);

  int act = (activate) ? 0 : 1;
  Buffer buffer;
  for(int i=0;i<6;i++)
    buffer  << CipInt( 0 );
  buffer  << CipInt( act );
  write_DI(buffer);
}
// Deactivate DPM
void fanuc_eth_ip::deactivateDPM(){activateDPM(false);}

void fanuc_eth_ip::setCurrentPos() {write_pos_register(get_current_joint_pos());}




