#include "ros2_fanuc_interface/fanuc_hw.h"

#include <algorithm>
#include <charconv>
#include <cmath>
#include <iterator>
#include <limits>
#include <set>
#include <string>
#include <vector>

#include "hardware_interface/component_parser.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rcutils/logging_macros.h"
#include "rclcpp/rclcpp.hpp"

#include<unistd.h>


namespace fanuc
{

JointComms::JointComms() : Node("fanuc_hw")
{
  cmd_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/cmd_j_pos",10);
  fb_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/fb_j_pos",10);
}





fanuc_eth_ip::fanuc_eth_ip(std::string ip)
{ 
  Logger::setLogLevel(LogLevel::ERROR);
  ip_=ip;
  si_.reset( new eipScanner::SessionInfo( ip_, 0xAF12 ) );
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
    
  for (int i=0;i<j_val.size();i++)
    j_val.at(i) *= M_PI/180.0;

  j_val.at(2) += ( j_val[1]);

  return j_val;
}
// WRITEs value on a REGULAR REGISTER 
void fanuc_eth_ip::write_register(int val, int reg)
{
    Buffer buffer;
    CipDint arg = val;
    buffer << arg;
    auto response3 = messageRouter_->sendRequest(si_, ServiceCodes::SET_ATTRIBUTE_SINGLE, EPath(0x6B, 0x01, reg), buffer.data());
}
// WRITEs value on a POSITION REGISTER 
void fanuc_eth_ip::write_pos_register(std::vector<double> j_vals, int reg)
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

  auto response2 = messageRouter_->sendRequest(si_, 0X10, EPath(0x7C, 0x01, reg), buffer.data());
}






double parse_double(const std::string & text)
{
  double result_value;
  const auto parse_result = std::from_chars(text.data(), text.data() + text.size(), result_value);
  if (parse_result.ec == std::errc())
  {
    return result_value;
  }

  return 0.0;
}

CallbackReturn FanucHw::on_init(const hardware_interface::HardwareInfo & info)
{

  RCLCPP_INFO(logger_, "init fanuc_hw");
  
  //TODO:: must from params
  std::string robot_ip = "10.11.31.111";
  
  EIP_driver_.reset( new fanuc_eth_ip (robot_ip) );

  RCLCPP_INFO_STREAM(logger_,"Initialized robot driver at ip: " << robot_ip );
  
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
  {
    return CallbackReturn::ERROR;
  }

  joint_position_.assign(6, 0);
  joint_velocities_.assign(6, 0);
  joint_position_command_.assign(6, 0); 

  joint_names_.resize(joint_position_.size());
  for (size_t j = 0; j < joint_position_.size(); ++j)
  {
    joint_names_.at(j) = info_.joints[j].name;
    RCLCPP_DEBUG_STREAM(logger_,info_.joints[j].name);
  }

  std::vector<double> j_pos = EIP_driver_->get_current_joint_pos();

  for(size_t i=0;i<joint_position_.size();i++)
  {
    joint_position_command_.at(i) = j_pos.at(i);
    RCLCPP_DEBUG_STREAM(logger_,joint_position_command_.at(i));
  }
    
  EIP_driver_->write_register(1,1);
  EIP_driver_->write_pos_register(joint_position_command_);
  
  comms_ = std::make_shared<JointComms>();
  executor_.add_node(comms_);
  std::thread([this]() { executor_.spin(); }).detach();

  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> FanucHw::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  state_interfaces.emplace_back(info_.joints[0].name, "position", &joint_position_[0]);
  state_interfaces.emplace_back(info_.joints[1].name, "position", &joint_position_[1]);
  state_interfaces.emplace_back(info_.joints[2].name, "position", &joint_position_[2]);
  state_interfaces.emplace_back(info_.joints[3].name, "position", &joint_position_[3]);
  state_interfaces.emplace_back(info_.joints[4].name, "position", &joint_position_[4]);
  state_interfaces.emplace_back(info_.joints[5].name, "position", &joint_position_[5]);

  state_interfaces.emplace_back(info_.joints[0].name, "velocity", &joint_velocities_[0]);
  state_interfaces.emplace_back(info_.joints[1].name, "velocity", &joint_velocities_[1]);
  state_interfaces.emplace_back(info_.joints[2].name, "velocity", &joint_velocities_[2]);
  state_interfaces.emplace_back(info_.joints[3].name, "velocity", &joint_velocities_[3]);
  state_interfaces.emplace_back(info_.joints[4].name, "velocity", &joint_velocities_[4]);
  state_interfaces.emplace_back(info_.joints[5].name, "velocity", &joint_velocities_[5]);

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> FanucHw::export_command_interfaces()
{

  std::vector<hardware_interface::CommandInterface> command_interfaces;


  command_interfaces.emplace_back(info_.joints[0].name, "position", &joint_position_command_[0]);
  command_interfaces.emplace_back(info_.joints[1].name, "position", &joint_position_command_[1]);
  command_interfaces.emplace_back(info_.joints[2].name, "position", &joint_position_command_[2]);
  command_interfaces.emplace_back(info_.joints[3].name, "position", &joint_position_command_[3]);
  command_interfaces.emplace_back(info_.joints[4].name, "position", &joint_position_command_[4]);
  command_interfaces.emplace_back(info_.joints[5].name, "position", &joint_position_command_[5]);

  return command_interfaces;
}

return_type FanucHw::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  std::vector<double> jp = EIP_driver_->get_current_joint_pos();


  for (size_t j = 0; j < joint_position_command_.size(); ++j)
  {
    joint_position_[j] = jp[j];
    joint_velocities_[j] = 0.0; // TODO: not implemented yet

    RCLCPP_DEBUG_STREAM(logger_,jp[j]);
  }  
  
  auto msg = sensor_msgs::msg::JointState();
  msg.header.stamp = comms_->get_clock()->now();
  msg.name = joint_names_;
  msg.position = joint_position_;
  msg.velocity = joint_velocities_;
  comms_->fb_pub_->publish(msg);
  
  return return_type::OK;
}

return_type FanucHw::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  EIP_driver_->write_pos_register(joint_position_command_);
  
  auto msg = sensor_msgs::msg::JointState();
  msg.header.stamp = comms_->get_clock()->now();
  msg.name = joint_names_;
  msg.position = joint_position_command_;
  comms_->cmd_pub_->publish(msg);

  return return_type::OK;
}


}  // namespace mock_components

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(fanuc::FanucHw, hardware_interface::SystemInterface)