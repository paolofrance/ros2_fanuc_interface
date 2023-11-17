#include "ros2_fanuc_interface/fanuc_topic.h"

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

JointComms::JointComms() : Node("joint_comms")
  {
    first_feedback_received_ = false;
    cmd_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/cmd_j_pos",10);
    fb_sub_ = this->create_subscription<sensor_msgs::msg::JointState>("/fb_j_pos",10,std::bind(&JointComms::stateCB, this, std::placeholders::_1));
  }

  void JointComms::stateCB(const sensor_msgs::msg::JointState state)
  {
    first_feedback_received_ = true;

    pos_ = state.position;
    vel_ = state.velocity;
    for(size_t i=0;i<pos_.size();i++)
      RCLCPP_DEBUG_STREAM(this->get_logger(),pos_[i]);
  };

  void JointComms::sendCommand(std::vector<std::string> joint_names, std::vector<double> joint_pos)
  {
    auto msg = sensor_msgs::msg::JointState();
    msg.header.stamp = this->get_clock()->now();
    msg.name = joint_names;
    msg.position = joint_pos;
    cmd_pub_->publish(msg);
  }

  std::vector<double> JointComms::getPosition(){return pos_;};
  std::vector<double> JointComms::getVelocity(){return vel_;};




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

CallbackReturn FanucTopic::on_init(const hardware_interface::HardwareInfo & info)
{

  RCLCPP_INFO(logger_, "init fanuc_topic_hw");
  
  comms_ = std::make_shared<JointComms>();
  executor_.add_node(comms_);
  std::thread([this]() { executor_.spin(); }).detach();
    
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

  unsigned int second = 1000000;
  while (!comms_->first_feedback_received_)
  {
    RCLCPP_WARN_THROTTLE(logger_,*comms_->get_clock(), 2000,"waitng for first feedback message");
    usleep(second);
  }

  std::vector<double> jp = comms_->getPosition();

  for(size_t i=0;i<joint_position_.size();i++)
  {
    joint_position_command_.at(i) = jp.at(i);
    RCLCPP_DEBUG_STREAM(logger_,joint_position_command_.at(i));
  }
  
  comms_->sendCommand(joint_names_, joint_position_command_);

  
  
  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> FanucTopic::export_state_interfaces()
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

std::vector<hardware_interface::CommandInterface> FanucTopic::export_command_interfaces()
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

return_type FanucTopic::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  std::vector<double> jp = comms_->getPosition();
  std::vector<double> jv = comms_->getVelocity();
  for (size_t j = 0; j < joint_position_command_.size(); ++j)
  {
    joint_position_[j] = jp[j];
    joint_velocities_[j] = jv[j];

    RCLCPP_DEBUG_STREAM(logger_,jp[j]);
  }
  
  return return_type::OK;
}

return_type FanucTopic::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  comms_->sendCommand(joint_names_, joint_position_command_);
  return return_type::OK;
}


}  // namespace mock_components

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(fanuc::FanucTopic, hardware_interface::SystemInterface)