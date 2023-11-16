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

  // initialize_storage_vectors(joint_commands_, joint_states_, standard_interfaces_, info_.joints);
  

  joint_position_.assign(6, 0);
  joint_velocities_.assign(6, 0);
  joint_position_command_.assign(6, 0);

  // for (auto i = 0u; i < info_.joints.size(); i++)
  // {
  //   for (auto j = 0u; j < standard_interfaces_.size(); j++)
  //   {
  //     if (std::isnan(joint_states_[j][i]))
  //     {
  //       joint_states_[j][i] = 0.0;
  //     }
  //   }
  // }

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

  // for (size_t i = 0; i < info_.joints.size(); ++i)
  // {
  //   const auto & joint = info_.joints[i];
  //   for (const auto & interface : joint.command_interfaces)
  //   {
  //     if (!get_interface(joint.name, standard_interfaces_, interface.name, i, joint_commands_, command_interfaces))
  //     {
  //         throw std::runtime_error(
  //           "Interface is not found in the standard nor other list. "
  //           "This should never happen!");
  //     }
  //   }
  // }

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

// Private methods
template <typename HandleType>
bool FanucTopic::get_interface(
  const std::string & name, const std::vector<std::string> & interface_list,
  const std::string & interface_name, const size_t vector_index,
  std::vector<std::vector<double>> & values, std::vector<HandleType> & interfaces)
{
  auto it = std::find(interface_list.begin(), interface_list.end(), interface_name);
  if (it != interface_list.end())
  {
    auto j = std::distance(interface_list.begin(), it);
    interfaces.emplace_back(name, *it, &values[j][vector_index]);
    return true;
  }
  return false;
}

void FanucTopic::initialize_storage_vectors(
  std::vector<std::vector<double>> & commands, std::vector<std::vector<double>> & states,
  const std::vector<std::string> & interfaces,
  const std::vector<hardware_interface::ComponentInfo> & component_infos)
{
  // Initialize storage for all joints, regardless of their existence
  commands.resize(interfaces.size());
  states.resize(interfaces.size());
  for (auto i = 0u; i < interfaces.size(); i++)
  {
    commands[i].resize(component_infos.size(), std::numeric_limits<double>::quiet_NaN());
    states[i].resize(component_infos.size(), std::numeric_limits<double>::quiet_NaN());
  }

  // Initialize with values from URDF
  for (auto i = 0u; i < component_infos.size(); i++)
  {
    const auto & component = component_infos[i];
    for (const auto & interface : component.state_interfaces)
    {
      auto it = std::find(interfaces.begin(), interfaces.end(), interface.name);

      // If interface name is found in the interfaces list
      if (it != interfaces.end())
      {
        auto index = std::distance(interfaces.begin(), it);

        // Check the initial_value param is used
        if (!interface.initial_value.empty())
        {
          states[index][i] = parse_double(interface.initial_value);
        }
      }
    }
  }
}

template <typename InterfaceType>
bool FanucTopic::populate_interfaces(
  const std::vector<hardware_interface::ComponentInfo> & components,
  std::vector<std::string> & interface_names, std::vector<std::vector<double>> & storage,
  std::vector<InterfaceType> & target_interfaces, bool using_state_interfaces)
{
  for (auto i = 0u; i < components.size(); i++)
  {
    const auto & component = components[i];
    const auto interfaces =
      (using_state_interfaces) ? component.state_interfaces : component.command_interfaces;
    for (const auto & interface : interfaces)
    {
      if (!get_interface(
            component.name, interface_names, interface.name, i, storage, target_interfaces))
      {
        return false;
      }
    }
  }

  return true;
}
}  // namespace mock_components

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(fanuc::FanucTopic, hardware_interface::SystemInterface)