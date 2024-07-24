# pragma once

#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include <rclcpp/logger.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/executors.hpp>
#include "sensor_msgs/msg/joint_state.hpp"

#include <fanuc_eth_ip/fanuc_eth_ip.hpp>
#include <fanuc_rmi/rmi_driver.h>

using hardware_interface::return_type;

namespace fanuc
{
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

static constexpr size_t POSITION_INTERFACE_INDEX = 0;
static constexpr size_t VELOCITY_INTERFACE_INDEX = 1;




class JointComms : public rclcpp::Node
{
  public:
    JointComms();

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr cmd_pub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr fb_pub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr cart_fb_pub_;
  private:
};



class HARDWARE_INTERFACE_PUBLIC FanucHw : public hardware_interface::SystemInterface
{
public:
  CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;

  return_type write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) override;

protected:

  const std::vector<std::string> standard_interfaces_ = {
  hardware_interface::HW_IF_POSITION, hardware_interface::HW_IF_VELOCITY,
  hardware_interface::HW_IF_ACCELERATION, hardware_interface::HW_IF_EFFORT};

  std::vector<double> joint_position_command_;
  std::vector<double> joint_position_;
  std::vector<double> joint_velocities_;

private:
  rclcpp::Logger logger_ = rclcpp::get_logger("fanuc_hw");
  rclcpp::executors::SingleThreadedExecutor executor_;
  std::shared_ptr<JointComms> comms_;
  std::vector<std::string> joint_names_; 
  std::vector<double>      joint_pos_  ;
  std::shared_ptr<fanuc_eth_ip> EIP_driver_;
  rmi::RMIDriver rmi_driver_;

  bool useRMI_;

  bool read_only_;

  template <typename HandleType>
  bool get_interface(
    const std::string & name, const std::vector<std::string> & interface_list,
    const std::string & interface_name, const size_t vector_index,
    std::vector<std::vector<double>> & values, std::vector<HandleType> & interfaces);

  void initialize_storage_vectors(
    std::vector<std::vector<double>> & commands, std::vector<std::vector<double>> & states,
    const std::vector<std::string> & interfaces,
    const std::vector<hardware_interface::ComponentInfo> & component_infos);

  template <typename InterfaceType>
  bool populate_interfaces(
    const std::vector<hardware_interface::ComponentInfo> & components,
    std::vector<std::string> & interfaces, std::vector<std::vector<double>> & storage,
    std::vector<InterfaceType> & target_interfaces, bool using_state_interfaces);

};

typedef FanucHw GenericRobot;

}  // namespace mock_components
