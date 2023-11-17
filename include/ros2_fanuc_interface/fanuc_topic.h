
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

using hardware_interface::return_type;

namespace fanuc
{
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

static constexpr size_t POSITION_INTERFACE_INDEX = 0;
static constexpr size_t VELOCITY_INTERFACE_INDEX = 1;


// class PubSubNode : public rclcpp::Node
// {
//   public:
//     PubSubNode();
//     void publishData();

//   private:
//     rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr cmd_publisher_;

// };


class JointComms : public rclcpp::Node
{
  public:
    JointComms();

    void sendCommand(std::vector<std::string> joint_names, std::vector<double> joint_pos);

    std::vector<double> getPosition();
    std::vector<double> getVelocity();
    bool first_feedback_received_;
  private:
    std::vector<double> pos_;
    std::vector<double> vel_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr fb_sub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr cmd_pub_;
    void stateCB(const sensor_msgs::msg::JointState state);
};






class HARDWARE_INTERFACE_PUBLIC FanucTopic : public hardware_interface::SystemInterface
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

  // /// The size of this vector is (standard_interfaces_.size() x nr_joints)
  // std::vector<std::vector<double>> joint_commands_;
  // std::vector<std::vector<double>> joint_states_;

  std::vector<double> joint_position_command_;
  std::vector<double> joint_position_;
  std::vector<double> joint_velocities_;

private:
  rclcpp::Logger logger_ = rclcpp::get_logger("fanuc_topic_hw");
  rclcpp::executors::SingleThreadedExecutor executor_;
  std::shared_ptr<JointComms> comms_;
  std::vector<std::string> joint_names_; 
  std::vector<double>      joint_pos_  ;

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

typedef FanucTopic GenericRobot;

}  // namespace mock_components