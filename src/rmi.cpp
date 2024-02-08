/*
Example of usage of the RMI control mode.
*/

#include "rclcpp/rclcpp.hpp"
#include <ros2_fanuc_interface/rmi_driver.h>

#include "std_msgs/msg/float32.hpp"
 


class RMI : public rclcpp::Node
{
  public:
  RMI();

  rmi::RMIDriver rmi_driver_;

  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr fb_pub_;
  private:
};

RMI::RMI(): Node("rmi")
{
  
  const double control_frequency = 50.0;

  int joint_number = 6;
  std::string robot_ip = "10.11.31.111";

  if (!rmi_driver_.init(robot_ip, joint_number))
    RCLCPP_ERROR_STREAM(this->get_logger(),"RMI non initialized. Robot ip: "<<robot_ip);
  RCLCPP_INFO_STREAM(this->get_logger(),"RMI  initialized. Robot ip: "<<robot_ip);
  
  cmd_pub_ = this->create_publisher<std_msgs::msg::Float32>("cmd", 10);
  fb_pub_ = this->create_publisher<std_msgs::msg::Float32>("fb", 10);
}


int main(int argc, char * argv[]) 
{
  
  rclcpp::init(argc, argv);
  std::shared_ptr< RMI > node = std::make_shared<RMI>();

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread([&executor]() { executor.spin(); }).detach();

  std::vector<double> jpos = node->rmi_driver_.getPosition();
  for(auto j:jpos)
    RCLCPP_INFO_STREAM(node->get_logger(),  "jpos [rad]: " << j);

  rclcpp::Rate rate(1);

  // jpos[0]=-jpos[0];

  node->rmi_driver_.setMode(rmi::Mode::IDLE);

  rclcpp::sleep_for(std::chrono::nanoseconds(100000));

  node->rmi_driver_.setTargetPosition(jpos);

  // node->rmi_driver_.setMode(rmi::Mode::POSITION);
  
  while (rclcpp::ok())
  {
    jpos[0]+=0.01;

    std::vector<double> jpos_fb = node->rmi_driver_.getPosition();
    node->rmi_driver_.setTargetPosition(jpos);



    rate.sleep();

  }


  rclcpp::shutdown();

  return 0;
}
