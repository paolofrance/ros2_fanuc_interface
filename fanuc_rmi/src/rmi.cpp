/*
Example of usage of the RMI control mode.
*/

#include "rclcpp/rclcpp.hpp"
#include <fanuc_rmi/rmi_driver.h>
#include <fanuc_rmi/rmi_communication.h>

#include "std_msgs/msg/float32.hpp"
 
#include <unistd.h>


class RMI : public rclcpp::Node
{
  public:
  RMI();


  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr fb_pub_;
  private:
};

RMI::RMI(): Node("rmi")
{
  
  const double control_frequency = 25.0;

  cmd_pub_ = this->create_publisher<std_msgs::msg::Float32>("cmd", 10);
  fb_pub_ = this->create_publisher<std_msgs::msg::Float32>("fb", 10);
}


int main(int argc, char * argv[]) 
{

  rclcpp::init(argc, argv);
  std::shared_ptr< RMI > node = std::make_shared<RMI>();

  int joint_number = 6;
  std::string robot_ip = "10.11.31.101";
  
  rmi::RMIDriver rmi_driver_;

  if (!rmi_driver_.init(robot_ip, joint_number))
    RCLCPP_ERROR_STREAM(node->get_logger(),"RMI non initialized. Robot ip: "<<robot_ip);
  // RCLCPP_INFO_STREAM(node->get_logger(),"RMI  initialized. Robot ip: "<<robot_ip);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread([&executor]() { executor.spin(); }).detach();

  std::vector<double> jpos = rmi_driver_.getPosition();
  // for(auto j:jpos)
    // RCLCPP_INFO_STREAM(node->get_logger(),  "jpos [rad]: " << j);

  double delta = 0.1;

  rclcpp::Rate rate(25);
  rclcpp::Rate rate2(100);

  sleep(1);
  jpos[0]-=0.01;
  rmi_driver_.setTargetPosition(jpos);
  while (!rmi_driver_.instruction_parsed_ && rclcpp::ok())
  {
    rate.sleep();
    // RCLCPP_INFO_STREAM_THROTTLE(node->get_logger(),*node->get_clock(),1000,  "...");
  }
  rmi_driver_.instruction_parsed_=false;
  
  int cnt =0;

  while (rclcpp::ok())
  {    
    cnt++;
    // std::cin.get();
    // jpos[0]+=0.01;
    jpos[0]-=0.01*sin(0.5*cnt);
    // delta*=delta;
    // jpos[0]+=delta;

    rmi_driver_.setTargetPosition(jpos);
    std_msgs::msg::Float32 cmd;
    cmd.data=jpos[0];
    node->cmd_pub_->publish(cmd);

    std::vector<double> jpos_fb = rmi_driver_.getPosition();
    std_msgs::msg::Float32 fb;
    fb.data=jpos_fb[0];
    node->fb_pub_->publish(fb);
    while (!rmi_driver_.instruction_parsed_ && rclcpp::ok())
    {
      rate2.sleep();
      RCLCPP_INFO_STREAM_THROTTLE(node->get_logger(),*node->get_clock(),10000,  "...");
    }
    rmi_driver_.instruction_parsed_=false;

    rate.sleep();

  }

  rmi_driver_.closeConnection();


  rclcpp::shutdown();

  return 0;
}
