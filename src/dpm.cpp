/*
Example of usage of the DPM control mode.
This example shows how to listen to a FT sensor and generate motion accordingly
*/


#include <ros2_fanuc_interface/fanuc_eth_ip.h>

#include <unistd.h>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"
#include <kdl/frames.hpp>
#include "std_msgs/msg/float32.hpp"
 


class WrenchSubscriber : public rclcpp::Node
{
  public:
    
    WrenchSubscriber();
    void topic_callback(const geometry_msgs::msg::WrenchStamped::SharedPtr msg);

    bool wrench_init_;
    std::vector<double> meas_wrench_={0,0,0,0,0,0};

    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr cmd_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr fb_publisher_;

  private:
    rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr subscription_;
};

WrenchSubscriber::WrenchSubscriber(): Node("wrench_subscriber")
{

  this->declare_parameter("wrench_topic","/wrench");
  std::string wrench_topic = this->get_parameter("wrench_topic").as_string();

  RCLCPP_INFO_STREAM(this->get_logger(), "subscribed to: " << wrench_topic );

  wrench_init_ = false;

  subscription_ = this->create_subscription<geometry_msgs::msg::WrenchStamped>(
  wrench_topic, 10, std::bind(&WrenchSubscriber::topic_callback, this, std::placeholders::_1));
  cmd_publisher_ = this->create_publisher<std_msgs::msg::Float32>("dpm_cmd", 10);
  fb_publisher_ = this->create_publisher<std_msgs::msg::Float32>("dpm_fb", 10);
}

void WrenchSubscriber::topic_callback(const geometry_msgs::msg::WrenchStamped::SharedPtr msg) 
{
  if (!wrench_init_)
    wrench_init_ = true;

  RCLCPP_DEBUG_STREAM(this->get_logger(), "\n" << geometry_msgs::msg::to_yaml(msg->wrench) );

  meas_wrench_.at(0) = msg->wrench.force.x;
  meas_wrench_.at(1) = msg->wrench.force.y;
  meas_wrench_.at(2) = msg->wrench.force.z;
  meas_wrench_.at(3) = msg->wrench.torque.x;
  meas_wrench_.at(4) = msg->wrench.torque.y;
  meas_wrench_.at(5) = msg->wrench.torque.z;
}


int main(int argc, char * argv[]) 
{
  
  rclcpp::init(argc, argv);
  std::shared_ptr< WrenchSubscriber > node = std::make_shared<WrenchSubscriber>();

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread([&executor]() { executor.spin(); }).detach();


  std::vector<double> wrench_0={0,0,0,0,0,0};
  rclcpp::Clock clock;
  rclcpp::Rate rate(1);
  while (!node->wrench_init_)
  {
    RCLCPP_INFO_THROTTLE(node->get_logger(),clock,1000,"waiting for first wrench message");
    rate.sleep();
  }

   wrench_0=node->meas_wrench_;

  for (auto w: wrench_0)
    RCLCPP_INFO_STREAM(node->get_logger(),"wrench_0: " << w);
  

  // creates the driver object
  fanuc_eth_ip driver("10.11.31.111");

  // reads joint position
  std::vector<double> j_pos = driver.get_current_joint_pos();

  for(auto j:j_pos)
    Logger(LogLevel::INFO) << "jpos [rad]: " << j;
  
  std::vector<double> j_deg(6);
  for (int i=0;i<j_pos.size();i++)
    j_deg.at(i) = j_pos.at(i) / 3.1415/180.0;
  j_deg.at(2) -= ( j_deg[1]);

  for(auto j:j_deg)
    Logger(LogLevel::INFO) << "jpos [deg]: " << j;

  // writes on a register
  driver.write_register(2,1);

  // writes on a position register
  // driver.write_pos_register(j_pos);


  driver.activateDPM();
  std_msgs::msg::Float32 msg_c,msg_f;
  rclcpp::Rate dpm_rate(250);
  {
    while (rclcpp::ok())
    {
      std::vector<double> w = {node->meas_wrench_[0]-wrench_0[0], 0, 0, 0, 0, 0};
      std::vector<int> cmd = {w[0], 0, 0, 0, 0, 0};
      RCLCPP_INFO_STREAM_THROTTLE(node->get_logger(),clock,500,"v: " << w[0] << ", cmd: " << cmd[0]);
      driver.writeDPM(cmd);


      j_pos = driver.get_current_joint_pos();

      msg_c.data=cmd[0];
      msg_f.data=j_pos[0]*100;

      node->cmd_publisher_->publish(msg_c);
      node->fb_publisher_->publish(msg_f);

      dpm_rate.sleep();
    }
  }

  driver.deactivateDPM();

  rclcpp::shutdown();

  return 0;
}
