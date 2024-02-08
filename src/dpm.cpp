/*
Example of usage of the DPM control mode.
This example shows how to listen to a FT sensor and generate motion accordingly
*/


#include <ros2_fanuc_interface/fanuc_eth_ip.h>

#include <unistd.h>
#include "rclcpp/rclcpp.hpp"
#include <std_msgs/msg/int64_multi_array.hpp>
#include <std_srvs/srv/set_bool.hpp> 
#include <sensor_msgs/msg/joint_state.hpp> 


class DPMSubscriber : public rclcpp::Node
{
  public:
    
    DPMSubscriber();
    ~ DPMSubscriber();
    std::shared_ptr<fanuc_eth_ip> EIP_driver_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr fb_publisher_;
    double rate_;

  private:
    void Callback(const std_msgs::msg::Int64MultiArray::SharedPtr msg);
    void ActivateDPM(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                    const std::shared_ptr<std_srvs::srv::SetBool::Response> response);
    rclcpp::Subscription<std_msgs::msg::Int64MultiArray>::SharedPtr subscription_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr activate_dpm_srv_;
};

DPMSubscriber::DPMSubscriber(): Node("dpm_subscriber")
{
  this->declare_parameter("robot_ip","10.11.31.111");
  std::string robot_ip = this->get_parameter("robot_ip").as_string();
  this->declare_parameter("deformation_topic","/dpm_move");
  std::string deformation_topic = this->get_parameter("deformation_topic").as_string();
  this->declare_parameter("rate",10.0);
  rate_ = this->get_parameter("rate").as_double();
  RCLCPP_INFO_STREAM(this->get_logger(), "Feedback publish rate: " << rate_ );

  RCLCPP_INFO_STREAM(this->get_logger(), "subscribed to: " << deformation_topic );

  subscription_ = this->create_subscription<std_msgs::msg::Int64MultiArray>( deformation_topic,
                                                                        10,
                                                                        std::bind(&DPMSubscriber::Callback,
                                                                        this, 
                                                                        std::placeholders::_1));

  activate_dpm_srv_ = create_service<std_srvs::srv::SetBool>(
      "activate_dpm", std::bind(&DPMSubscriber::ActivateDPM, this,
                              std::placeholders::_1, std::placeholders::_2));

  fb_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("dpm_fb", 10);

  EIP_driver_.reset( new fanuc_eth_ip (robot_ip) );
  RCLCPP_INFO_STREAM(this->get_logger(),"Initialized EIP driver at ip: " << robot_ip );

}

DPMSubscriber::~ DPMSubscriber()
{
  EIP_driver_->deactivateDPM();
}

void DPMSubscriber::ActivateDPM(
      const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
      const std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
  if (request->data==true)
  {
    EIP_driver_->activateDPM();
    RCLCPP_INFO_STREAM(this->get_logger()," DPM activated! ");
  }
  else
  {
    EIP_driver_->deactivateDPM();
    RCLCPP_INFO_STREAM(this->get_logger()," DPM de activated! ");
  }

  response->success = true;
}


void DPMSubscriber::Callback(const std_msgs::msg::Int64MultiArray::SharedPtr msg) 
{
  RCLCPP_DEBUG_STREAM(this->get_logger(), "\n" << std_msgs::msg::to_yaml(*msg) );

  std::vector<int> dpm_deformation(6);
  dpm_deformation.at(0) = msg->data[0];
  dpm_deformation.at(1) = msg->data[1];
  dpm_deformation.at(2) = msg->data[2];
  dpm_deformation.at(3) = msg->data[3];
  dpm_deformation.at(4) = msg->data[4];
  dpm_deformation.at(5) = msg->data[5];

  EIP_driver_->writeDPM(dpm_deformation);

}



int main(int argc, char * argv[]) 
{
  
  rclcpp::init(argc, argv);
  std::shared_ptr< DPMSubscriber > node = std::make_shared<DPMSubscriber>();

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread([&executor]() { executor.spin(); }).detach();

  std::vector<std::string> j_names = {"j1","j2","j3","j4","j5","j6"};



  rclcpp::Rate rate(node->rate_);
  while (rclcpp::ok())
  {
    std::vector<double> jp = node->EIP_driver_->get_current_joint_pos();
    sensor_msgs::msg::JointState js_msg;

    for(int i=0;i<j_names.size();i++)
    {
      js_msg.name.push_back(j_names.at(i));
      js_msg.position.push_back(jp.at(i));
    }
    js_msg.header.stamp = node->get_clock()->now();

    node->fb_publisher_->publish(js_msg);

    rate.sleep();    
  }
  node->EIP_driver_->deactivateDPM();

  rclcpp::shutdown();

  return 0;
}
