/*
Example of usage of the DPM control mode.
This example shows how to listen to a FT sensor and generate motion accordingly
*/


#include <fanuc_eth_ip/fanuc_eth_ip.hpp>

#include <unistd.h>
#include "rclcpp/rclcpp.hpp"
#include <std_msgs/msg/int64_multi_array.hpp>
#include <std_srvs/srv/set_bool.hpp> 
#include <sensor_msgs/msg/joint_state.hpp> 
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <mutex> 

class DPMSubscriber : public rclcpp::Node
{
  public:
    
    DPMSubscriber();
    ~ DPMSubscriber();
    std::shared_ptr<fanuc_eth_ip> EIP_driver_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr fb_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr cart_fb_publisher_;
    double rate_;
    std::mutex mtx_;
    // geometry_msgs::Quaternion quaternionFromRPY(double roll, double pitch, double yaw)

    std::vector<double> GetCurrentJPos();
    std::vector<double> GetCurrentPose();
  
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
  cart_fb_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("cart_dpm_fb", 10);

  EIP_driver_.reset( new fanuc_eth_ip (robot_ip) );
  RCLCPP_INFO_STREAM(this->get_logger(),"Initialized EIP driver at ip: " << robot_ip );

}

DPMSubscriber::~ DPMSubscriber()
{
  mtx_.lock();
  EIP_driver_->deactivateDPM();
  mtx_.unlock();
}

void DPMSubscriber::ActivateDPM(
      const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
      const std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
  if (request->data==true)
  {
    mtx_.lock();
    EIP_driver_->activateDPM();
    mtx_.unlock();
    RCLCPP_INFO_STREAM(this->get_logger()," DPM activated! ");
  }
  else
  {
    mtx_.lock();
    EIP_driver_->deactivateDPM();
    mtx_.unlock();
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

  mtx_.lock();
  EIP_driver_->writeDPM(dpm_deformation);
  mtx_.unlock();

}

std::vector<double> DPMSubscriber::GetCurrentJPos()
{
  mtx_.lock();
  std::vector<double> ret = EIP_driver_->get_current_joint_pos();
  mtx_.unlock();
  return ret;
}

std::vector<double> DPMSubscriber::GetCurrentPose()
{
  mtx_.lock();
  std::vector<double> ret = EIP_driver_->get_current_pose();
  mtx_.unlock();

  for (int i=0;i<3;i++)
    ret.at(i) /= 1000;

  for (int i=3;i<6;i++)
    ret.at(i) *= 0.0174533;

  return ret;
}

// TODO: implement conversion and cartesian feedback via geometry_msgs::Pose
// geometry_msgs::Quaternion DPMSubscriber::quaternionFromRPY(double roll, double pitch, double yaw)
// {
//   tf2::Quaternion quaternion_tf2;
//   quaternion_tf2.setRPY(roll, pitch, yaw);
//   geometry_msgs::Quaternion quaternion = tf2::toMsg(quaternion_tf2);
//   return quaternion;
// }

int main(int argc, char * argv[]) 
{
  
  rclcpp::init(argc, argv);
  std::shared_ptr< DPMSubscriber > node = std::make_shared<DPMSubscriber>();

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread([&executor]() { executor.spin(); }).detach();

  std::vector<std::string> j_names = {"j1","j2","j3","j4","j5","j6"};
  std::vector<std::string> c_names = {"1x","2y","3z","4r","5p","6y"};

  node->EIP_driver_->read_register(RegisterEnum::DpmStatus);

  rclcpp::Rate rate(node->rate_);
  while (rclcpp::ok())
  {
    std::vector<double> jp = node->GetCurrentJPos();
    std::vector<double> cp = node->GetCurrentPose();

    sensor_msgs::msg::JointState c_msg;
    for(int i=0;i<c_names.size();i++)
    {  
      c_msg.name.push_back(c_names.at(i));
      c_msg.position.push_back(cp.at(i));
    }


    sensor_msgs::msg::JointState js_msg;

    for(int i=0;i<j_names.size();i++)
    {
      js_msg.name.push_back(j_names.at(i));
      js_msg.position.push_back(jp.at(i));
    }

    js_msg.header.stamp = node->get_clock()->now();
    c_msg.header.stamp = node->get_clock()->now();

    node->fb_publisher_->publish(js_msg);
    node->cart_fb_publisher_->publish(c_msg);

    rate.sleep();    
  }
  node->EIP_driver_->deactivateDPM();

  rclcpp::shutdown();

  return 0;
}
