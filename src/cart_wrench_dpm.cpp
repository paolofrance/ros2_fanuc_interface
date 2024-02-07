/*
Example of usage of the DPM control mode.
This example shows how to listen to a FT sensor and generate motion accordingly
*/


#include <ros2_fanuc_interface/fanuc_eth_ip.h>

#include <unistd.h>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include <kdl/frames.hpp>
#include "std_msgs/msg/float32.hpp"
#include <sensor_msgs/msg/joint_state.hpp>

#include <urdf/model.h>
#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>



class WrenchSubscriber : public rclcpp::Node
{
  public:
    
    WrenchSubscriber();
    void topic_callback(const geometry_msgs::msg::WrenchStamped::SharedPtr msg);
    bool updateSensorTransform(const std::vector<double>& j);

    std::vector<double> get_wrench(const bool base = true);

    bool wrench_init_;
    std::vector<double> meas_wrench_={0,0,0,0,0,0};
    std::vector<double> wrench_0_={0,0,0,0,0,0};

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr cmd_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr fb_publisher_;

    rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr wrench_b_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr wrench_s_publisher_;
  
    KDL::Frame ft_sensor_transform_;
    KDL::Chain robot_chain_;
    std::shared_ptr<KDL::ChainFkSolverPos_recursive> fksolver_;
    std::string robot_base_link_;
    std::string end_effector_link_;
    std::vector<std::string> joint_names_;

  private:
    rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr subscription_;
};

WrenchSubscriber::WrenchSubscriber(): Node("cart_wrench_dpm")
{

  // this->declare_parameter("joint_names", std::vector<std::string>(6));
  this->declare_parameter("wrench_topic","/wrench");
  this->declare_parameter("robot_description","");
  this->declare_parameter("robot_base_link","");
  this->declare_parameter("end_effector_link","");

  std::string wrench_topic = this->get_parameter("wrench_topic").as_string();

  RCLCPP_INFO_STREAM(this->get_logger(), "subscribed to: " << wrench_topic );

  wrench_init_ = false;

  subscription_ = this->create_subscription<geometry_msgs::msg::WrenchStamped>(
  wrench_topic, 10, std::bind(&WrenchSubscriber::topic_callback, this, std::placeholders::_1));
  cmd_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("dpm_cmd", 10);
  fb_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("dpm_fb", 10);

  wrench_b_publisher_ = this->create_publisher<geometry_msgs::msg::WrenchStamped>("wrench_base", 10);
  wrench_s_publisher_ = this->create_publisher<geometry_msgs::msg::WrenchStamped>("wrench_sensor", 10);


  urdf::Model robot_model;
  KDL::Tree robot_tree;
  std::string robot_description;

  for (int i=0;i<6;i++)
    joint_names_.push_back ("j" + std::to_string(i) );

  robot_description = this->get_parameter("robot_description").as_string();
  if (robot_description.empty())
  {
    RCLCPP_ERROR(this->get_logger(), "robot_description is empty");
    return;
  }
  robot_base_link_ = this->get_parameter("robot_base_link").as_string();
  if (robot_base_link_.empty())
  {
    RCLCPP_ERROR(this->get_logger(), "robot_base_link is empty");
    return;
  }
  else
    RCLCPP_INFO_STREAM(this->get_logger(), "robot_base_link: " << robot_base_link_);

  end_effector_link_ = this->get_parameter("end_effector_link").as_string();
  if (end_effector_link_.empty())
  {
    RCLCPP_ERROR(this->get_logger(), "end_effector_link is empty");
    return;
  }
  else
    RCLCPP_INFO_STREAM(this->get_logger(), "end_effector_link: " << end_effector_link_);

  if (!robot_model.initString(robot_description))
  {
    RCLCPP_ERROR(this->get_logger(), "Failed to parse urdf model from 'robot_description'");
  }
  if (!kdl_parser::treeFromUrdfModel(robot_model, robot_tree))
  {
    RCLCPP_ERROR(this->get_logger(), "Failed to parse KDL tree from urdf model");
  }
  if (!robot_tree.getChain(robot_base_link_, end_effector_link_, robot_chain_))
  {
    const std::string error =
      ""
      "Failed to parse robot chain from urdf model. "
      "Do robot_base_link and end_effector_link exist?";
    RCLCPP_ERROR(this->get_logger(), error.c_str());
  }
  fksolver_.reset(new KDL::ChainFkSolverPos_recursive(robot_chain_));
}


std::vector<double> WrenchSubscriber::get_wrench(const bool base)
{
  if(base)
    return meas_wrench_;
  else
    RCLCPP_ERROR(this->get_logger(), "NOT YET IMPLEMENTED FOR OTHER FRAMES");

}

void WrenchSubscriber::topic_callback(const geometry_msgs::msg::WrenchStamped::SharedPtr msg) 
{
  if (!wrench_init_)
  {
    wrench_init_ = true;
    wrench_0_.at(0) = msg->wrench.force.x;
    wrench_0_.at(1) = msg->wrench.force.y;
    wrench_0_.at(2) = msg->wrench.force.z;
    wrench_0_.at(3) = msg->wrench.torque.x;
    wrench_0_.at(4) = msg->wrench.torque.y;
    wrench_0_.at(5) = msg->wrench.torque.z;

  }

  RCLCPP_DEBUG_STREAM(this->get_logger(), "\n" << geometry_msgs::msg::to_yaml(msg->wrench) );

  geometry_msgs::msg::WrenchStamped wb_msg;
  geometry_msgs::msg::WrenchStamped ws_msg;


  KDL::Wrench tmp;
  tmp[0] = msg->wrench.force.x  - wrench_0_.at(0);
  tmp[1] = msg->wrench.force.y  - wrench_0_.at(1);
  tmp[2] = msg->wrench.force.z  - wrench_0_.at(2);
  tmp[3] = msg->wrench.torque.x - wrench_0_.at(3);
  tmp[4] = msg->wrench.torque.y - wrench_0_.at(4);
  tmp[5] = msg->wrench.torque.z - wrench_0_.at(5);

  // Compute how the measured wrench appears in the frame of interest.
  tmp = ft_sensor_transform_ * tmp;

  meas_wrench_.at(0) = tmp[0];
  meas_wrench_.at(1) = tmp[1];
  meas_wrench_.at(2) = tmp[2];
  meas_wrench_.at(3) = tmp[3];
  meas_wrench_.at(4) = tmp[4];
  meas_wrench_.at(5) = tmp[5];

  wb_msg.header.stamp = this->get_clock()->now();
  ws_msg.header.stamp = this->get_clock()->now();

  wb_msg.header.frame_id = robot_base_link_;
  ws_msg.header.frame_id = end_effector_link_;

  wb_msg.wrench.force.x  = meas_wrench_.at(0);
  wb_msg.wrench.force.y  = meas_wrench_.at(1);
  wb_msg.wrench.force.z  = meas_wrench_.at(2);
  wb_msg.wrench.torque.x = meas_wrench_.at(3);
  wb_msg.wrench.torque.y = meas_wrench_.at(4);
  wb_msg.wrench.torque.z = meas_wrench_.at(5);

  // ws_msg = msg;

  wrench_b_publisher_->publish(wb_msg);
  // wrench_s_publisher_->publish(ws_msg);

}

bool WrenchSubscriber::updateSensorTransform(const std::vector<double>& j)
{
  KDL::JntArray jointpositions = KDL::JntArray(j.size());

  for(unsigned int i=0;i<j.size();i++)
    jointpositions(i)=j[i];

  fksolver_->JntToCart(jointpositions, ft_sensor_transform_);

  geometry_msgs::msg::Pose pose;
  pose.position.x = ft_sensor_transform_.p[0];
  pose.position.y = ft_sensor_transform_.p[1];
  pose.position.z = ft_sensor_transform_.p[2];
  ft_sensor_transform_.M.GetQuaternion(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
  RCLCPP_DEBUG_STREAM(this->get_logger(),"pose: "<< geometry_msgs::msg::to_yaml(pose) );

  return true;
}





int main(int argc, char * argv[]) 
{
  
  rclcpp::init(argc, argv);
  std::shared_ptr< WrenchSubscriber > node = std::make_shared<WrenchSubscriber>();

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread([&executor]() { executor.spin(); }).detach();

  fanuc_eth_ip driver("10.11.31.111");
  std::vector<double> j_pos = driver.get_current_joint_pos();

  for(auto j:j_pos)
    RCLCPP_ERROR_STREAM(node->get_logger(),"j_pos: "<<j);
    
  if(!node->updateSensorTransform(j_pos))
  {
    RCLCPP_ERROR_STREAM(node->get_logger(),"error in update j frame: ");
    return -1;
  }

  rclcpp::Clock clock;
  rclcpp::Rate rate(1);
  while (!node->wrench_init_)
  {
    RCLCPP_INFO_THROTTLE(node->get_logger(),clock,1000,"waiting for first wrench message");
    rate.sleep();
  }

  driver.write_register(2,1);

  driver.activateDPM();


  sensor_msgs::msg::JointState msg_c,msg_f;
  rclcpp::Rate dpm_rate(10);
  {
    while (rclcpp::ok())
    {

      node->updateSensorTransform( driver.get_current_joint_pos() );
      
      std::vector<double> w = node->get_wrench();
      // std::vector<int> cmd (w.begin(), w.end() );

      std::vector<int> cmd = {0,0,0,0,0,0};
      cmd[0]=w[0];
      cmd[1]=w[1];

      // RCLCPP_INFO_STREAM_THROTTLE(node->get_logger(),clock,500,"v: " << w[0] << ", cmd: " << cmd[0]);
      // RCLCPP_INFO_STREAM(node->get_logger(),"cmd: " << cmd[0] << ", "<< cmd[1] << ", "<< cmd[2] << ", "<< cmd[3] << ", "<< cmd[4] << ", "<< cmd[5]);
      driver.writeDPM(cmd);


      j_pos = driver.get_current_joint_pos();

      for (int i=0; i<cmd.size();i++)
      {
        msg_c.name.push_back(node->joint_names_.at(i));
        msg_f.name.push_back(node->joint_names_.at(i));

        msg_c.position.push_back( cmd.at(i) );
        RCLCPP_INFO_STREAM(node->get_logger(),"msg_c: " << msg_c.position[i] );
        msg_f.position.push_back( j_pos.at(i)*100 );
      } 

      node->cmd_publisher_->publish(msg_c);
      node->fb_publisher_->publish(msg_f);



      dpm_rate.sleep();
    }
  }

  driver.deactivateDPM();

  rclcpp::shutdown();

  return 0;
}
