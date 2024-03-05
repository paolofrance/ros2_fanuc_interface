/*
services implementation to control the gripper via TP programs (TP_programs/gripper)
R[10]: gripper activate
R[11]: gripper position
R[12]: gripper speed
R[13]: gripper force

R[21] cheange tool

R[99] de-activate collaborative
*/


#include <fanuc_eth_ip/fanuc_eth_ip.hpp>

#include <unistd.h>
#include "rclcpp/rclcpp.hpp"

#include <fanuc_srvs_msgs/srv/gripper_move.hpp>
#include <fanuc_srvs_msgs/srv/tool_change.hpp>
#include <std_srvs/srv/set_bool.hpp>

#include <mutex> 

class FanucSrvs : public rclcpp::Node
{
  public:
    
    FanucSrvs();
    ~ FanucSrvs();
    std::shared_ptr<fanuc_eth_ip> EIP_driver_;
    std::mutex mtx_;
  private:
    bool gripper_activated_ = false;
    void ActivateGripper(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                    const std::shared_ptr<std_srvs::srv::SetBool::Response> response);
    void MoveGripper(const std::shared_ptr<fanuc_srvs_msgs::srv::GripperMove::Request> request,
                    const std::shared_ptr<fanuc_srvs_msgs::srv::GripperMove::Response> response);
    void ToolChange(const std::shared_ptr<fanuc_srvs_msgs::srv::ToolChange::Request> request,
                    const std::shared_ptr<fanuc_srvs_msgs::srv::ToolChange::Response> response);
    void DeActivateCollab(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                          const std::shared_ptr<std_srvs::srv::SetBool::Response> response);
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr activate_gripper_srv_;
    rclcpp::Service<fanuc_srvs_msgs::srv::GripperMove>::SharedPtr move_gripper_srv_;
    rclcpp::Service<fanuc_srvs_msgs::srv::ToolChange>::SharedPtr tool_change_srv_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr deactivate_collab_srv_;

};

FanucSrvs::FanucSrvs(): Node("fanuc_srvs")
{
  this->declare_parameter("robot_ip","10.11.31.111");
  std::string robot_ip = this->get_parameter("robot_ip").as_string();

  activate_gripper_srv_ = create_service<std_srvs::srv::SetBool>(
      "activate_gripper", std::bind(&FanucSrvs::ActivateGripper, this,
                              std::placeholders::_1, std::placeholders::_2));
  move_gripper_srv_ = create_service<fanuc_srvs_msgs::srv::GripperMove>(
      "move_gripper", std::bind(&FanucSrvs::MoveGripper, this,
                              std::placeholders::_1, std::placeholders::_2));
  tool_change_srv_ = create_service<fanuc_srvs_msgs::srv::ToolChange>(
      "tool_change", std::bind(&FanucSrvs::ToolChange, this,
                              std::placeholders::_1, std::placeholders::_2));
  deactivate_collab_srv_ = create_service<std_srvs::srv::SetBool>(
      "deactivate_collab", std::bind(&FanucSrvs::DeActivateCollab, this,
                              std::placeholders::_1, std::placeholders::_2));

  EIP_driver_.reset( new fanuc_eth_ip (robot_ip) );
  RCLCPP_INFO_STREAM(this->get_logger(),"Initialized EIP driver at ip: " << robot_ip );
}

FanucSrvs::~ FanucSrvs()
{
  RCLCPP_INFO_STREAM(this->get_logger(),"closing node" );
}


void FanucSrvs::ActivateGripper(
      const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
      const std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
  mtx_.lock();
  EIP_driver_->write_register(1,15);
  mtx_.unlock();
  RCLCPP_INFO_STREAM(this->get_logger()," gripper activated! ");
  gripper_activated_=true;
  response->success = true;
}

void FanucSrvs::MoveGripper(
      const std::shared_ptr<fanuc_srvs_msgs::srv::GripperMove::Request> request,
      const std::shared_ptr<fanuc_srvs_msgs::srv::GripperMove::Response> response)
{
  if(gripper_activated_)
  {
    mtx_.lock();
    this->EIP_driver_->write_register(request->position,11);
    this->EIP_driver_->write_register(request->velocity,12);
    this->EIP_driver_->write_register(request->effort,13);
    this->EIP_driver_->write_register(1,10);
    mtx_.unlock();
    RCLCPP_INFO_STREAM(this->get_logger()," moveing gripper ");
    response->success = true;
  }
  else
  {
    RCLCPP_ERROR_STREAM(this->get_logger()," gripper not activated! ");
    response->success = false;
  }

}

void FanucSrvs::ToolChange(
      const std::shared_ptr<fanuc_srvs_msgs::srv::ToolChange::Request> request,
      const std::shared_ptr<fanuc_srvs_msgs::srv::ToolChange::Response> response)
{
  mtx_.lock();
  this->EIP_driver_->write_register(request->position_id,21);
  this->EIP_driver_->write_register(request->get,22);
  mtx_.unlock();
  RCLCPP_INFO_STREAM(this->get_logger()," cahnging tool ");
  response->success = true;
  gripper_activated_=false;

}


void FanucSrvs::DeActivateCollab(
      const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
      const std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
  mtx_.lock();
  EIP_driver_->write_register(request->data ,99);
  mtx_.unlock();
  RCLCPP_INFO_STREAM(this->get_logger()," collaborative mode "<< !request->data <<"! ");
  gripper_activated_=true;
  response->success = true;
}


int main(int argc, char * argv[]) 
{
  
  rclcpp::init(argc, argv);
  std::shared_ptr< FanucSrvs > node = std::make_shared<FanucSrvs>();

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread([&executor]() { executor.spin(); }).detach();

  while (rclcpp::ok())
  {
    RCLCPP_INFO_STREAM_THROTTLE(node->get_logger(),*node->get_clock(),10000," looping srv! ");
  }

  rclcpp::shutdown();

  return 0;
}
