#include <fanuc_rmi/rmi_communication.h>
#include <rclcpp/rclcpp.hpp>

namespace rmi {

// ---------------------------
// CONSTRUCTOR AND DESCTRUCTOR
// ---------------------------

RMICommunication::RMICommunication() {}
RMICommunication::~RMICommunication() {}

// ---------------------
// COMMUNICATION METHODS
// ---------------------

const std::string RMICommunication::cmc_Connect()
{ return "{\"Communication\":\"FRC_Connect\"}\r\n"; }

const std::string RMICommunication::cmc_Disconnect()
{ return "{\"Communication\":\"FRC_Disconnect\"}\r\n"; }

// ---------------
// COMMAND METHODS
// ---------------

const std::string RMICommunication::cmd_Initialize()
{ return "{\"Command\":\"FRC_Initialize\"}\r\n"; }

const std::string RMICommunication::cmd_Abort()
{ return "{\"Command\":\"FRC_Abort\"}\r\n"; }

const std::string RMICommunication::cmd_Pause()
{ return "{\"Command\":\"FRC_Pause\"}\r\n"; }

const std::string RMICommunication::cmd_Continue()
{ return "{\"Command\":\"FRC_Continue\"}\r\n"; }

const std::string RMICommunication::cmd_ReadError()
{ return "{\"Command\":\"FRC_ReadError\"}\r\n"; }

const std::string RMICommunication::cmd_SetUFrameUTool(int frame_number_, int tool_number_)
{ 
  std::string msg = "{\"Command\":\"FRC_SetUFrameUTool\",\"UFrameNumber\":";
  msg.append(std::to_string(frame_number_));
  msg.append(",\"UToolNumber\":");
  msg.append(std::to_string(tool_number_));
  msg.append("}\r\n");
  return msg;
}

const std::string RMICommunication::cmd_GetStatus()
{ return "{\"Command\":\"FRC_GetStatus\"}\r\n"; }

const std::string RMICommunication::cmd_ReadUFrameData(int frame_number_)
{ 
  std::string msg = "{\"Command\":\"FRC_ReadUFrameData\",\"FrameNumber\":";
  msg.append(std::to_string(frame_number_));
  msg.append("}\r\n");
  return msg;
}

const std::string RMICommunication::cmd_WriteUFrameData(int frame_number_, Pose pose_)
{ 
  std::string msg = "{\"Command\":\"FRC_WriteUFrameData\",\"FrameNumber\":";
  msg.append(std::to_string(frame_number_));
  msg.append(",\"Frame\":{\"X\":");
  msg.append(std::to_string(pose_.x_));
  msg.append(",\"Y\":");
  msg.append(std::to_string(pose_.y_));
  msg.append(",\"Z\":");
  msg.append(std::to_string(pose_.z_));
  msg.append(",\"W\":");
  msg.append(std::to_string(pose_.w_));
  msg.append(",\"P\":");
  msg.append(std::to_string(pose_.p_));
  msg.append(",\"R\":");
  msg.append(std::to_string(pose_.r_));
  msg.append("}}\r\n");
  return msg;
}

const std::string RMICommunication::cmd_ReadUToolData(int tool_number_)
{ 
  std::string msg = "{\"Command\":\"FRC_ReadUToolData\",\"ToolNumber\":";
  msg.append(std::to_string(tool_number_));
  msg.append("}\r\n");
  return msg;
}

const std::string RMICommunication::cmd_WriteUToolData(int tool_number_, Pose pose_)
{ 
  std::string msg = "{\"Command\":\"FRC_WriteUToolData\",\"ToolNumber\":";
  msg.append(std::to_string(tool_number_));
  msg.append(",\"Frame\":{\"X\":");
  msg.append(std::to_string(pose_.x_));
  msg.append(",\"Y\":");
  msg.append(std::to_string(pose_.y_));
  msg.append(",\"Z\":");
  msg.append(std::to_string(pose_.z_));
  msg.append(",\"W\":");
  msg.append(std::to_string(pose_.w_));
  msg.append(",\"P\":");
  msg.append(std::to_string(pose_.p_));
  msg.append(",\"R\":");
  msg.append(std::to_string(pose_.r_));
  msg.append("}}\r\n");
  return msg;
}

const std::string RMICommunication::cmd_ReadDIN(int input_port_number_)
{ 
  std::string msg = "{\"Command\":\"FRC_ReadDIN\",\"PortNumber\":";
  msg.append(std::to_string(input_port_number_));
  msg.append("}\r\n");
  return msg;
}

const std::string RMICommunication::cmd_WriteDOUT(int output_port_number_, PortValue port_value_)
{ 
  std::string msg = "{\"Command\":\"FRC_WriteDOUT\",\"PortNumber\":";
  msg.append(std::to_string(output_port_number_));
  if (port_value_ == PortValue::ON) {
    msg.append(",\"PortValue\":\"ON\"}\r\n");
  }
  else if (port_value_ == PortValue::OFF) {
    msg.append(",\"PortValue\":\"OFF\"}\r\n");
  }
  else {
    RCLCPP_ERROR( rclcpp::get_logger("RMI"), "Invalid PortValue requested. Supported values are ON or OFF.");
  }
  return msg;
}

const std::string RMICommunication::cmd_ReadCartesianPosition()
{ return "{\"Command\":\"FRC_ReadCartesianPosition\"}\r\n"; }

const std::string RMICommunication::cmd_ReadJointAngles()
{ return "{\"Command\":\"FRC_ReadJointAngles\"}\r\n"; }

const std::string RMICommunication::cmd_SetOverRide(int override_)
{ 
  if (override_ > 100) {
    override_ = 100;
    RCLCPP_WARN( rclcpp::get_logger("RMI"), "Requested override out of bounds, defaulting to %d",override_);
  }
  else if (override_ < 0) {
    override_ = 0;
    RCLCPP_WARN( rclcpp::get_logger("RMI"), "Requested override out of bounds, defaulting to %d",override_);
  }
  std::string msg = "{\"Command\":\"FRC_SetOverRide\",\"Value\":";
  msg.append(std::to_string(override_));
  msg.append("}\r\n");
  return msg;
}

const std::string RMICommunication::cmd_GetUFrameUTool()
{ return "{\"Command\":\"FRC_GetUFrameUTool\"}\r\n"; }

const std::string RMICommunication::cmd_ReadPositionRegister(int register_number_)
{ 
  std::string msg = "{\"Command\":\"FRC_ReadPositionRegister\",\"RegisterNumber\":";
  msg.append(std::to_string(register_number_));
  msg.append("}\r\n");
  return msg;
}

const std::string RMICommunication::cmd_WritePositionRegister(int register_number_, Configuration configuration_, Pose pose_)
{ 
  std::string msg = "{\"Command\":\"FRC_WritePositionRegister\",\"RegisterNumber\":";
  msg.append(std::to_string(register_number_));
  msg.append(",\"Configuration\":{\"UToolNumber\":");
  msg.append(std::to_string(configuration_.u_tool_number_));
  msg.append(",\"UFrameNumber\":");
  msg.append(std::to_string(configuration_.u_frame_number_));
  msg.append(",\"Front\":");
  msg.append(std::to_string(configuration_.front_));
  msg.append(",\"Up\":");
  msg.append(std::to_string(configuration_.up_));
  msg.append(",\"Left\":");
  msg.append(std::to_string(configuration_.left_));
  msg.append(",\"Flip\":");
  msg.append(std::to_string(configuration_.flip_));
  msg.append(",\"Turn4\":");
  msg.append(std::to_string(configuration_.turn_4_));
  msg.append(",\"Turn5\":");
  msg.append(std::to_string(configuration_.turn_5_));
  msg.append(",\"Turn6\":");
  msg.append(std::to_string(configuration_.turn_6_));
  msg.append("},\"Position\":{\"X\"");
  msg.append(std::to_string(pose_.x_));
  msg.append(",\"Y\"");
  msg.append(std::to_string(pose_.y_));
  msg.append(",\"Z\"");
  msg.append(std::to_string(pose_.z_));
  msg.append(",\"W\"");
  msg.append(std::to_string(pose_.w_));
  msg.append(",\"R\"");
  msg.append(std::to_string(pose_.p_));
  msg.append(",\"R\"");
  msg.append(std::to_string(pose_.r_));
  msg.append(",\"Ext1\"");
  msg.append(std::to_string(pose_.ext_1_));
  msg.append(",\"Ext2\"");
  msg.append(std::to_string(pose_.ext_2_));
  msg.append(",\"Ext3\"");
  msg.append(std::to_string(pose_.ext_3_));
  msg.append("}}\r\n");
  return msg;
}

const std::string RMICommunication::cmd_Reset()
{ return "{\"Command\":\"FRC_Reset\"}\r\n"; }

const std::string RMICommunication::cmd_ReadTCPSpeed()
{ return "{\"Command\":\"FRC_ReadTCPSpeed\"}\r\n"; }

// -------------------
// INSTRUCTION METHODS
// -------------------

const std::string RMICommunication::ins_WaitDin(int sequence_id_, int input_port_number_, PortValue port_value_)
{ 
  std::string msg = "{\"Instruction\":\"FRC_WaitDin\",\"SequenceID\":";
  msg.append(std::to_string(sequence_id_));
  msg.append(",\"PortNumber\":");
  msg.append(std::to_string(input_port_number_));
  if (port_value_ == PortValue::ON) {
    msg.append(",\"PortValue\":\"ON\"}\r\n");
  }
  else if (port_value_ == PortValue::OFF) {
    msg.append(",\"PortValue\":\"OFF\"}\r\n");
  }
  else {
    RCLCPP_ERROR( rclcpp::get_logger("RMI"), "Invalid PortValue requested. Supported values are ON or OFF.");
  }
  return msg;
}

const std::string RMICommunication::ins_SetUFrame(int sequence_id_, int frame_number_)
{ 
  std::string msg = "{\"Instruction\":\"FRC_SetUFrame\",\"SequenceID\":";
  msg.append(std::to_string(sequence_id_));
  msg.append(",\"FrameNumber\":");
  msg.append(std::to_string(frame_number_));
  msg.append("}\r\n");
  return msg;
}

const std::string RMICommunication::ins_SetUTool(int sequence_id_, int tool_number_)
{ 
  std::string msg = "{\"Instruction\":\"FRC_SetUTool\",\"SequenceID\":";
  msg.append(std::to_string(sequence_id_));
  msg.append(",\"ToolNumber\":");
  msg.append(std::to_string(tool_number_));
  msg.append("}\r\n");
  return msg;
}

const std::string RMICommunication::ins_WaitTime(int sequence_id_, float time_)
{ 
  std::string msg = "{\"Instruction\":\"FRC_WaitTime\",\"SequenceID\":";
  msg.append(std::to_string(sequence_id_));
  msg.append(",\"Time\":");
  msg.append(std::to_string(time_));
  msg.append("}\r\n");
  return msg;
}

const std::string RMICommunication::ins_PayLoad(int sequence_id_, int schedule_number_)
{ 
  std::string msg = "{\"Instruction\":\"FRC_PayLoad\",\"SequenceID\":";
  msg.append(std::to_string(sequence_id_));
  msg.append(",\"ScheduleNumber\":");
  msg.append(std::to_string(schedule_number_));
  msg.append("}\r\n");
  return msg;
}

const std::string RMICommunication::ins_LinearMotion(int sequence_id_, MotionType motion_type_, Configuration configuration_, Pose pose_, SpeedInfo speed_info_, TermInfo term_info_)
{ 
  std::string msg;
  if (motion_type_ == MotionType::ABS) {
    msg.append("{\"Instruction\":\"FRC_LinearMotion\",\"SequenceID\":");
  }
  else if (motion_type_ == MotionType::REL) {
    msg.append("{\"Instruction\":\"FRC_LinearRelative\",\"SequenceID\":");
  }
  else {
    RCLCPP_ERROR( rclcpp::get_logger("RMI"),"Invalid MotionType requested. Supported values are ABS or REL.");
  }
  msg.append(std::to_string(sequence_id_));
  msg.append(",\"Configuration\":{\"UToolNumber\":");
  msg.append(std::to_string(configuration_.u_tool_number_));
  msg.append(",\"UFrameNumber\":");
  msg.append(std::to_string(configuration_.u_frame_number_));
  msg.append(",\"Front\":");
  msg.append(std::to_string(configuration_.front_));
  msg.append(",\"Up\":");
  msg.append(std::to_string(configuration_.up_));
  msg.append(",\"Left\":");
  msg.append(std::to_string(configuration_.left_));
  msg.append(",\"Flip\":");
  msg.append(std::to_string(configuration_.flip_));
  msg.append(",\"Turn4\":");
  msg.append(std::to_string(configuration_.turn_4_));
  msg.append(",\"Turn5\":");
  msg.append(std::to_string(configuration_.turn_5_));
  msg.append(",\"Turn6\":");
  msg.append(std::to_string(configuration_.turn_6_));
  msg.append("},\"Position\":{\"X\"");
  msg.append(std::to_string(pose_.x_));
  msg.append(",\"Y\"");
  msg.append(std::to_string(pose_.y_));
  msg.append(",\"Z\"");
  msg.append(std::to_string(pose_.z_));
  msg.append(",\"W\"");
  msg.append(std::to_string(pose_.w_));
  msg.append(",\"R\"");
  msg.append(std::to_string(pose_.p_));
  msg.append(",\"R\"");
  msg.append(std::to_string(pose_.r_));
  msg.append(",\"Ext1\"");
  msg.append(std::to_string(pose_.ext_1_));
  msg.append(",\"Ext2\"");
  msg.append(std::to_string(pose_.ext_2_));
  msg.append(",\"Ext3\"");
  msg.append(std::to_string(pose_.ext_3_));
  if (speed_info_.speed_type_ == SpeedType::MMSEC) {
    msg.append("},\"SpeedType\":\"mmSec\",\"Speed\":");
  }
  else if (speed_info_.speed_type_ == SpeedType::INCHMIN) {
    msg.append("},\"SpeedType\":\"InchMin\",\"Speed\":");
  }
  else if (speed_info_.speed_type_ == SpeedType::TIME) {
    msg.append("},\"SpeedType\":\"Time\",\"Speed\":");
  }
  else {
    RCLCPP_ERROR( rclcpp::get_logger("RMI"),"Invalid SpeedType requested. Supported values are MMSEC, INCHMIN or TIME.");
  }
  msg.append(std::to_string(speed_info_.speed_value_));
  if (term_info_.term_type_ == TermType::FINE) {
    msg.append(",\"TermType\":\"FINE\",\"TermValue\":");
  }
  else if (term_info_.term_type_ == TermType::CNT) {
    msg.append(",\"TermType\":\"CNT\",\"TermValue\":");
  }
  else if (term_info_.term_type_ == TermType::CR) {
    msg.append(",\"TermType\":\"CR\",\"TermValue\":");
  }
  else {
    RCLCPP_ERROR( rclcpp::get_logger("RMI"),"Invalid TermType requested. Supported values are FINE, CNT or CR.");
  }
  msg.append(std::to_string(term_info_.term_value_));
  //TODO: Add optional parameters
  msg.append("}\r\n");
  return msg;
}

const std::string RMICommunication::ins_JointMotion(int sequence_id_, MotionType motion_type_, Configuration configuration_, Pose pose_, SpeedInfo speed_info_, TermInfo term_info_)
{ 
  std::string msg;
  if (motion_type_ == MotionType::ABS) {
    msg.append("{\"Instruction\":\"FRC_JointMotion\",\"SequenceID\":");
  }
  else if (motion_type_ == MotionType::REL) {
    msg.append("{\"Instruction\":\"FRC_JointRelative\",\"SequenceID\":");
  }
  else {
    RCLCPP_ERROR( rclcpp::get_logger("RMI"),"Invalid MotionType requested. Supported values are ABS or REL.");
  }
  msg.append(std::to_string(sequence_id_));
  msg.append(",\"Configuration\":{\"UToolNumber\":");
  msg.append(std::to_string(configuration_.u_tool_number_));
  msg.append(",\"UFrameNumber\":");
  msg.append(std::to_string(configuration_.u_frame_number_));
  msg.append(",\"Front\":");
  msg.append(std::to_string(configuration_.front_));
  msg.append(",\"Up\":");
  msg.append(std::to_string(configuration_.up_));
  msg.append(",\"Left\":");
  msg.append(std::to_string(configuration_.left_));
  msg.append(",\"Flip\":");
  msg.append(std::to_string(configuration_.flip_));
  msg.append(",\"Turn4\":");
  msg.append(std::to_string(configuration_.turn_4_));
  msg.append(",\"Turn5\":");
  msg.append(std::to_string(configuration_.turn_5_));
  msg.append(",\"Turn6\":");
  msg.append(std::to_string(configuration_.turn_6_));
  msg.append("},\"Position\":{\"X\"");
  msg.append(std::to_string(pose_.x_));
  msg.append(",\"Y\"");
  msg.append(std::to_string(pose_.y_));
  msg.append(",\"Z\"");
  msg.append(std::to_string(pose_.z_));
  msg.append(",\"W\"");
  msg.append(std::to_string(pose_.w_));
  msg.append(",\"R\"");
  msg.append(std::to_string(pose_.p_));
  msg.append(",\"R\"");
  msg.append(std::to_string(pose_.r_));
  msg.append(",\"Ext1\"");
  msg.append(std::to_string(pose_.ext_1_));
  msg.append(",\"Ext2\"");
  msg.append(std::to_string(pose_.ext_2_));
  msg.append(",\"Ext3\"");
  msg.append(std::to_string(pose_.ext_3_));
  if (speed_info_.speed_type_ == SpeedType::PERC) {
    msg.append("},\"SpeedType\":\"Percent\",\"Speed\":");
  }
  else if (speed_info_.speed_type_ == SpeedType::TIME) {
    msg.append("},\"SpeedType\":\"Time\",\"Speed\":");
  }
  else {
    RCLCPP_ERROR( rclcpp::get_logger("RMI"),"Invalid SpeedType requested. Supported values are PERC or TIME.");
  }
  msg.append(std::to_string(speed_info_.speed_value_));
  if (term_info_.term_type_ == TermType::FINE) {
    msg.append(",\"TermType\":\"FINE\",\"TermValue\":");
  }
  else if (term_info_.term_type_ == TermType::CNT) {
    msg.append(",\"TermType\":\"CNT\",\"TermValue\":");
  }
  else if (term_info_.term_type_ == TermType::CR) {
    msg.append(",\"TermType\":\"CR\",\"TermValue\":");
  }
  else {
    RCLCPP_ERROR( rclcpp::get_logger("RMI"),"Invalid TermType requested. Supported values are FINE, CNT or CR.");
  }
  msg.append(std::to_string(term_info_.term_value_));
  //TODO: Add optional parameters
  msg.append("}\r\n");
  return msg;
}

const std::string RMICommunication::ins_CircularMotion(int sequence_id_, MotionType motion_type_, Configuration configuration_, Configuration via_configuration_, Pose pose_, Pose via_pose_, SpeedInfo speed_info_, TermInfo term_info_)
{ 
  std::string msg;
  if (motion_type_ == MotionType::ABS) {
    msg.append("{\"Instruction\":\"FRC_CircularMotion\",\"SequenceID\":");
  }
  else if (motion_type_ == MotionType::REL) {
    msg.append("{\"Instruction\":\"FRC_CircularRelative\",\"SequenceID\":");
  }
  else {
    RCLCPP_ERROR( rclcpp::get_logger("RMI"),"Invalid MotionType requested. Supported values are ABS or REL.");
  }
  msg.append(std::to_string(sequence_id_));
  msg.append(",\"Configuration\":{\"UToolNumber\":");
  msg.append(std::to_string(configuration_.u_tool_number_));
  msg.append(",\"UFrameNumber\":");
  msg.append(std::to_string(configuration_.u_frame_number_));
  msg.append(",\"Front\":");
  msg.append(std::to_string(configuration_.front_));
  msg.append(",\"Up\":");
  msg.append(std::to_string(configuration_.up_));
  msg.append(",\"Left\":");
  msg.append(std::to_string(configuration_.left_));
  msg.append(",\"Flip\":");
  msg.append(std::to_string(configuration_.flip_));
  msg.append(",\"Turn4\":");
  msg.append(std::to_string(configuration_.turn_4_));
  msg.append(",\"Turn5\":");
  msg.append(std::to_string(configuration_.turn_5_));
  msg.append(",\"Turn6\":");
  msg.append(std::to_string(configuration_.turn_6_));
  msg.append("},\"Position\":{\"X\"");
  msg.append(std::to_string(pose_.x_));
  msg.append(",\"Y\"");
  msg.append(std::to_string(pose_.y_));
  msg.append(",\"Z\"");
  msg.append(std::to_string(pose_.z_));
  msg.append(",\"W\"");
  msg.append(std::to_string(pose_.w_));
  msg.append(",\"R\"");
  msg.append(std::to_string(pose_.p_));
  msg.append(",\"R\"");
  msg.append(std::to_string(pose_.r_));
  msg.append(",\"Ext1\"");
  msg.append(std::to_string(pose_.ext_1_));
  msg.append(",\"Ext2\"");
  msg.append(std::to_string(pose_.ext_2_));
  msg.append(",\"Ext3\"");
  msg.append(std::to_string(pose_.ext_3_));
  msg.append("},\"ViaConfiguration\":{\"UToolNumber\":");
  msg.append(std::to_string(via_configuration_.u_tool_number_));
  msg.append(",\"UFrameNumber\":");
  msg.append(std::to_string(via_configuration_.u_frame_number_));
  msg.append(",\"Front\":");
  msg.append(std::to_string(via_configuration_.front_));
  msg.append(",\"Up\":");
  msg.append(std::to_string(via_configuration_.up_));
  msg.append(",\"Left\":");
  msg.append(std::to_string(via_configuration_.left_));
  msg.append(",\"Flip\":");
  msg.append(std::to_string(via_configuration_.flip_));
  msg.append(",\"Turn4\":");
  msg.append(std::to_string(via_configuration_.turn_4_));
  msg.append(",\"Turn5\":");
  msg.append(std::to_string(via_configuration_.turn_5_));
  msg.append(",\"Turn6\":");
  msg.append(std::to_string(via_configuration_.turn_6_));
  msg.append("},\"ViaPosition\":{\"X\"");
  msg.append(std::to_string(via_pose_.x_));
  msg.append(",\"Y\"");
  msg.append(std::to_string(via_pose_.y_));
  msg.append(",\"Z\"");
  msg.append(std::to_string(via_pose_.z_));
  msg.append(",\"W\"");
  msg.append(std::to_string(via_pose_.w_));
  msg.append(",\"R\"");
  msg.append(std::to_string(via_pose_.p_));
  msg.append(",\"R\"");
  msg.append(std::to_string(via_pose_.r_));
  msg.append(",\"Ext1\"");
  msg.append(std::to_string(via_pose_.ext_1_));
  msg.append(",\"Ext2\"");
  msg.append(std::to_string(via_pose_.ext_2_));
  msg.append(",\"Ext3\"");
  msg.append(std::to_string(via_pose_.ext_3_));
  if (speed_info_.speed_type_ == SpeedType::MMSEC) {
    msg.append("},\"SpeedType\":\"mmSec\",\"Speed\":");
  }
  else if (speed_info_.speed_type_ == SpeedType::INCHMIN) {
    msg.append("},\"SpeedType\":\"InchMin\",\"Speed\":");
  }
  else if (speed_info_.speed_type_ == SpeedType::TIME) {
    msg.append("},\"SpeedType\":\"Time\",\"Speed\":");
  }
  else {
    RCLCPP_ERROR( rclcpp::get_logger("RMI"),"Invalid SpeedType requested. Supported values are MMSEC, INCHMIN or TIME.");
  }
  msg.append(std::to_string(speed_info_.speed_value_));
  if (term_info_.term_type_ == TermType::FINE) {
    msg.append(",\"TermType\":\"FINE\",\"TermValue\":");
  }
  else if (term_info_.term_type_ == TermType::CNT) {
    msg.append(",\"TermType\":\"CNT\",\"TermValue\":");
  }
  else if (term_info_.term_type_ == TermType::CR) {
    msg.append(",\"TermType\":\"CR\",\"TermValue\":");
  }
  else {
    RCLCPP_ERROR( rclcpp::get_logger("RMI"),"Invalid TermType requested. Supported values are FINE, CNT or CR.");
  }
  msg.append(std::to_string(term_info_.term_value_));
  //TODO: Add optional parameters
  msg.append("}\r\n");
  return msg;
}

const std::string RMICommunication::ins_JointMotionJRep(int sequence_id_, MotionType motion_type_, JointAngle joint_angle_, SpeedInfo speed_info_, TermInfo term_info_)
{
  std::string msg;
  if (motion_type_ == MotionType::ABS) {
    msg.append("{\"Instruction\":\"FRC_JointMotionJRep\",\"SequenceID\":");
  }
  else if (motion_type_ == MotionType::REL) {
    msg.append("{\"Instruction\":\"FRC_JointRelativeJRep\",\"SequenceID\":");
  }
  else {
    RCLCPP_ERROR( rclcpp::get_logger("RMI"),"Invalid MotionType requested. Supported values are ABS or REL.");
  }
  msg.append(std::to_string(sequence_id_));
  msg.append(",\"JointAngle\":{\"J1\":");
  msg.append(std::to_string(joint_angle_.jnt_1_));
  msg.append(",\"J2\":");
  msg.append(std::to_string(joint_angle_.jnt_2_));
  msg.append(",\"J3\":");
  msg.append(std::to_string(joint_angle_.jnt_3_));
  msg.append(",\"J4\":");
  msg.append(std::to_string(joint_angle_.jnt_4_));
  msg.append(",\"J5\":");
  msg.append(std::to_string(joint_angle_.jnt_5_));
  msg.append(",\"J6\":");
  msg.append(std::to_string(joint_angle_.jnt_6_));
  msg.append(",\"J7\":");
  msg.append(std::to_string(joint_angle_.jnt_7_));
  msg.append(",\"J8\":");
  msg.append(std::to_string(joint_angle_.jnt_8_));
  msg.append(",\"J9\":");
  msg.append(std::to_string(joint_angle_.jnt_9_));
  if (speed_info_.speed_type_ == SpeedType::PERC) {
    msg.append("},\"SpeedType\":\"Percent\",\"Speed\":");
  }
  else if (speed_info_.speed_type_ == SpeedType::TIME) {
    msg.append("},\"SpeedType\":\"Time\",\"Speed\":");
  }
  else {
    RCLCPP_ERROR( rclcpp::get_logger("RMI"),"Invalid SpeedType requested. Supported values are PERC or TIME.");
  }
  msg.append(std::to_string(speed_info_.speed_value_));
  if (term_info_.term_type_ == TermType::FINE) {
    msg.append(",\"TermType\":\"FINE\",\"TermValue\":");
  }
  else if (term_info_.term_type_ == TermType::CNT) {
    msg.append(",\"TermType\":\"CNT\",\"TermValue\":");
  }
  else if (term_info_.term_type_ == TermType::CR) {
    msg.append(",\"TermType\":\"CR\",\"TermValue\":");
  }
  else {
    RCLCPP_ERROR( rclcpp::get_logger("RMI"),"Invalid TermType requested. Supported values are FINE, CNT or CR.");
  }
  msg.append(std::to_string(term_info_.term_value_));
  //TODO: Add optional parameters
  msg.append(",\"NoBlend\":\"ON\"");
  
  msg.append("}\r\n");
  return msg;
}

const std::string RMICommunication::ins_LinearMotionJRep(int sequence_id_, MotionType motion_type_, JointAngle joint_angle_, SpeedInfo speed_info_, TermInfo term_info_)
{
  std::string msg;
  if (motion_type_ == MotionType::ABS) {
    msg.append("{\"Instruction\":\"FRC_LinearMotionJRep\",\"SequenceID\":");
  }
  else if (motion_type_ == MotionType::REL) {
    msg.append("{\"Instruction\":\"FRC_LinearRelativeJRep\",\"SequenceID\":");
  }
  else {
    RCLCPP_ERROR( rclcpp::get_logger("RMI"),"Invalid MotionType requested. Supported values are ABS or REL.");
  }
  msg.append(std::to_string(sequence_id_));
  msg.append(",\"JointAngles\":{\"J1\":");
  msg.append(std::to_string(joint_angle_.jnt_1_));
  msg.append(",\"J2\":");
  msg.append(std::to_string(joint_angle_.jnt_2_));
  msg.append(",\"J3\":");
  msg.append(std::to_string(joint_angle_.jnt_3_));
  msg.append(",\"J4\":");
  msg.append(std::to_string(joint_angle_.jnt_4_));
  msg.append(",\"J5\":");
  msg.append(std::to_string(joint_angle_.jnt_5_));
  msg.append(",\"J6\":");
  msg.append(std::to_string(joint_angle_.jnt_6_));
  msg.append(",\"J7\":");
  msg.append(std::to_string(joint_angle_.jnt_7_));
  msg.append(",\"J8\":");
  msg.append(std::to_string(joint_angle_.jnt_8_));
  msg.append(",\"J9\":");
  msg.append(std::to_string(joint_angle_.jnt_9_));
  if (speed_info_.speed_type_ == SpeedType::MMSEC) {
    msg.append("},\"SpeedType\":\"mmSec\",\"Speed\":");
  }
  else if (speed_info_.speed_type_ == SpeedType::INCHMIN) {
    msg.append("},\"SpeedType\":\"InchMin\",\"Speed\":");
  }
  else if (speed_info_.speed_type_ == SpeedType::TIME) {
    msg.append("},\"SpeedType\":\"Time\",\"Speed\":");
  }
  else {
    RCLCPP_ERROR( rclcpp::get_logger("RMI"),"Invalid SpeedType requested. Supported values are MMSEC, INCHMIN or TIME.");
  }
  msg.append(std::to_string(speed_info_.speed_value_));
  if (term_info_.term_type_ == TermType::FINE) {
    msg.append(",\"TermType\":\"FINE\",\"TermValue\":");
  }
  else if (term_info_.term_type_ == TermType::CNT) {
    msg.append(",\"TermType\":\"CNT\",\"TermValue\":");
  }
  else if (term_info_.term_type_ == TermType::CR) {
    msg.append(",\"TermType\":\"CR\",\"TermValue\":");
  }
  else {
    RCLCPP_ERROR( rclcpp::get_logger("RMI"),"Invalid TermType requested. Supported values are FINE, CNT or CR.");
  }
  msg.append(std::to_string(term_info_.term_value_));
  //TODO: Add optional parameters
  msg.append("}\r\n");
  return msg;
}

} // namespace