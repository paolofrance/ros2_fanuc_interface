#ifndef CRX10IAL_RMI_COMMUNICATION_H
#define CRX10IAL_RMI_COMMUNICATION_H

#include <mutex>
#include <iostream>
#include <netinet/in.h>

namespace rmi {

  typedef enum {
    COMMUNICATION,
    COMMAND,
    INSTRUCTION
  } CmdType;

  typedef enum {
    ABS,
    REL
  } MotionType;

  typedef enum {
    MMSEC,
    INCHMIN,
    TIME,
    PERC,
  } SpeedType;

  typedef enum {
    FINE,
    CNT,
    CR
  } TermType;

  typedef enum {
    ON,
    OFF
  } PortValue;

    typedef enum {
    DISCONNECTED,
    CONNECTED,
    NOT_READY,
    READY,
    INITIALIZED,
    RESET,
    ERROR
  } State;

  typedef enum {
    NONE,
    IDLE,
    POSITION,
    VELOCITY,
    POSVEL,
    EFFORT
  } Mode;

  typedef struct {
    std::string ip_address_;
    int port_number_;
    int major_version_;
    int minor_version_;
  } Connection;

  typedef struct {
    int servo_ready_;
    int tp_mode_;
    int rmi_motion_status_;
    int program_status_;
    int single_step_mode_;
    int number_u_tool_;
    int number_u_frame_;
    int next_sequence_id_;
  } Status;

  typedef struct {
    float x_;
    float y_;
    float z_;
    float w_;
    float p_;
    float r_;
    float ext_1_;
    float ext_2_;
    float ext_3_;
  } Pose;

  typedef struct {
    float jnt_1_;
    float jnt_2_;
    float jnt_3_;
    float jnt_4_;
    float jnt_5_;
    float jnt_6_;
    float jnt_7_;
    float jnt_8_;
    float jnt_9_;
  } JointAngle;

  typedef struct {
    int u_tool_number_;
    int u_frame_number_;
    int front_;
    int up_;
    int left_;
    int flip_;
    int turn_4_;
    int turn_5_;
    int turn_6_;
  } Configuration;

  typedef struct {
    int time_tag_;
    Configuration configuration_;
    Pose pose_;
  } CartesianPosition;

  typedef struct {
    int time_tag_;
    JointAngle joint_angle_;
  } JointPosition;

  typedef struct {
    int register_number_;
    Configuration configuration_;
    Pose pose_;
  } PositionRegisterInfo;

  typedef struct {
    int time_tag_;
    float tcp_speed_;
  } TCPSpeedInfo;

  typedef struct {
    SpeedType speed_type_;
    int speed_value_;
  } SpeedInfo;

  typedef struct {
    TermType term_type_;
    int term_value_;
  } TermInfo;

  typedef struct {
    int frame_number_;
    Pose pose_;
  } FrameInfo;

  typedef struct {
    int tool_number_;
    Pose pose_;
  } ToolInfo;

  typedef struct {
    int port_number_;
    int port_value_;
  } DINInfo;

class RMICommunication {

  public:

    //Constructor and Destructor
    RMICommunication();
    ~RMICommunication();

    //Communication methods
    const std::string cmc_Connect();
    const std::string cmc_Disconnect();

    //Command methods
    const std::string cmd_Initialize();
    const std::string cmd_Abort();
    const std::string cmd_Pause();
    const std::string cmd_Continue();
    const std::string cmd_ReadError();
    const std::string cmd_SetUFrameUTool(int frame_number_, int tool_number_);
    const std::string cmd_GetStatus();
    const std::string cmd_ReadUFrameData(int frame_number_);
    const std::string cmd_WriteUFrameData(int frame_number_, Pose pose_);
    const std::string cmd_ReadUToolData(int tool_number_);
    const std::string cmd_WriteUToolData(int tool_number_, Pose pose_);
    const std::string cmd_ReadDIN(int input_port_number_);
    const std::string cmd_WriteDOUT(int output_port_number_, PortValue port_value_);
    const std::string cmd_ReadCartesianPosition();
    const std::string cmd_ReadJointAngles();
    const std::string cmd_SetOverRide(int override_);
    const std::string cmd_GetUFrameUTool();
    const std::string cmd_ReadPositionRegister(int register_number_);
    const std::string cmd_WritePositionRegister(int register_number_, Configuration configuration_, Pose pose_);
    const std::string cmd_Reset();
    const std::string cmd_ReadTCPSpeed();

    //Instruction methods
    const std::string ins_WaitDin(int sequence_id_, int input_port_number_, PortValue port_value_);
    const std::string ins_SetUFrame(int sequence_id_, int frame_number_);
    const std::string ins_SetUTool(int sequence_id_, int tool_number_);
    const std::string ins_WaitTime(int sequence_id_, float time_);
    const std::string ins_PayLoad(int sequence_id_, int schedule_number_);
    const std::string ins_LinearMotion(int sequence_id_, MotionType motion_type_, Configuration configuration_, Pose pose_, SpeedInfo speed_info_, TermInfo term_info_);
    const std::string ins_JointMotion(int sequence_id_, MotionType motion_type_, Configuration configuration_, Pose pose_, SpeedInfo speed_info_, TermInfo term_info_);
    const std::string ins_CircularMotion(int sequence_id_, MotionType motion_type_, Configuration configuration_, Configuration via_configuration_, Pose pose_, Pose via_pose_, SpeedInfo speed_info_, TermInfo term_info_);
    const std::string ins_JointMotionJRep(int sequence_id_, MotionType motion_type_, JointAngle joint_angle_, SpeedInfo speed_info_, TermInfo term_info_);
    const std::string ins_LinearMotionJRep(int sequence_id_, MotionType motion_type_, JointAngle joint_angle_, SpeedInfo speed_info_, TermInfo term_info_);

}; //class
}  //namespace

#endif //CRX10IAL_RMI_COMMUNICATION_H