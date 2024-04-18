#include <fanuc_rmi/rmi_driver.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/clock.hpp>

#define pi 3.1415926

namespace rmi {

// --------------
// PUBLIC METHODS
// --------------

RMIDriver::RMIDriver()
    : timeout(5)
{
  state = State::VOID;
  mode = Mode::NONE;

  cmd_seq_id = 0;
  snd_seq_id = 0;
  rcv_seq_id = 0;

  alive = true;
  shutdown_flag = false;
  check_seq_id_flag = false;

  int error_thread;
  if (pthread_mutex_init(&lock, NULL) != 0) {
    std::cout<<"\033[1;31m[RMI DRIVER]: Mutex init has failed.\033[0m"<<std::endl;
    throw;
  }
  error_thread = pthread_create(&parsingThread, NULL, parsingThreadEntryFunction, this);
  if (error_thread != 0) {
    alive = false;
    std::cout<<"\033[1;31m[RMI DRIVER]: Error creating parsing thread: "<<strerror(error_thread)<<"\033[0m"<<std::endl;
    throw;
  }
  error_thread = pthread_create(&sendingThread, NULL, sendingThreadEntryFunction, this);
  if (error_thread != 0) {
    alive = false;
    std::cout<<"\033[1;31m[RMI DRIVER]: Error creating sending thread: "<<strerror(error_thread)<<"\033[0m"<<std::endl;
    throw;
  }
}

RMIDriver::~RMIDriver(){}

void RMIDriver::closeConnection()
{
  sendRequest(rmi_comm.cmd_Abort());
  while (state != State::ABORT) {
    auto start = std::chrono::steady_clock::now();
    if ((std::chrono::steady_clock::now()-start) < timeout) {
      if (state != State::ERROR) {
        std::cout<<"\033[1;33m[RMI DRIVER]: Waiting for remote controller to abort...\033[0m"<<std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
      }
      else {
        alive = false;
        close(socket_descriptor_);
        std::cout<<"\033[1;31m[RMI DRIVER]: An error occurred while attempting abort. Forcing...\033[0m"<<std::endl;
        break;
      }
    }
    else {
      alive = false;
      close(socket_descriptor_);
      std::cout<<"\033[1;31m[RMI DRIVER]: Remote controller did not abort within timeout. Forcing...\033[0m"<<std::endl;
      break;
    }
  }
  sendRequest(rmi_comm.cmc_Disconnect());
  std::cout<<"\033[1;31m[RMI DRIVER]: disconnect: "<<rmi_comm.cmc_Disconnect()<<"\033[0m"<<std::endl;
  auto start = std::chrono::steady_clock::now();
  while (state != State::DISCONNECTED) {
    if ((std::chrono::steady_clock::now()-start) < timeout) {
      if (state != State::ERROR) {
        std::cout<<"\033[1;33m[RMI DRIVER]: Waiting for remote controller to disconnect...\033[0m"<<std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
      }
      else {
        alive = false;
        close(socket_descriptor_);
        std::cout<<"\033[1;31m[RMI DRIVER]: An error occurred while attempting disconnection. Forcing...\033[0m"<<std::endl;
        break;
      }
    }
    else {
      alive = false;
      close(socket_descriptor_);
      std::cout<<"\033[1;31m[RMI DRIVER]: Remote controller did not disconnect within timeout. Forcing...\033[0m"<<std::endl;
      break;
    }
  }
  std::cout<<"\033[1;32m[RMI DRIVER]: Clean disconnection successful!\033[0m"<<std::endl;
  alive = false;
  ::shutdown(socket_descriptor_, SHUT_RDWR);
  close(socket_descriptor_);
}


bool RMIDriver::init(std::string ip_, int joint_number_)
{
  //save params
  connection.ip_address_ = ip_;
  joint_number = joint_number_;
  target_pos.resize(joint_number);
  current_pos.resize(joint_number);
  current_vel.resize(joint_number);
  current_eff.resize(joint_number);
  //ask RMI for the communication port and connect to it
  if (initSocket(16001) == 0) {
    sendRequest(rmi_comm.cmc_Connect());
    auto start = std::chrono::steady_clock::now();
    while (state != State::CONNECTED) {
      if ((std::chrono::steady_clock::now()-start) < timeout) {
        if (state != State::ERROR) {
          // std::cout<<"\033[1;33m[RMI DRIVER]: Waiting for remote controller to accept connection...\033[0m"<<std::endl;
          std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }
        else {
          close(socket_descriptor_);
          std::cout<<"\033[1;31m[RMI DRIVER]: An error occurred while attempting connection.\033[0m"<<std::endl;
          throw;
        }
      }
      else {
        close(socket_descriptor_);
        std::cout<<"\033[1;31m[RMI DRIVER]: Remote controller did not accept connection before timeout.\033[0m"<<std::endl;
        throw;
      }
    }
    close(socket_descriptor_);
    if (initSocket(connection.port_number_) != 0) {
      std::cout<<"\033[1;31m[RMI DRIVER]: Socket connection failed (controller port: "<< connection.port_number_ <<").\033[0m"<<std::endl;
      throw;
    }
  }
  else {
    std::cout<<"\033[1;31m[RMI DRIVER]: Socket connection failed (default port).\033[0m"<<std::endl;
    throw;
  }
  //check RMI status
  sendRequest(rmi_comm.cmd_GetStatus());
  auto start = std::chrono::steady_clock::now();
  while (state != State::READY) {
    if ((std::chrono::steady_clock::now()-start) < timeout) {
      if (state != State::ERROR) {
        if(state == State::NOT_READY) {
          state = State::RESET;
          sendRequest(rmi_comm.cmd_Reset());
          std::cout<<"\033[1;33m[RMI DRIVER]: A reset request has been sent.\033[0m"<<std::endl;
        }
        else {
          // std::cout<<"\033[1;33m[RMI DRIVER]: Checking if remote controller is ready...\033[0m"<<std::endl;
          std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }
      }
      else {
        std::cout<<"\033[1;31m[RMI DRIVER]: An error occurred while attempting to check remote controller status.\033[0m"<<std::endl;
        throw;
        return false;
      }
    }
    else {
      std::cout<<"\033[1;31m[RMI DRIVER]: Remote controller not ready in time.\033[0m"<<std::endl;
      throw;
      return false;
    }
  }
  //command RMI initialization
  sendRequest(rmi_comm.cmd_Initialize());
  start = std::chrono::steady_clock::now();
  while (state != State::INITIALIZED) {
    if ((std::chrono::steady_clock::now()-start) < timeout) {
      if (state != State::ERROR) {
        // std::cout<<"\033[1;33m[RMI DRIVER]: Waiting for remote controller to initialize...\033[0m"<<std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
      }
      else {
        std::cout<<"\033[1;31m[RMI DRIVER]: An error occurred while attempting initialization.\033[0m"<<std::endl;
        throw;
        return false;
      }
    }
    else {
      std::cout<<"\033[1;31m[RMI DRIVER]: Remote controller did not initialize before timeout.\033[0m"<<std::endl;
      throw;
      return false;
    }
  }
  std::cout<<"\033[1;32m[RMI DRIVER]: Connection successfully estabilished on "<<connection.ip_address_<<" port "<<connection.port_number_<<"!\033[0m"<<std::endl;
  sendRequest(rmi_comm.cmd_ReadJointAngles());
  while (!read_joint_angles_ok_) {
  std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  mtx_ja_.lock();
  read_joint_angles_ok_ = false;
  mtx_ja_.unlock();
  return true;
}


State RMIDriver::getState() {return state;}
Mode RMIDriver::getMode() {return mode;}
void RMIDriver::getRmiStatus(){sendRequest(rmi_comm.cmd_GetStatus());}


std::vector<double> RMIDriver::getPosition()
{
  // sendRequest(rmi_comm.cmd_ReadJointAngles());
  pthread_mutex_lock(&lock);
  current_pos.at(0) = double(joint_position.joint_angle_.jnt_1_*pi/180);
  current_pos.at(1) = double(joint_position.joint_angle_.jnt_2_*pi/180);
  current_pos.at(2) = double((joint_position.joint_angle_.jnt_3_+joint_position.joint_angle_.jnt_2_)*pi/180);
  current_pos.at(3) = double(joint_position.joint_angle_.jnt_4_*pi/180);
  current_pos.at(4) = double(joint_position.joint_angle_.jnt_5_*pi/180);
  current_pos.at(5) = double(joint_position.joint_angle_.jnt_6_*pi/180);
  pthread_mutex_unlock(&lock);
  return current_pos;
}


std::vector<double> RMIDriver::getVelocity()
{
  //remember to add mutex
  current_vel = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  return current_vel;
}


std::vector<double> RMIDriver::getEffort()
{
  //remember to add mutex
  current_eff = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  return current_eff;
}


void RMIDriver::setMode(Mode mode_) {mode = mode_;}


void RMIDriver::setState(State state_) {state = state_;}


void RMIDriver::setTargetPosition(std::vector<double> pos_target_)
{
  if (pos_target_.size() == joint_number) {
    SpeedInfo si = {SpeedType::PERC, 100};
    TermInfo ti = {TermType::CNT, 100};
    JointAngle ja;
    ja.jnt_1_ = float(pos_target_.at(0)*180/pi);
    ja.jnt_2_ = float(pos_target_.at(1)*180/pi);
    ja.jnt_3_ = float((pos_target_.at(2)-pos_target_.at(1))*180/pi);
    ja.jnt_4_ = float(pos_target_.at(3)*180/pi);
    ja.jnt_5_ = float(pos_target_.at(4)*180/pi);
    ja.jnt_6_ = float(pos_target_.at(5)*180/pi);
    ja.jnt_7_ = float(0.0);
    ja.jnt_8_ = float(0.0);
    ja.jnt_9_ = float(0.0);
    cmd_seq_id++;
    instruction_list.push(rmi_comm.ins_JointMotionJRep(cmd_seq_id, MotionType::ABS, ja, si, ti));
    // std::cout<<"\033[1;34m[setTargetPosition] "<<instruction_list.front()<<"\033[0m"<<std::endl<<std::flush;
  }
  else {
    std::cout<<"\033[1;31m[RMI DRIVER]: Requested joint positions is different from set joint number.\033[0m"<<std::endl;
    throw;
  }
}


void RMIDriver::setTargetPosVel(std::vector<double> pos_target_, int vel_target_, bool fine_motion_)
{
  if (pos_target_.size() == joint_number) {
    SpeedInfo si = {SpeedType::TIME, vel_target_};
    TermInfo ti;
    if (fine_motion_) {
      ti = {TermType::FINE, 100};
    }
    else {
      ti = {TermType::CNT, 100};
    }
    JointAngle ja;
    ja.jnt_1_ = float(pos_target_.at(0)*180/pi);
    ja.jnt_2_ = float(pos_target_.at(1)*180/pi);
    ja.jnt_3_ = float((pos_target_.at(2)-pos_target_.at(1))*180/pi);
    ja.jnt_4_ = float(pos_target_.at(3)*180/pi);
    ja.jnt_5_ = float(pos_target_.at(4)*180/pi);
    ja.jnt_6_ = float(pos_target_.at(5)*180/pi);
    ja.jnt_7_ = float(0.0);
    ja.jnt_8_ = float(0.0);
    ja.jnt_9_ = float(0.0);
    cmd_seq_id++;
    instruction_list.push(rmi_comm.ins_JointMotionJRep(cmd_seq_id, MotionType::ABS, ja, si, ti));
  }
  else {
    std::cout<<"\033[1;31m[RMI DRIVER]: Requested joint positions is different from set joint number.\033[0m"<<std::endl;
    throw;
  }
}

// -----------------
// PROTECTED METHODS
// -----------------

void RMIDriver::parsingThreadFunction()
{
  while (alive) {
    parseResponse();
    if (state == State::ERROR && !shutdown_flag) {
      shutdown_flag = true;
      std::cout<<"\033[1;31m[RMI DRIVER]: Error detected, requesting details...\033[0m"<<std::endl;
      sendRequest(rmi_comm.cmd_ReadError());
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
}

void RMIDriver::sendingThreadFunction()
{
  rclcpp::Clock c;
  rclcpp::Time start = c.now();
  bool read=true;
  while (alive) {
    if (state == State::INITIALIZED) {
      if (c.now()-start > rclcpp::Duration(0,35000000)) {
        start = c.now();
        sendRequest(rmi_comm.cmd_ReadJointAngles());

        // std::cout<<"\033[1;31m[RMI DRIVER]: ---------------------------...\033[0m"<<std::endl;
        // mtx_ja_.lock();
        // read = read_joint_angles_ok_;
        // mtx_ja_.unlock();
        
        // if(read)
        // {
        //   mtx_ja_.lock();
        //   read_joint_angles_ok_ = false;
        //   mtx_ja_.unlock();
        // }
        
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
      }
      if (instruction_list.size() > 0) {
        try {
          snd_seq_id = std::stoi(instruction_list.front().substr(instruction_list.front().find("SequenceID")+strlen("SequenceID")+2));
        }
        catch (std::invalid_argument) {
          std::cout<<"\033[1;33m[RMI DRIVER]: STOI error.\033[0m"<<std::endl;
        }
        catch (std::out_of_range) {
          std::cout<<"\033[1;33m[RMI DRIVER]: Out of range error.\033[0m"<<std::endl;
        }
        if(snd_seq_id - rcv_seq_id < 8) {
          sendRequest(instruction_list.front());
          std::cout<<"\033[1;33minstruction_list.front(): "<<instruction_list.front()<<".\033[0m"<<std::endl; // debug
          instruction_list.pop();
          std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
      }
    }
  }
  std::queue<std::string> empty;
  std::swap(instruction_list,empty);
}

// ---------------
// PRIVATE METHODS
// ---------------

int RMIDriver::initSocket(int port_number_)
{
  socket_descriptor_ = socket(AF_INET, SOCK_STREAM, 0);
  if (socket_descriptor_ < 0) {
    std::cout<<"\033[1;31m[RMI DRIVER]: "<<strerror(errno)<<"' during 'socket()'.\033[0m"<<std::endl;
    return errno;
  }
  memset(&socket_address_, 0, sizeof(socket_address_));
  socket_address_length_ = sizeof(sockaddr_in);
  socket_address_.sin_family = AF_INET;
  socket_address_.sin_port = htons(port_number_);
  if (inet_pton(AF_INET, connection.ip_address_.c_str(), &(socket_address_.sin_addr)) != 1) {
    std::cout<<"\033[1;31m[RMI DRIVER]: "<<strerror(errno)<<"' during 'inet_pton()'.\033[0m"<<std::endl;
    pthread_mutex_unlock(&lock);
    return errno;
  }
  // if (connect(socket_descriptor_, (struct sockaddr *)&socket_address_, sizeof(socket_address_)) < 0) {
  //   std::cout<<"\033[1;31m[RMI DRIVER]: "<<strerror(errno)<<"' during 'connect()'.\033[0m"<<std::endl;
  //   return errno;
  // }
  // fcntl(socket_descriptor_, F_SETFL, O_NONBLOCK);
  int ret = 0, count = 0;
  fcntl(socket_descriptor_, F_SETFL, O_NONBLOCK);
  ret = connect(socket_descriptor_, (struct sockaddr *)&socket_address_, sizeof(socket_address_));
  while (ret < 0 && count < 10) {
    if (errno == EINPROGRESS) {
      // std::cout<<"\033[1;33m[RMI DRIVER]: "<<strerror(errno)<<": attempting to connect to socket...\033[0m"<<std::endl;
    }
    ret = connect(socket_descriptor_, (struct sockaddr *)&socket_address_, sizeof(socket_address_));
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    count ++;
  }
  if (ret < 0) {
    return -1;
  }
  return 0;
}

bool RMIDriver::sendRequest(const std::string msg_out)
{ 
  std::cout<<"[sendRequest]: request :" <<msg_out<<std::endl<<std::flush; //debug
  ssize_t sent = ::sendto(socket_descriptor_, msg_out.c_str(), msg_out.length()+1, 0, (const sockaddr *)&socket_address_, socket_address_length_);
  if (sent > 0) 
  {  
    std::cout<<"[sendRequest]: SENT "<<std::endl<<std::flush; // debug
    return true;
  }
  else {
    std::cout<<"[sendRequest]: NOT SENT "<<std::endl<<std::flush;  // debug
    return false;
  }
}

bool RMIDriver::parseResponse()
{
  char msg[1000];
  bool ret = false;
  ssize_t received = ::recvfrom(socket_descriptor_, msg, sizeof(msg), 0, (sockaddr *)&socket_address_, &socket_address_length_);

  if (received > 0) {
    std::string msg_in(msg,received-3);
    // std::cout<<"\033[1;33m[parseResponse]: msg in :" <<msg_in<<"\033[0m"<<std::endl<<std::flush;
    if (msg_in.find("Communication") != std::string::npos) {
      ret = parseCommunication(msg_in);
      return ret;
    }
    else if (msg_in.find("Command") != std::string::npos) {
      ret = parseCommand(msg_in);
      return ret;
    }
    else if (msg_in.find("Instruction") != std::string::npos) {
      ret = parseInstruction(msg_in);
      return ret;
    }
    else {
      std::cout<<"\033[1;33m[RMI DRIVER]: Unknown packet type received.\033[0m"<<std::endl;
      return ret;
    }
  }
  else {
    return false;
  }
  
}

bool RMIDriver::parseCommunication(std::string msg_in)
{
  
  std::cout<<"\033[1;36m[parseCommunication]: response :" <<msg_in<<"\033[0m"<<std::endl<<std::flush;
  
  if (msg_in.find("FRC_Connect") != std::string::npos) {
    return parseConnect(msg_in);
  }
  else if (msg_in.find("FRC_Disconnect") != std::string::npos) {
    return parseDisconnect(msg_in);
  }
  else if (msg_in.find("FRC_Terminate") != std::string::npos) {
    return parseTerminate();
  }
  else if (msg_in.find("FRC_SystemFault") != std::string::npos) {
    return parseSystemFault(msg_in);
  }
  else {
    std::cout<<"\033[1;33m[RMI DRIVER]: Unknown communication packet received.\033[0m"<<std::endl;
    return false;
  }
}

bool RMIDriver::parseCommand(std::string msg_in)
{
  std::cout<<"\033[1;32m[parseCommand]: response :" <<msg_in<<"\033[0m"<<std::endl<<std::flush;  // debug
  
  if (msg_in.find("FRC_Initialize") != std::string::npos) {
    return parseInitialize(msg_in);
  }
  else if (msg_in.find("FRC_Abort") != std::string::npos) {
    return parseAbort(msg_in);
  }
  else if (msg_in.find("FRC_Pause") != std::string::npos) {
    return parsePause(msg_in);
  }
  else if (msg_in.find("FRC_Continue") != std::string::npos) {
    return parseContinue(msg_in);
  }
  else if (msg_in.find("FRC_ReadError") != std::string::npos) {
    return parseReadError(msg_in);
  }
  else if (msg_in.find("FRC_SetUFrameUTool") != std::string::npos) {
    return parseSetUFrameUTool(msg_in);
  }
  else if (msg_in.find("FRC_GetStatus") != std::string::npos) {
    return parseGetStatus(msg_in);
  }
  else if (msg_in.find("FRC_ReadUFrameData") != std::string::npos) {
    return parseReadUFrameData(msg_in);
  }
  else if (msg_in.find("FRC_WriteUFrameData") != std::string::npos) {
    return parseWriteUFrameData(msg_in);
  }
  else if (msg_in.find("FRC_ReadUToolData") != std::string::npos) {
    return parseReadUToolData(msg_in);
  }
  else if (msg_in.find("FRC_WriteUToolData") != std::string::npos) {
    return parseWriteUToolData(msg_in);
  }
  else if (msg_in.find("FRC_ReadDIN") != std::string::npos) {
    return parseReadDIN(msg_in);
  }
  else if (msg_in.find("FRC_WriteDOUT") != std::string::npos) {
    return parseWriteDOUT(msg_in);
  }
  else if (msg_in.find("FRC_ReadCartesianPosition") != std::string::npos) {
    return parseReadCartesianPosition(msg_in);
  }
  else if (msg_in.find("FRC_ReadJointAngles") != std::string::npos) {
    return parseReadJointAngles(msg_in);
  }
  else if (msg_in.find("FRC_SetOverRide") != std::string::npos) {
    return parseSetOverRide(msg_in);
  }
  else if (msg_in.find("FRC_GetUFrameUTool") != std::string::npos) {
    return parseGetUFrameUTool(msg_in);
  }
  else if (msg_in.find("FRC_ReadPositionRegister") != std::string::npos) {
    return parseReadPositionRegister(msg_in);
  }
  else if (msg_in.find("FRC_WritePositionRegister") != std::string::npos) {
    return parseWritePositionRegister(msg_in);
  }
  else if (msg_in.find("FRC_Reset") != std::string::npos) {
    return parseReset(msg_in);
  }
  else if (msg_in.find("FRC_ReadTCPSpeed") != std::string::npos) {
    return parseReadTCPSpeed(msg_in);
  }
  else {
    std::cout<<"\033[1;33m[RMI DRIVER]: Unknown command packet received.\033[0m"<<std::endl;
    return true;
  }
}

bool RMIDriver::parseInstruction(std::string msg_in)
{
  std::cout<<"\033[1;31[parseInstruction]: "<< msg_in <<".\033[0m"<<std::endl<<std::flush; // debug
  int error_id = std::stoi(msg_in.substr(msg_in.find("ErrorID")+strlen("ErrorID")+4));
  if (error_id != 0) {
    state = State::ERROR;
  }
  else {
    rcv_seq_id = std::stoi(msg_in.substr(msg_in.find("SequenceID")+strlen("SequenceID")+4));
    instruction_parsed_=true;
  }
  
  return true;
}

//Communication

bool RMIDriver::parseConnect(std::string msg_in)
{
  int error_id = std::stoi(msg_in.substr(msg_in.find("ErrorID")+strlen("ErrorID")+4));
  if (error_id == 0) {
    state = State::CONNECTED;
    connection.port_number_ = std::stoi(msg_in.substr(msg_in.find("PortNumber")+strlen("PortNumber")+4));
    connection.major_version_ = std::stoi(msg_in.substr(msg_in.find("MajorVersion")+strlen("MajorVersion")+4));
    connection.minor_version_ = std::stoi(msg_in.substr(msg_in.find("MinorVersion")+strlen("MinorVersion")+4));
  }
  else {
    state = State::ERROR;
  }
  return true;
}

bool RMIDriver::parseDisconnect(std::string msg_in)
{
  int error_id = std::stoi(msg_in.substr(msg_in.find("ErrorID")+strlen("ErrorID")+4));
  if (error_id == 0) {
    state = State::DISCONNECTED;
  }
  else {
    state = State::ERROR;
  }
  return true;
}

bool RMIDriver::parseTerminate()
{
  state = State::DISCONNECTED;
  std::cout<<"\033[1;31m[RMI DRIVER]: Received termination signal from remote controller.\033[0m"<<std::endl;

  return true;
}

bool RMIDriver::parseSystemFault(std::string msg_in)
{
  int fault_seq_id = std::stoi(msg_in.substr(msg_in.find("SequenceID")+strlen("SequenceID")+4));
  std::cout<<"\033[1;31m[RMI DRIVER]: System fault occurred for sequence ID: "<<fault_seq_id<<"\033[0m"<<std::endl;
  state = State::ERROR;
  return true;
}


//Command

bool RMIDriver::parseInitialize(std::string msg_in)
{
  std::cout<<"\033[1;36m[parseCommunication]: response :" <<msg_in<<"\033[0m"<<std::endl<<std::flush;

  int error_id = std::stoi(msg_in.substr(msg_in.find("ErrorID")+strlen("ErrorID")+4));
  if (error_id == 0) {
    state = State::INITIALIZED;
  }
  else {
    state = State::ERROR;
    std::cout<<"---------------- Error occurred during initialization."<<std::endl;
  }
  return true;
}

bool RMIDriver::parseAbort(std::string msg_in)
{
  int error_id = std::stoi(msg_in.substr(msg_in.find("ErrorID")+strlen("ErrorID")+4));
  if (error_id != 0) {
    state = State::ERROR;
  }
  state = State::ABORT;
  return true;
}

bool RMIDriver::parsePause(std::string msg_in)
{
  int error_id = std::stoi(msg_in.substr(msg_in.find("ErrorID")+strlen("ErrorID")+4));
  if (error_id != 0) {
    state = State::ERROR;
  }
  return true;
}

bool RMIDriver::parseContinue(std::string msg_in)
{
  int error_id = std::stoi(msg_in.substr(msg_in.find("ErrorID")+strlen("ErrorID")+4));
  if (error_id != 0) {
    state = State::ERROR;
  }
  return true;
}

bool RMIDriver::parseReadError(std::string msg_in)
{
  std::string tmp_str = msg_in.substr(msg_in.find("ErrorData")+strlen("ErrorData")+5);
  std::string error_data = tmp_str.substr(0,tmp_str.length()-3);
  std::cout<<"\033[1;31m[RMI DRIVER]: Error code: "<<error_data<<". Shutting down...\033[0m"<<std::endl;
  closeConnection();
  std::terminate();

  return true;
}

bool RMIDriver::parseSetUFrameUTool(std::string msg_in)
{
  int error_id = std::stoi(msg_in.substr(msg_in.find("ErrorID")+strlen("ErrorID")+4));
  if (error_id != 0) {
    state = State::ERROR;
  }
  return true;
}

bool RMIDriver::parseGetStatus(std::string msg_in)
{
  std::cout<<"\033[1;36m[parseGetStatus]: response :" <<msg_in<<"\033[0m"<<std::endl<<std::flush;

  int error_id = std::stoi(msg_in.substr(msg_in.find("ErrorID")+strlen("ErrorID")+4));
  status.servo_ready_ = std::stoi(msg_in.substr(msg_in.find("ServoReady")+strlen("ServoReady")+4));
  status.tp_mode_ = std::stoi(msg_in.substr(msg_in.find("TPMode")+strlen("TPMode")+4));
  status.rmi_motion_status_ = std::stoi(msg_in.substr(msg_in.find("RMIMotionStatus")+strlen("RMIMotionStatus")+4));
  status.program_status_ = std::stoi(msg_in.substr(msg_in.find("ProgramStatus")+strlen("ProgramStatus")+4));
  status.single_step_mode_ = std::stoi(msg_in.substr(msg_in.find("SingleStepMode")+strlen("SingleStepMode")+4));
  status.number_u_tool_ = std::stoi(msg_in.substr(msg_in.find("NumberUTool")+strlen("NumberUTool")+4));
  status.number_u_frame_ = std::stoi(msg_in.substr(msg_in.find("NumberUFrame")+strlen("NumberUFrame")+4));
  if (check_seq_id_flag) {
    status.next_sequence_id_ = std::stoi(msg_in.substr(msg_in.find("NextSequenceID")+strlen("NextSequenceID")+4));
  }
  
  if (error_id != 0) {
    state = State::NOT_READY;
  }
  else if (status.servo_ready_ !=1) {
    std::cout<<"\033[1;31m[RMI DRIVER]: Servos not ready.\033[0m"<<std::endl;
    throw;
  }
  else if (status.tp_mode_ != 0) {
    std::cout<<"\033[1;31m[RMI DRIVER]: TP is enabled.\033[0m"<<std::endl;
    throw;
  }
  else {state = State::READY;}
  
  return true;
}

bool RMIDriver::parseReadUFrameData(std::string msg_in)
{ 
  int error_id = std::stoi(msg_in.substr(msg_in.find("ErrorID")+strlen("ErrorID")+4));
  frame_info.frame_number_ = std::stoi(msg_in.substr(msg_in.find("FrameNumber")+strlen("FrameNumber")+4));
  frame_info.pose_.x_ = std::stof(msg_in.substr(msg_in.find("X\"")+strlen("X")+3));
  frame_info.pose_.y_ = std::stof(msg_in.substr(msg_in.find("Y\"")+strlen("Y")+3));
  frame_info.pose_.z_ = std::stof(msg_in.substr(msg_in.find("Z\"")+strlen("Z")+3));
  frame_info.pose_.w_ = std::stof(msg_in.substr(msg_in.find("W\"")+strlen("W")+3));
  frame_info.pose_.p_ = std::stof(msg_in.substr(msg_in.find("P\"")+strlen("P")+3));
  frame_info.pose_.r_ = std::stof(msg_in.substr(msg_in.find("R\"")+strlen("R")+3));
  if (error_id != 0) {
    state = State::ERROR;
  }
  return true;
}

bool RMIDriver::parseWriteUFrameData(std::string msg_in)
{
  int error_id = std::stoi(msg_in.substr(msg_in.find("ErrorID")+strlen("ErrorID")+4));
  if (error_id != 0) {
    state = State::ERROR;
  }
  return true;
}

bool RMIDriver::parseReadUToolData(std::string msg_in)
{
  int error_id = std::stoi(msg_in.substr(msg_in.find("ErrorID")+strlen("ErrorID")+4));
  tool_info.tool_number_ = std::stoi(msg_in.substr(msg_in.find("ToolNumber")+strlen("ToolNumber")+4));
  tool_info.pose_.x_ = std::stof(msg_in.substr(msg_in.find("X\"")+strlen("X")+3));
  tool_info.pose_.y_ = std::stof(msg_in.substr(msg_in.find("Y\"")+strlen("Y")+3));
  tool_info.pose_.z_ = std::stof(msg_in.substr(msg_in.find("Z\"")+strlen("Z")+3));
  tool_info.pose_.w_ = std::stof(msg_in.substr(msg_in.find("W\"")+strlen("W")+3));
  tool_info.pose_.p_ = std::stof(msg_in.substr(msg_in.find("P\"")+strlen("P")+3));
  tool_info.pose_.r_ = std::stof(msg_in.substr(msg_in.find("R\"")+strlen("R")+3));
  if (error_id != 0) {
    state = State::ERROR;
  }
  return true;
}

bool RMIDriver::parseWriteUToolData(std::string msg_in)
{
  int error_id = std::stoi(msg_in.substr(msg_in.find("ErrorID")+strlen("ErrorID")+4));
  if (error_id != 0) {
    state = State::ERROR;
  }
  return true;
}

bool RMIDriver::parseReadDIN(std::string msg_in)
{
  int error_id = std::stoi(msg_in.substr(msg_in.find("ErrorID")+strlen("ErrorID")+4));
  din_info.port_number_ = std::stoi(msg_in.substr(msg_in.find("PortNumber")+strlen("PortNumber")+4));
  din_info.port_value_ = std::stoi(msg_in.substr(msg_in.find("PortValue")+strlen("PortValue")+4));
  if (error_id != 0) {
    state = State::ERROR;
  }
  return true;
}

bool RMIDriver::parseWriteDOUT(std::string msg_in)
{
  int error_id = std::stoi(msg_in.substr(msg_in.find("ErrorID")+strlen("ErrorID")+4));
  if (error_id != 0) {
    state = State::ERROR;
  }
  return true;
}

bool RMIDriver::parseReadCartesianPosition(std::string msg_in)
{
  int error_id = std::stoi(msg_in.substr(msg_in.find("ErrorID")+strlen("ErrorID")+4));
  cartesian_position.time_tag_ = std::stoi(msg_in.substr(msg_in.find("TimeTag")+strlen("TimeTag")+4));
  cartesian_position.configuration_.u_tool_number_ = std::stoi(msg_in.substr(msg_in.find("UToolNumber")+strlen("UToolNumber")+4));
  cartesian_position.configuration_.u_frame_number_ = std::stoi(msg_in.substr(msg_in.find("UFrameNumber")+strlen("UFrameNumber")+4));
  cartesian_position.configuration_.front_ = std::stoi(msg_in.substr(msg_in.find("Front")+strlen("Front")+4));
  cartesian_position.configuration_.up_ = std::stoi(msg_in.substr(msg_in.find("Up")+strlen("Up")+4));
  cartesian_position.configuration_.left_ = std::stoi(msg_in.substr(msg_in.find("Left")+strlen("Left")+4));
  cartesian_position.configuration_.flip_ = std::stoi(msg_in.substr(msg_in.find("Flip")+strlen("Flip")+4));
  cartesian_position.configuration_.turn_4_ = std::stoi(msg_in.substr(msg_in.find("Turn4")+strlen("Turn4")+4));
  cartesian_position.configuration_.turn_5_ = std::stoi(msg_in.substr(msg_in.find("Turn5")+strlen("Turn5")+4));
  cartesian_position.configuration_.turn_6_ = std::stoi(msg_in.substr(msg_in.find("Turn6")+strlen("Turn6")+4));
  cartesian_position.pose_.x_ = std::stof(msg_in.substr(msg_in.find("X\"")+strlen("X")+3));
  cartesian_position.pose_.y_ = std::stof(msg_in.substr(msg_in.find("Y\"")+strlen("Y")+3));
  cartesian_position.pose_.z_ = std::stof(msg_in.substr(msg_in.find("Z\"")+strlen("Z")+3));
  cartesian_position.pose_.w_ = std::stof(msg_in.substr(msg_in.find("W\"")+strlen("W")+3));
  cartesian_position.pose_.p_ = std::stof(msg_in.substr(msg_in.find("P\"")+strlen("P")+3));
  cartesian_position.pose_.r_ = std::stof(msg_in.substr(msg_in.find("R\"")+strlen("R")+3));
  if (error_id != 0) {
    state = State::ERROR;
  }
  return true;
}

bool RMIDriver::parseReadJointAngles(std::string msg_in)
{
  int error_id = std::stoi(msg_in.substr(msg_in.find("ErrorID")+strlen("ErrorID")+4));
  pthread_mutex_lock(&lock);
  joint_position.time_tag_ = std::stoi(msg_in.substr(msg_in.find("TimeTag")+strlen("TimeTag")+4));
  joint_position.joint_angle_.jnt_1_ = std::stof(msg_in.substr(msg_in.find("J1")+strlen("J1")+4));
  joint_position.joint_angle_.jnt_2_ = std::stof(msg_in.substr(msg_in.find("J2")+strlen("J2")+4));
  joint_position.joint_angle_.jnt_3_ = std::stof(msg_in.substr(msg_in.find("J3")+strlen("J3")+4));
  joint_position.joint_angle_.jnt_4_ = std::stof(msg_in.substr(msg_in.find("J4")+strlen("J4")+4));
  joint_position.joint_angle_.jnt_5_ = std::stof(msg_in.substr(msg_in.find("J5")+strlen("J5")+4));
  joint_position.joint_angle_.jnt_6_ = std::stof(msg_in.substr(msg_in.find("J6")+strlen("J6")+4));
  joint_position.joint_angle_.jnt_7_ = std::stof(msg_in.substr(msg_in.find("J7")+strlen("J7")+4));
  joint_position.joint_angle_.jnt_8_ = std::stof(msg_in.substr(msg_in.find("J8")+strlen("J8")+4));
  joint_position.joint_angle_.jnt_9_ = std::stof(msg_in.substr(msg_in.find("J9")+strlen("J9")+4));
  pthread_mutex_unlock(&lock);
  if (error_id != 0) {
    state = State::ERROR;
  }

  mtx_ja_.lock();
  read_joint_angles_ok_ = true;
  mtx_ja_.unlock();
  return true;
}

bool RMIDriver::parseSetOverRide(std::string msg_in)
{
  int error_id = std::stoi(msg_in.substr(msg_in.find("ErrorID")+strlen("ErrorID")+4));
  if (error_id != 0) {
    state = State::ERROR;
  }
  return true;
}

bool RMIDriver::parseGetUFrameUTool(std::string msg_in)
{
  int error_id = std::stoi(msg_in.substr(msg_in.find("ErrorID")+strlen("ErrorID")+4));
  frame_id = std::stoi(msg_in.substr(msg_in.find("UFrameNumber")+strlen("UFrameNumber")+4));
  tool_id = std::stoi(msg_in.substr(msg_in.find("UToolNumber")+strlen("UToolNumber")+4));
  if (error_id != 0) {
    state = State::ERROR;
  }
  return true;
}

bool RMIDriver::parseReadPositionRegister(std::string msg_in)
{
  int error_id = std::stoi(msg_in.substr(msg_in.find("ErrorID")+strlen("ErrorID")+4));
  position_register_info.register_number_ = std::stoi(msg_in.substr(msg_in.find("RegisterNumber")+strlen("RegisterNumber")+4));
  position_register_info.configuration_.u_tool_number_ = std::stoi(msg_in.substr(msg_in.find("UToolNumber")+strlen("UToolNumber")+4));
  position_register_info.configuration_.u_frame_number_ = std::stoi(msg_in.substr(msg_in.find("UFrameNumber")+strlen("UFrameNumber")+4));
  position_register_info.configuration_.front_ = std::stoi(msg_in.substr(msg_in.find("Front")+strlen("Front")+4));
  position_register_info.configuration_.up_ = std::stoi(msg_in.substr(msg_in.find("Up")+strlen("Up")+4));
  position_register_info.configuration_.left_ = std::stoi(msg_in.substr(msg_in.find("Left")+strlen("Left")+4));
  position_register_info.configuration_.flip_ = std::stoi(msg_in.substr(msg_in.find("Flip")+strlen("Flip")+4));
  position_register_info.configuration_.turn_4_ = std::stoi(msg_in.substr(msg_in.find("Turn4")+strlen("Turn4")+4));
  position_register_info.configuration_.turn_5_ = std::stoi(msg_in.substr(msg_in.find("Turn5")+strlen("Turn5")+4));
  position_register_info.configuration_.turn_6_ = std::stoi(msg_in.substr(msg_in.find("Turn6")+strlen("Turn6")+4));
  position_register_info.pose_.x_ = std::stof(msg_in.substr(msg_in.find("X\"")+strlen("X")+3));
  position_register_info.pose_.y_ = std::stof(msg_in.substr(msg_in.find("Y\"")+strlen("Y")+3));
  position_register_info.pose_.z_ = std::stof(msg_in.substr(msg_in.find("Z\"")+strlen("Z")+3));
  position_register_info.pose_.w_ = std::stof(msg_in.substr(msg_in.find("W\"")+strlen("W")+3));
  position_register_info.pose_.p_ = std::stof(msg_in.substr(msg_in.find("P\"")+strlen("P")+3));
  position_register_info.pose_.r_ = std::stof(msg_in.substr(msg_in.find("R\"")+strlen("R")+3));
  if (error_id != 0) {
    state = State::ERROR;
  }
  return true;
}

bool RMIDriver::parseWritePositionRegister(std::string msg_in)
{
  int error_id = std::stoi(msg_in.substr(msg_in.find("ErrorID")+strlen("ErrorID")+4));
  if (error_id != 0) {
    state = State::ERROR;
  }
  return true;
}

bool RMIDriver::parseReset(std::string msg_in)
{
  int error_id = std::stoi(msg_in.substr(msg_in.find("ErrorID")+strlen("ErrorID")+4));
  if (error_id != 0) {
    state = State::ERROR;
  }
  else {
    state = State::READY;
  }
  return true;
}

bool RMIDriver::parseReadTCPSpeed(std::string msg_in)
{
  int error_id = std::stoi(msg_in.substr(msg_in.find("ErrorID")+strlen("ErrorID")+4));
  tcp_speed_info.time_tag_ = std::stoi(msg_in.substr(msg_in.find("TimeTag")+strlen("TimeTag")+4));
  tcp_speed_info.tcp_speed_ = std::stof(msg_in.substr(msg_in.find("Speed")+strlen("Speed")+4));
  if (error_id != 0) {
    state = State::ERROR;
  }
  return true;
}


} // namespace