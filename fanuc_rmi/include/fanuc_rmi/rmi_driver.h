#ifndef CRX10IAL_RMI_DRIVER_H
#define CRX10IAL_RMI_DRIVER_H

// unix libraries
#include <arpa/inet.h>
#include <netdb.h>
#include <netinet/in.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <unistd.h>
#include <fcntl.h>

// standard libraries
#include <climits>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <regex>
#include <mutex>
#include <sstream>
#include <thread>
#include <pthread.h>
#include <queue>
#include <atomic>
#include <memory>
#include <fanuc_rmi/rmi_communication.h>

namespace rmi {
class RMIDriver {

  public:

    RMIDriver();
    ~RMIDriver();
    bool init(std::string ip_, int joint_number_);
    void closeConnection();

    State getState();
    Mode getMode();
    void getRmiStatus();
    
    std::vector<double> getPosition();
    std::vector<double> getVelocity();
    std::vector<double> getEffort();

    void setMode(Mode mode_);
    void setState(State state);
    void setTargetPosition(std::vector<double> pos_target_);
    void setTargetPosVel(std::vector<double> pos_target_, int vel_target_, bool fine_motion_);

    bool instruction_parsed_=false;
    bool read_joint_angles_ok_ = false;
    std::mutex mtx_ja_;

    RMICommunication rmi_comm;
    
    bool sendRequest(const std::string msg_out);
    bool parseResponse();

  protected:

    void parsingThreadFunction();
    void sendingThreadFunction();

  private:

    sockaddr_in socket_address_;
    socklen_t socket_address_length_;
    int socket_descriptor_,socket_descriptor_new_;

    bool alive;
    std::chrono::seconds timeout;
    pthread_t parsingThread, sendingThread;
    pthread_mutex_t lock;
    static void * parsingThreadEntryFunction(void * This) {((RMIDriver *)This)->parsingThreadFunction(); return NULL;}
    static void * sendingThreadEntryFunction(void * This) {((RMIDriver *)This)->sendingThreadFunction(); return NULL;}
    
    std::atomic<State> state;
    std::atomic<Mode> mode;

    Connection connection;
    Status status;
    FrameInfo frame_info;
    ToolInfo tool_info;
    DINInfo din_info;
    TCPSpeedInfo tcp_speed_info;
    PositionRegisterInfo position_register_info;
    CartesianPosition cartesian_position;
    JointPosition joint_position;
    
    int joint_number;
    int tool_id, frame_id;
    std::atomic<int> cmd_seq_id, snd_seq_id, rcv_seq_id;
    std::atomic<bool> shutdown_flag, check_seq_id_flag;

    std::queue<std::string> instruction_list;
    std::vector<double> target_pos;
    std::vector<double> current_pos;
    std::vector<double> current_vel;
    std::vector<double> current_eff;

    int initSocket(int port_number_);
    int initSocketNew(int port_number_);
    bool parseCommunication(std::string msg_in);
    bool parseCommand(std::string msg_in);
    bool parseInstruction(std::string msg_in);
    
    //Communication
    bool parseConnect(std::string msg_in);
    bool parseDisconnect(std::string msg_in);
    bool parseTerminate();
    bool parseSystemFault(std::string msg_in);

    //Command
    bool parseInitialize(std::string msg_in);
    bool parseAbort(std::string msg_in);
    bool parsePause(std::string msg_in);
    bool parseContinue(std::string msg_in);
    bool parseReadError(std::string msg_in);
    bool parseSetUFrameUTool(std::string msg_in);
    bool parseGetStatus(std::string msg_in);
    bool parseReadUFrameData(std::string msg_in);
    bool parseWriteUFrameData(std::string msg_in);
    bool parseReadUToolData(std::string msg_in);
    bool parseWriteUToolData(std::string msg_in);
    bool parseReadDIN(std::string msg_in);
    bool parseWriteDOUT(std::string msg_in);
    bool parseReadCartesianPosition(std::string msg_in);
    bool parseReadJointAngles(std::string msg_in);
    bool parseSetOverRide(std::string msg_in);
    bool parseGetUFrameUTool(std::string msg_in);
    bool parseReadPositionRegister(std::string msg_in);
    bool parseWritePositionRegister(std::string msg_in);
    bool parseReset(std::string msg_in);
    bool parseReadTCPSpeed(std::string msg_in);

}; //class
}  //namespace

#endif //CRX10IAL_RMI_DRIVER_H
