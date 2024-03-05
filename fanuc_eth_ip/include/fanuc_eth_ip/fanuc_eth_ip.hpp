#include <EIPScanner/MessageRouter.h>
#include <EIPScanner/utils/Logger.h>
#include <EIPScanner/utils/Buffer.h>

using eipScanner::SessionInfo;
using eipScanner::MessageRouter;
using namespace eipScanner::cip;
using namespace eipScanner::utils;

class fanuc_eth_ip
{
private:
    std::string ip_;
    std::shared_ptr< eipScanner::SessionInfo > si_;
    std::shared_ptr< eipScanner::MessageRouter > messageRouter_ = std::make_shared< eipScanner::MessageRouter >();

public:
    fanuc_eth_ip(std::string ip);
    ~fanuc_eth_ip();
    std::vector<double> get_current_joint_pos();
    std::vector<double> get_current_pose();
    void write_register(const int val, const int reg = 1);
    void write_pos_register(const std::vector<double> j_vals, const int reg = 1);
    bool write_DI(const Buffer buffer);
    bool write_flag(const bool val, const int pos);
    void activateDPM(const bool activate = true);
    void deactivateDPM();
    bool writeDPM(const std::vector<int> vals);
    bool writeDPMScaled(const std::vector<double> vals, const double scale = 0.01);
    void setCurrentPos();

};
