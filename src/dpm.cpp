#include <ros2_fanuc_interface/fanuc_eth_ip.h>

#include <unistd.h>
 

int main() {
    
    // creates the driver object
    fanuc_eth_ip driver("10.11.31.111");

    // reads joint position
    std::vector<double> j_pos = driver.get_current_joint_pos();

    for(auto j:j_pos)
        Logger(LogLevel::ERROR) << "jpos: " << j;


    // writes on a register
    driver.write_register(2,1);

    // writes on a position register
    driver.write_pos_register(j_pos);

    driver.activateDPM();


    {
        std::vector<int> v = {1, 0, 0, 0, 0, 0};
        driver.writeDPM(v);
    }

    {
        int delta_x = 1;
        double cycle_time = 0.008;
        std::vector<int> v = {delta_x, 0, 0, 0, 0, 0};
        for (int i = 0; i< 200/delta_x; i++)
        {
            driver.writeDPM(v);
            sleep(cycle_time);
        }
    }

    driver.deactivateDPM();

    return 0;
}
