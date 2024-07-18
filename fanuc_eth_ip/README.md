# fanuc_eth_ip

This package implements Ethernet/IP communication from remote PC to the Fanuc Controller. 

### Prerequisites - EIPScanner
The communication between the external pc and the Fanuc controller happens by means of the [Ethernet/IP](https://en.wikipedia.org/wiki/EtherNet/IP) protocol. 
In particular, this package relies on the implementation provided in [EIPScanner](https://eipscanner.readthedocs.io/en/latest/).  
Please, follow the instructoins [here](https://eipscanner.readthedocs.io/en/latest/getting_started.html#installing) to install it.

NOTE: a previous version of this driver used a python driver for the Ethernet/IP communication. Please refer to the [python-driver](https://github.com/paolofrance/ros2_fanuc_interface/tree/python-driver) branch.

### Implemented Functions



| Function | Description |
| ------------- | ------------- |
| `std::vector<double> get_current_joint_pos()` | reads the current joint configuration from the real robot |
| `std::vector<double> get_current_pose()` |reads the current Cartesian pose from the real robot |
| `int read_register(const int& reg)` | reads the vlaue written in the register reg |
| `void write_register(const int val, const int reg)` | writes the value val into register reg |
| `void write_pos_register(std::vector<double> j_vals, const int reg)` | writes joint values j_vals into position register reg |
| `void setCurrentPos()` | writes joint values read from `get_current_joint_pos()` into position register 1| 
| `bool write_DI(const Buffer buffer)` | writes digital inputs |
| `void activateDPM(const bool activate)` | opens DPM communication |
| `void deactivateDPM()` | closes DPM communication |
| `bool writeDPM(const std::vector<int> vals)` | writes values to modify the path via DPM |

to be tested: 

| Function | Description |
| ------------- | ------------- |
| `bool write_flag(const bool val, const int pos)` | should write flags |
| `writeDPMScaled(const std::vector<double> vals, const double scale)` | should scale values for the DPM according to the parameter scale |






## Contacts
Ethernet/IP driver: paolo.franceschi@supsi.ch  

### Acknowledgements
This package is developed by [Paolo Franceschi](https://paolofrance.github.io), with the [ARM (Automation Robotics and Machines Laboratory)](https://sites.supsi.ch/isteps_en/Laboratories/gruppo1.html) at SUPSI, Lugano, CH. 
The EU project [Fluently](https://www.fluently-horizonproject.eu/) partially funded the development of this package.

