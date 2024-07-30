
# TODO

---

- [X] Get Gazebo spawn working (possibly comment the line which keeps opening/closing Gazebo for testing purposes)
    It seems like this is due to missing inertia tags.
- [X] Estimate link masses and inertias
- [X] Add a new URDF/xacro file covering joint control plugin
- [ ] Add joint stiffness (Uneccesary?)
- [X] Inertia? Needed really. Box/equations would be good, check collision meshes (Used [MeshLabInertiaToURDF](https://github.com/vonunwerth/MeshLabInertiaToURDF))
- [X] Build to Moveit control (See crx_description/urdf/crx.ros2_control.xacro, and [medium](https://kolkemboi.medium.com/simulate-6-dof-robot-arm-in-ros2-gazebo-and-moveit2-a171c7e9b0ad))
- [X] Integrate with ros2_fanuc_interface! (Robot_bringup.launch)
- [ ] Pass use_sim_time in robot_bringup.launch
- [ ] Add headless Gazebo mode (Invert argument)
- [ ] Allow alternate between Gazebo and Fake Controller
- [ ] Web bridge (Grasshopper)
- [ ] Then ros2_file_server

Ensure Moveit works again
THEN HEADLESS
Use sim time possible?
