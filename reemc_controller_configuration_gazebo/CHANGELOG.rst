^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package reemc_controller_configuration_gazebo
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.10.5 (2016-12-13)
-------------------
* Fixed simulation walking params
* Contributors: Hilario Tome

0.10.4 (2016-10-13)
-------------------
* Updated reemc hardware gazebo config
* Contributors: Hilario Tome

0.10.3 (2016-10-06)
-------------------
* Added reem-c specifics to walking params
* Moving the config file for pal_hardware_gazebo to reemc_controller_configuration_gazebo as we are deprecating reemc_hardware_gazebo
* Contributors: Hilario Tome, Sam Pfeiffer

0.10.2 (2016-04-14)
-------------------

0.10.1 (2016-04-14)
-------------------

0.10.0 (2016-04-04)
-------------------

0.9.5 (2016-03-04)
------------------

0.9.4 (2015-06-15)
------------------

0.9.3 (2015-06-13)
------------------
* Update jtc simulation tolerances
* Contributors: Adolfo Rodriguez Tsouroukdissian

0.9.2 (2015-06-05)
------------------
* Make sim bringup fully aware of REEM-C variants
  - Separate ROS param configuration of hand controllers from the main
  joint_trajectory_controller.yaml file. Correct hand controller configuration
  is loaded based on the robot launch argument.
  - Make reemc_empty_world.launch aware of the 'robot' argument.
* Add pids of 3finger hand too
* Fix wrist ft sensor read in hardware_gazebo.
  Set new frame_id for wrist ft measures.
  Default robot launched in gazebo set to full_ft_hey5.
  Modify names of sensors used by walking to new ankle ft names.
  Add pid values for hey5 and remove the pids value of 3 finger hand.
* Contributors: Adolfo Rodriguez Tsouroukdissian, Bence Magyar, Luca Marchionni

0.9.1 (2015-04-08)
------------------
* adds enabled param to odometry and moves odometry related params to 'odometry' ns
* removes trailing spaces
* merged hand description from rockin branch and fixed pids for underactuated joints.
  Increased torso max torque in urdf for simulating sitting.
* refs #7537 : adds covariance params (for gazebo)
* refs #7537 : adds use_imu_yaw and odom_pub_rate params (for gazebo)
* reemc_controller_configuration_gazebo: walking -> walking_controller
* Catkinize reemc_controller_configuration_gazebo
* Merge from OROCOS_2.X
* Update manifests with maintainer information
* Merged from OROCOS_2.X
* walking: fix dependencies
* Remove reemc_ prefix from reemc_default_controllers.launch
  This mimics the non-Gazebo launch files, which follow this convention.
* added config and launch for reemc sqaut controller
* modified gains for arms and torso for better tracking walking trajectories for upper body
* added yaml for additional joints controlled by walking when moving upper body.
  Additional parameter for activating or deactivating upper body movements added to the launch files.
* adding simulation configs for walking component for full robot (arms and torso) or just lower body.
* changed namespace for parameters related to biped_controller
* changed parameters names and namespace for simulated robot
* fixed typo in gazebo parameter
* changed launch file for simulated reemc to load parameters in walking_controller namespace
* adding parameters for walking in a separated yaml file
* adding params for ft sensor height in simulation
* Walking refactored with dynamic_reconfigure parameters.
  Added launch files for walking with different parameters on real and simulated robot.
* merging from trunk.
* adding fake pid value for robot soles
* Add dummy laser joint PIDs. Refs #6173.
* removed default controller spawner duplicated.
* Merge from trunk to OROCOS_2.X branch
* Initial migration of REEM-C simulation model to ros_control. Refs #5961.
* commit after:
  i) undoing changes in stacks before 41800
  svn merge -r HEAD:41800 svn+ssh://carles@server/srv/svn/repos/branches/OROCOS_2.X/pal-ros-pkg/stacks  .
  ii) updating stacks to trunk
  svn merge svn+ssh://carles@server/srv/svn/repos/trunk/pal-ros-pkg/stacks .
* Merge from trunk to Orocos_2.X branch
* Commented stuff depending on gazebo and drcsim packages.
* added cfg and launch to use drcsim controllers
* Updated launches and config files
* added launch, node and config file to control reem-c simulated robot with joint_state_publisher GUI and joint action controller
* adding stacks for reemc simulation
* Contributors: Adolfo Rodriguez Tsouroukdissian, Carlez Lopez, Enrique Fernandez, Hilario Tome, Jordan Palacios, Luca Marchionni, Paul Mathieu, Victor Lopez
