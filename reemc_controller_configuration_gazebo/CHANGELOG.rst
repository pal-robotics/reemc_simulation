^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package reemc_controller_configuration_gazebo
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
