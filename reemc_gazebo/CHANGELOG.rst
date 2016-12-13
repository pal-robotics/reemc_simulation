^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package reemc_gazebo
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.10.5 (2016-12-13)
-------------------

0.10.4 (2016-10-13)
-------------------

0.10.3 (2016-10-06)
-------------------
* Moving the config file for pal_hardware_gazebo to reemc_controller_configuration_gazebo as we are deprecating reemc_hardware_gazebo
* Contributors: Sam Pfeiffer

0.10.2 (2016-04-14)
-------------------

0.10.1 (2016-04-14)
-------------------
* Added pal hardware gazebo to REEM-C simulation
* Contributors: Hilario Tome

0.10.0 (2016-04-04)
-------------------

0.9.5 (2016-03-04)
------------------

0.9.4 (2015-06-15)
------------------
* Add ground plane and sun SDF models
* Fix REEM-C look to point world
* Contributors: Adolfo Rodriguez Tsouroukdissian, Jordi Pages

0.9.3 (2015-06-13)
------------------

0.9.2 (2015-06-05)
------------------
* Roslaunch 'robot' arg: default to 'full_ft_hey5'
  Previous default was 'full'.
* Make sim bringup fully aware of REEM-C variants
  - Separate ROS param configuration of hand controllers from the main
  joint_trajectory_controller.yaml file. Correct hand controller configuration
  is loaded based on the robot launch argument.
  - Make reemc_empty_world.launch aware of the 'robot' argument.
* Pass robot name param
* Fix wrist ft sensor read in hardware_gazebo.
  Set new frame_id for wrist ft measures.
  Default robot launched in gazebo set to full_ft_hey5.
  Modify names of sensors used by walking to new ankle ft names.
  Add pid values for hey5 and remove the pids value of 3 finger hand.
* Contributors: Adolfo Rodriguez Tsouroukdissian, Bence Magyar, Luca Marchionni

0.9.1 (2015-04-08)
------------------
* Add pal_gazebo_plugins dependency
* refs #8519 : syncs with 4.1_REEMC_SDE4 branch
  svn merge svn+ssh://server/srv/svn/repos/branches/4.1_REEMC_SDE4/pal-ros-pkg/catkin_pkgs/reemc_simulation/reemc_gazebo -c 53361
* refs #8519 : creates bauman world
* Updated the xacro generated reemc model urdf
* refs #7535 : moves reemc_bringup to reemc_bringup pkg
* Adding generated urdf
* add required elements to simulate pal textured object detection in reemc
* refs #7640 : copies simple_office world from reem_gazebo/worlds
* reemc_simulation: partial merge from OROCOS_2.X
* Catkinize reemc_gazebo
* reemc_gazebo: remove unneeded models
* reemc_gazebo: spawn model from robot_description instead of sdf
* Update manifests with maintainer information
* reemc_simulation: fix launch files (again and again)
* Merged from OROCOS_2.X
* added gzname just to be able to use launch files from reemc_gazebo_2dnav
* Merge from OROCOS_2.X
* reemc_gazebo: add GAZEBO_MODEL_PATH to launch file
  this way, no previous environment is needed
* reemc_gazebo: use gazebo_ros launch files instad of pal_gazebo_pkg
* Merge from OROCOS_2.X
* Merge from OROCOS_2.X
* reemc_gazebo: add dependency to play_motion
* reemc_simulation: fix dependencies
* Remove reemc_ prefix from reemc_default_controllers.launch
  This mimics the non-Gazebo launch files, which follow this convention.
* reemc_gazebo: added back reemc_empty_world.launch
* Moved config files to bringup and eliminated duplicated launch file.
  Updated reemc_gazebo.launch to have everything necessary for sitting.
  Refs #6437
* Added ikea urban chair.
  Refs #6437
* Added chairs world. To launch it do
  roslaunch reemc_gazebo reemc_gazebo.launch world:=chairs
  Refs #6437
* Added ikea ektorp armchair and goetz sofa.
  Refs #6437
* Added ikea folke chair to gazebo
  Refs #6437
* Added ikea harry chair to gazebo
  Refs #6437
* Added IKEA Stefan chair to gazebo.
  Refs #6437
* reemc_gazebo: fix default robot position to be (0,0)
* Adding parameter to reemc_bringup.launch for using upper body when walking (available only when full robot is simulated)
* reemc_gazebo: fixed launch files (play_motion and joy_teleop)
* reemc_gazebo: add small_squat to available joystick motions
* reemc_gazebo: update with play_motion by default
* enable gzpose + set vcg to show feet target marker refs #6192
* reemc_gazebo: use the new joy_teleop node by default
* removed params from bringup, that are loaded from different launch file
* reemc_gazebo: more cleanup
  don't worry, the files are just moved to reemc_gazebo_2dnav
* Cleanup in reemc_gazebo launch files
* small office world with door obstacle in the door refs #6192; use with:
  roslaunch reemc_gazebo reemc_navigation.launch world:=small_office_door_obstacle
  the robot falls when step on the door obstacle
* door_obstacle sdf model
* Cleaning in remmc_gazebo launchfiles
* fake localization (for footstep planner debugging), connected to initialpose corrector refs #6192
* Walking refactored with dynamic_reconfigure parameters.
  Added launch files for walking with different parameters on real and simulated robot.
* Reemc_gazebo launch files (big) cleanup
  By the way, walking is broken.
  You can have a look at the launch/reemc_bug.launch, and
  launch a working simulation with roslaunch reemc_gazebo
  reemc_gazebo.launch world:=empty robot:=bug
  Hopefully that will be fixed soon
  @hilariotome
  @luca
* Namespace cleanup in walking controller
* Change default REEM-C map in simulation
* Use pal_local_planner with REEM-C
  Also change default map (will be back in next commit to avoid a svn bug)
* merge from trunk to OROCOS_2.X branch
* merging from trunk.
* added ros_control stuff for lower_body simulation
* adding empty world for not having the model included into world file.
* Update bringup script.
* Convert to current SDF version. Fix walking merge bug.
* Merge from trunk to OROCOS_2.X branch
* Initial migration of REEM-C simulation model to ros_control. Refs #5961.
* commit after:
  i) undoing changes in stacks before 41800
  svn merge -r HEAD:41800 svn+ssh://carles@server/srv/svn/repos/branches/OROCOS_2.X/pal-ros-pkg/stacks  .
  ii) updating stacks to trunk
  svn merge svn+ssh://carles@server/srv/svn/repos/trunk/pal-ros-pkg/stacks .
* Contributors: Adolfo Rodriguez Tsouroukdissian, Bence Magyar, Carlez Lopez, Enrique Fernandez, Hilario Tome, Jordi Pages, Luca Marchionni, Paul Mathieu, Sam Pfeiffer, Victor Lopez
