^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package reemc_hardware_gazebo
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.10.5 (2016-12-13)
-------------------

0.10.4 (2016-10-13)
-------------------
* Removed specific reemc hardware gazebo for generic pal hardware gazebo
* Contributors: Hilario Tome

0.10.3 (2016-10-06)
-------------------

0.10.2 (2016-04-14)
-------------------
* Fixed missing install rule for config folder of reemc_hardware_gazebo
* Contributors: Hilario Tome

0.10.1 (2016-04-14)
-------------------
* Added pal hardware gazebo to REEM-C simulation
* Contributors: Hilario Tome

0.10.0 (2016-04-04)
-------------------
* Dubnium compilation fixes
* Contributors: Hilario Tome

0.9.5 (2016-03-04)
------------------
* Fix wrist ft tranformations and ankle ft tf frames
* Contributors: Luca Marchionni

0.9.4 (2015-06-15)
------------------

0.9.3 (2015-06-13)
------------------

0.9.2 (2015-06-05)
------------------
* Fix wrist ft sensor read in hardware_gazebo.
  Set new frame_id for wrist ft measures.
  Default robot launched in gazebo set to full_ft_hey5.
  Modify names of sensors used by walking to new ankle ft names.
  Add pid values for hey5 and remove the pids value of 3 finger hand.
* Add wrist F/T sensors to simulated robot.
* Contributors: Adolfo Rodriguez Tsouroukdissian, Luca Marchionni

0.9.1 (2015-04-08)
------------------
* Fixing gazebo headers for release
* Reverting joint mode experimental stuff
* Add mode manager class to factor some code out
  Now parsing URDF for available joint interfaces
  Refs #9845
* Fix behaviour of switching mechanism.
  Refs #9845
* Adding first prototype of joint mode implementation
  Refs #9845
* Implemented ros_control inferface for actuators max limit current.
  Added spcefic code for reemc_hardware (real robot) and reemc_hardware_gazebo (simulation).
* reemc_hardware_gazebo: forgot package.xml
* Catkinize reemc_hardware_gazebo
* reemc_hardware_gazebo: fix robot namespacing
* Update manifests with maintainer information
* reemc_hardware_gazebo: removed unneeded dependency
  gazebo_ros_control is (should be) exporting it
* reemc_hardware_gazebo: cut dependency on overlay
* Merge from OROCOS_2.X
* reemc_simulation: fix dependencies
* Adjusted signs of ft sensor in gazebo to be the same as on Reem-B.
* changed sign of torque in y derection
* fixed force torque sensor for gazebo simulation
* Add joint limits enforcing. Remove Eigen dependency.
  - Test with newer inertia and controller gains. Needs better tracking to work well.
* Propagate changes in ros_control. Add force-torque sensors+IMU.
* Initial migration of REEM-C simulation model to ros_control. Refs #5961.
* Contributors: Adolfo Rodriguez Tsouroukdissian, Bence Magyar, Luca Marchionni, Paul Mathieu, Sammy Pfeiffer
