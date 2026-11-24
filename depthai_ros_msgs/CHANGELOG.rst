^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package depthai_ros_msgs
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

3.3.0 (2026-06-10)
------------------
* Require DepthAI 3.7.1 packages.
  * Adds telemetry to help us understand how the library is used so we can improve it
  * To opt out set `DEPTHAI_TELEMETRY=0` in the environment

3.2.1 (2026-05-15)
------------------
* Add support for RVC4 IMU magnetometer and rotation vector data.

3.2.0 (2026-05-06)
------------------
* Expose pipeline auto calibration mode through the ROS driver.

3.1.2 (2026-05-05)
------------------
* Require DepthAI 3.6.1 packages.
* Update release preparation defaults for DepthAI 3.6.1.

2.5.3 (2022-08-21)
-----------
* Updated release version
* moved the depthai bridge position of find package (`#115 <https://github.com/luxonis/depthai-ros/issues/115>`_)
* Contributors: Sachin, Sachin Guruswamy

2.5.2 (2022-06-01)
-------------------
* Upgraded examples
* Fixed bugs for Noetic

2.5.1 (2022-05-20)
-------------------
* Fix Build farm issues

2.5.0 (2022-05-20)
-------------------
* Release 2.5.0
* add ament package:
* created Bridge and Coverters to handle images, IMU and camera Info
