^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package depthai-ros
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.10.3 (2024-10-14)
-------------------
* Allow setting USB speed without specifying device information

2.10.2 (2024-09-26)
-------------------
* Fix Stereo K matrix publishing
* Fix socket ID for NN detections
* Remove catching errors when starting the device since it introduced undefined behavior
* Add desqueeze to NN node

2.10.1 (2024-09-18)
-------------------
* Fix ToF synced publishing
* Add camera_info publishing when publishing compressed images
* Catch errors when starting the device

2.10.0 (2024-08-29)
-------------------
* Adding stl files for SR and LR models by @danilo-pejovic in https://github.com/luxonis/depthai-ros/pull/491
* No imu fix Humble by @Serafadam in https://github.com/luxonis/depthai-ros/pull/500
* Tracking converter for ROS2 Humble by @daniqsilva25 in https://github.com/luxonis/depthai-ros/pull/505
* Added Env Hooks so that depthai xacro can be used with gazebo sim by @r4hul77 in https://github.com/luxonis/depthai-ros/pull/507
* Fix resource paths for Ignition Gazebo by @Nibanovic in https://github.com/luxonis/depthai-ros/pull/511
* Use simulation flag to decide how to load meshes. by @destogl in https://github.com/luxonis/depthai-ros/pull/524
* Add new launch file for starting multiple rgbd cameras on robots. by @destogl in https://github.com/luxonis/depthai-ros/pull/532
* Missing fields in detection messages Humble by @Serafadam in https://github.com/luxonis/depthai-ros/pull/574
* Ip autodiscovery fix Humble by @Serafadam in https://github.com/luxonis/depthai-ros/pull/561
* RS Mode & Sync - Humble by @Serafadam in https://github.com/luxonis/depthai-ros/pull/578
* Compressed image publishers by @Serafadam in https://github.com/luxonis/depthai-ros/pull/580
* ToF Support Humble by @Serafadam in https://github.com/luxonis/depthai-ros/pull/581
* WLS fix humble by @Serafadam in https://github.com/luxonis/depthai-ros/pull/582
* Syncing & RS updates Humble by @Serafadam in https://github.com/luxonis/depthai-ros/pull/586

2.9.0 (2024-01-24)
-------------------

* New documentation homepage
* Updated support for LR and SR cameras
* Added parameter to toggle restart on logging error
* Changed argument for camera.launch file from `pass_tf_args_as_params` to `publish_tf_from_calibration` to be more explicit
* Added the option to run NN as part of sensor node
* Added option to run Spatial NN as part of stereo node

2.8.2 (2023-10-17)
-------------------

* Fixed default resolution for Stereo cameras
* Added CameraInfo update based on alpha scaling
* Logger restart bugfix
* URDF parameters fix

2.8.1 (2023-09-12)
-------------------

* Added support for OpenCV Stereo order convention
* Added disparity to depth use spec translation parameter
* Updated sensor socket logic
* Fixed issues when running robot_state_publisher as component
* Added missing tf2 dependencies

2.8.0 (2023-09-01)
-------------------
* Add camera image orientation param 
* Performance update
* Feature tracker
* Handle USB speed when usb id is specified
* Change misleading error to a clearer message
* Watchdog 
* Depth alignment update 
* Synced stereo streams
* Lazy Publishing 
* Urdf loader
* Add exposure offset

2.7.5 (2023-08-07)
-------------------
* IMU sync fix

2.7.4 (2023-06-26)
-------------------
* ROS time update
* Minor bugfixes

2.7.3 (2023-06-16)
-------------------
* Pipeline generation as a plugin
* Fixed bounding box generation issue
* Stereo rectified streams publishing
* Camera trigger mechanisms
* Brightness filter

2.7.2 (2023-05-08)
-------------------
* IMU improvements

2.7.1 (2023-03-29)
-------------------
* Add custom output size option for streams

2.7.0 (2023-03-28)
-------------------
* Added depthai_descriptions package
* Added depthai_filters package
* XLinkIn option for image subscription
* Additional debugging options
* Bugfixes

2.6.4 (2023-02-23)
-------------------
* Fix sensor name detection
* Enable subpixel mode
* Update camera start/stop services

2.6.3 (2023-02-10)
-------------------
* Camera calibration updates
* Option to connect to the device via USB port id

2.6.2 (2023-02-01)
-------------------
* Fixed timestamp in SpatialDetector
* Updated topic names in stereo_inertial_node

2.6.1 (2023-01-11)
-------------------
* Update docker image building

2.6.0 (2023-01-11)
-------------------
* Added depthai_ros_driver package

2.5.3 (2022-08-21)
-------------------
* Updated release version
* Contributors: Sachin

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


