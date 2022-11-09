# Depthai ROS Repository
Hi and welcome to the main depthai-ros respository!

Supported ROS versions:
- Noetic
- Galactic
- Humble

For development check out respective git branches.

### Install from ros binaries

Add USB rules to your system
```
echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="03e7", MODE="0666"' | sudo tee /etc/udev/rules.d/80-movidius.rules
sudo udevadm control --reload-rules && sudo udevadm trigger
```
Install depthai-ros. (Available for Noetic, foxy, galactic and humble)
`sudo apt install ros-<distro>-depthai-ros`

## Install from source

### Install dependencies

The following script will install depthai-core and update usb rules and install depthai devices

```
sudo wget -qO- https://raw.githubusercontent.com/luxonis/depthai-ros/main/install_dependencies.sh | sudo bash
```
if you don't have opencv installed then try `sudo apt install libopencv-dev`


if you don't have rosdep installed and not initialized please execute the following steps:
1. `sudo apt install python-rosdep`(melodic) or `sudo apt install python3-rosdep`
2. `sudo rosdep init`
3. `rosdep update`

### Setting up procedure
The following setup procedure assumes you have cmake version >= 3.10.2 and OpenCV version >= 4.0.0. We selected `dai_ws` as the name for a new folder, as it will be our depthai ros workspace.

1. `mkdir -p dai_ws/src`
2. `cd dai_ws/src`
3. `git clone https://github.com/luxonis/depthai-ros.git`
4. `cd ../..`
5. `rosdep install --from-paths src --ignore-src -r -y`
6. `source /opt/ros/<ros-distro>/setup.bash`
7. `catkin_make` (For ROS1) `colcon build` (for ROS2)
8. `source devel/setup.bash` (For ROS1) & `source install/setup.bash` (for ROS2) 

## Executing an example

### ROS1
1. `cd dai_ws` (Our workspace)
2. `source devel/setup.bash`
3. `roslaunch depthai_examples stereo_inertial_node.launch` - example node
For more examples please check the launch files.

### ROS2
1. `cd dai_ws` (Our workspace)
2. `source install/setup.bash`
3. `ros2 launch depthai_examples stereo_inertial_node.launch.py` - example node
For more examples please check the launch files.



## Running Examples

### Mobilenet Publisher:
#### ROS1:
##### OAK-D
```
roslaunch depthai_examples mobile_publisher.launch camera_model:=OAK-D
```
##### OAK-D-LITE
```
roslaunch depthai_examples mobile_publisher.launch camera_model:=OAK-D-LITE
```
##### With visualizer
```
roslaunch depthai_examples mobile_publisher.launch | rqt_image_view -t /mobilenet_publisher/color/image
```

#### ROS2:

##### OAK-D
```
ros2 launch depthai_examples mobile_publisher.launch.py camera_model:=OAK-D
```

##### OAK-D-LITE
```
ros2 launch depthai_examples mobile_publisher.launch.py camera_model:=OAK-D-LITE
```


### Testing results
- ImageConverter - Tested using `roslaunch depthai_examples stereo_inertial_node.launch` && `roslaunch depthai_examples rgb_publisher.launch`'
- ImgDetectionCnverter - tested using `roslaunch depthai_examples mobile_publisher.launch`
- SpatialImgDetectionConverter - Ntested using `roslaunch depthai_examples stereo_inertial_node.launch`


### Users can write Custom converters and plug them in for bridge Publisher. 
If there a standard Message or usecase for which we have not provided a ros msg or
 converter feel free to create a issue or reach out to us on our discord community. We would be happy to add more. 
