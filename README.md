# depthai-ros
main branch supports ROS Melodic, ROS Noetic, ROS2 Foxy & Galactic. Might also work on kinetic too.



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


<!-- 
7. `cd ~`
8. `git clone https://github.com/luxonis/depthai-core.git --branch develop`
9. `cd ~/depthai-core`
10. `mkdir build`
11. `cmake .. -D BUILD_SHARED_LIBS=ON`
12. `cmake --build . --parallel --config Release --target install`   
13. `cd ~`
14. `mkdir -p dai_ws/src`
15. `cd dai_ws/src`
16. `git clone https://github.com/luxonis/depthai-ros.git --branch noetic-devel`
17. `git clone https://github.com/luxonis/depthai-ros-examples.git --branch noetic-devel`
18. `git clone https://github.com/ros-perception/vision_msgs.git --branch noetic-devel`
19. `cd ~/dai_ws`
20. `source /opt/ros/<ros-distro>/setup.zsh`     
21. `catkin_make_isolated --cmake-args -D depthai_DIR=${depthai-core insall directory}/lib/cmake/depthai` -->

<!-- 1. `cd ~`
2. `git clone --recursive https://github.com/luxonis/depthai-core.git --branch develop`
3. `cd ~/depthai-core`
4. `mkdir build`
5. `cd build`
6. `cmake .. -DBUILD_SHARED_LIBS=ON`
7. `cmake --build . --config Release --target install`   
8. `cd ~`
9. `mkdir -p dai_ws/src`
10. `cd dai_ws/src`
11. `git clone https://github.com/luxonis/depthai-ros.git --branch noetic-devel`
12. `git clone https://github.com/luxonis/depthai-ros-examples.git --branch noetic-devel`
13. `git clone https://github.com/ros-perception/vision_msgs.git --branch noetic-devel`
14. `cd ~/dai_ws`
15. `source /opt/ros/<ros-distro>/setup.bash` or `source /opt/ros/<ros-distro>/setup.zsh` if using zsh instead of bash
16. `catkin_make_isolated --cmake-args -Ddepthai_DIR=~/depthai-core/build/install/lib/cmake/depthai` (Melodic)
17. `catkin_make_isolated --cmake-args -D depthai_DIR=~/depthai-core/build/install/lib/cmake/depthai` (Noetic) -->

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
