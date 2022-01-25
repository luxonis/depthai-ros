# depthai-ros(Gen2)
main branch supports ROS Melodic, ROS Noetic, ROS2 Foxy & Galactic. Might also work on kinetic too.


## Getting Started

### Install Dependencies
The following script will install depthai-core and update usb rules and install depthai devices

```
sudo wget -qO- https://raw.githubusercontent.com/luxonis/depthai-ros/main/install_dependencies.sh | sudo bash
```
if you don't have opencv installed then try `sudo apt install libopencv-dev`


if you don't have rosdep installed and not initialized please execute the following steps:
1. `sudo apt install python-rosdep2`(melodic) or `sudo apt install python3-rosdep`
2. `sudo rosdep init`
3. `rosdep update`

install the following vcstool
`sudo apt install python3-vcstool`
### Setting up procedure
The following setup procedure assumes you have cmake version >= 3.10.2 and OpenCV version >= 4.0.0

1. `mkdir -p <directory_for_workspaces>/src`
2. `cd <directory_for_workspaces>`
3. `wget https://raw.githubusercontent.com/luxonis/depthai-ros/main/underlay.repos`
4. `vcs import src < underlay.repos`
5. `rosdep install --from-paths src --ignore-src -r -y`
6. `source /opt/ros/<ros-distro>/setup.bash`
7. `catkin_make`
8. `source devel/setup.bash` 


<!-- 
7. `cd ~`
8. `git clone https://github.com/luxonis/depthai-core.git --branch develop`
9. `cd ~/depthai-core`
10. `mkdir build`
11. `cmake .. -D BUILD_SHARED_LIBS=ON`
12. `cmake --build . --parallel --config Release --target install`   
13. `cd ~`
14. `mkdir -p ros_ws/src`
15. `cd ros_ws/src`
16. `git clone https://github.com/luxonis/depthai-ros.git --branch noetic-devel`
17. `git clone https://github.com/luxonis/depthai-ros-examples.git --branch noetic-devel`
18. `git clone https://github.com/ros-perception/vision_msgs.git --branch noetic-devel`
19. `cd ~/ros_ws`
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
9. `mkdir -p ros_ws/src`
10. `cd ros_ws/src`
11. `git clone https://github.com/luxonis/depthai-ros.git --branch noetic-devel`
12. `git clone https://github.com/luxonis/depthai-ros-examples.git --branch noetic-devel`
13. `git clone https://github.com/ros-perception/vision_msgs.git --branch noetic-devel`
14. `cd ~/ros_ws`
15. `source /opt/ros/<ros-distro>/setup.bash` or `source /opt/ros/<ros-distro>/setup.zsh` if using zsh instead of bash
16. `catkin_make_isolated --cmake-args -Ddepthai_DIR=~/depthai-core/build/install/lib/cmake/depthai` (Melodic)
17. `catkin_make_isolated --cmake-args -D depthai_DIR=~/depthai-core/build/install/lib/cmake/depthai` (Noetic) -->

### Executing an example

1. `cd ~/ros_ws`
2. `source ~/ros_ws/devel/setup.bash`
3. `roslaunch depthai_examples stereo_node.launch` - example node


## Testing results
- ImageConverter - Tested using `roslaunch depthai_examples stereo_node.launch` && `roslaunch depthai_examples stereo_nodelet.launch` && `roslaunch depthai_examples rgb_publisher.launch`'
- ImgDetectionCnverter - tested using `roslaunch depthai_examples mobile_publisher.launch`
- SpatialImgDetectionConverter - Not tested yet. (Will add an example on this soon) 


### Users can write Custom converters and plug them in for bridge Publisher. 
If there a standard Message or usecase for which we have not provided a ros msg or
 converter feel free to create a issue or reach out to us on our discord community. We would be happy to add more. 
