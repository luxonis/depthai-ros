# depthai-ros(Gen2)
noetic-devel branch also works on melodic(tested). Might also work on kinetic too.

## Dependencies and USB-rules setup
Install the dependencies described [here](https://docs.luxonis.com/projects/api/en/latest/install/#ubuntu)
followed by `pythn3 -m pip install opencv-python` (This should install C++ lib opencv version greater than 4.0.0 which is required here. 


## Getting Started
### Setting up procedure
The following setup procedure assumes you have cmake version >= 3.10.2 and OpenCV version >= 4.0.0

1. `cd ~`
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
17. `catkin_make_isolated --cmake-args -D depthai_DIR=~/depthai-core/build/install/lib/cmake/depthai` (Noetic)


### Executing an example

1. `cd ~/ros_ws`
2. `source ~/ros_ws/devel_isolated/setup.bash`
3. `roslaunch depthai_examples stereo_node.launch` - example node


## Testing results
- ImageConverter - Tested using `roslaunch depthai_examples stereo_node.launch` && `roslaunch depthai_examples stereo_nodelet.launch` && `roslaunch depthai_examples rgb_publisher.launch`'
- ImgDetectionCnverter - tested using `roslaunch depthai_examples mobile_publisher.launch`
- SpatialImgDetectionConverter - Not tested yet. (Will add an example on this soon) 


### Users can write Custom converters and plug them in for bridge Publisher. 
If there a standard Message or usecase for which we have not provided a ros msg or
 converter feel free to create a issue or reach out to us on our discord community. We would be happy to add more. 
