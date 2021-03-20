# depthai-ros(Gen2)
noetic-devel branch also works on melodic(tested). Might also work on kinetic too.


## Getting Started
### Setting up procedure

1. `cd ~`
2. `git clone https://github.com/luxonis/depthai-core.git --branch develop`
3. `cd ~/depthai-core`
4. `mkdir build`
5. `cmake .. -D BUILD_SHARED_LIBS=ON`
6. `cmake --build . --parallel --config Release --target install`   
7. `cd ~`
8. `mkdir -p ros_ws/src`
9. `cd ros_ws/src`
10. `git clone https://github.com/luxonis/depthai-ros.git --branch noetic-devel`
11. `git clone https://github.com/luxonis/depthai-ros-examples.git --branch noetic-devel`
12. `git clone https://github.com/ros-perception/vision_msgs.git --branch noetic-devel`
13. `cd ~/ros_ws`
14. `source /opt/ros/<ros-distro>/setup.zsh`     
15. `catkin_make_isolated --cmake-args -D depthai_DIR=${depthai-core insall directory}/lib/cmake/depthai`
