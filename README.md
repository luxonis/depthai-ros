# depthai-ros(Gen2)

## Getting Started

### Install Dependencies
The following script will install depthai-core and update usb rules and install depthai devices

```
sudo wget -qO- https://raw.githubusercontent.com/luxonis/depthai-ros/foxy-devel/install_dependencies.sh | sudo bash
```

if you don't haave opencv installed then try `sudo apt install libopencv-dev`

if you don't have rosdep installed and not initialized please execute the following steps:
1. `sudo apt install python3-rosdep`
2. `sudo rosdep init`
3. `rosdep update`

install the following vcstool
`sudo apt install python3-vcstool`
### Setting up procedure
The following setup procedure assumes you have cmake version >= 3.10.2 and OpenCV version >= 4.0.0

1. `mkdir -p <directory_for_workspaces>/src`
2. `cd <directory_for_workspaces>`
3. `wget https://raw.githubusercontent.com/luxonis/depthai-ros/foxy-devel/underlay.repos`
4. `vcs import src < underlay.repos`
5. `rosdep install -y -r -q --from-paths src --ignore-src --rosdistro <add your distro>`
6. `source /opt/ros/<ros-distro>/setup.bash`
7. `colcon build`
8. `source install/setup.bash` 

### Executing an example

1. `cd <directory_for_workspaces>`
2. `source install/setup.bash`
3. `ros2 run depthai_examples stereo_node` - example node


## Testing results (WIP)
<!-- - ImageConverter - Tested using `roslaunch depthai_examples stereo_node.launch` && `roslaunch depthai_examples stereo_nodelet.launch` && `roslaunch depthai_examples rgb_publisher.launch`'
- ImgDetectionCnverter - tested using `roslaunch depthai_examples mobile_publisher.launch`
- SpatialImgDetectionConverter - Not tested yet. (Will add an example on this soon)  -->


### Users can write Custom converters and plug them in for bridge Publisher. 
If there a standard Message or usecase for which we have not provided a ros msg or
 converter feel free to create a issue or reach out to us on our discord community. We would be happy to add more. 
