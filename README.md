# DepthAI ROS1 

This repo contains ROS1 based messages and simple examples on creating ros1 nodes using DepthAI-gen2 - https://github.com/luxonis/depthai. 

Please note that this version is still under construction and prone to issues. Feel free to create an issue on GitHub or ping us on our discord channel.
## Setup Instructions
1. Clone this repository into ros workspace along with it's submodules.

2. Do not use the vision_msgs from the installed packages. Clone the new version from [here](https://github.com/ros-perception/vision_msgs) which has been recently updated. 

3. Build the packages in this repository using `catkin_make_isolated`. P.S: Do not use `catkin_make` on these packages it will throw an error. 


## Available ROS node Examples
- Stereo Node (Do not use any extended features like subpixel/lrchecks they are prone to error in convertion which needs to be fixed on the device side)[]
- RGB node (Publishes 1080p rgb stream)
- Mobilenet publisher
Take a look at the launch files of the examples [here](./depthai_examples/launch)