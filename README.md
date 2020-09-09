# DepthAI ROS2 Wrapper

This is an attempt at basic DepthAI to ROS2 interface. It's largely leveraging the existing depthai python demo on https://github.com/luxonis/depthai. 

Also note that this example depends on changes on the develop_merge_config_manager branch of https://github.com/luxonis/depthai.

## Setup Instructions
1. Clone our DepthAI github.

`git clone git@github.com:luxonis/depthai.git`

2. Switch to the develop_merge_config_manager branch and install DepthAI dependencies. 
```
cd depthai
git checkout develop_merge_config_manager
python3 -m pip install -r requirements.txt
```

3. Add the depthai directory to PYTHONPATH. You can also add it to your .bashrc file to skip this each time you open a new shell.
```
export PYTHONPATH=$PYTHONPATH:<depthai-dir>
```

4. Setup ros for this shell.
```
source /opt/ros/eloquent/setup.bash
cd <your-ros-workspace>
```

5. Pull down the DepthAI ros2 package from github.
```
cd <your-ros-workspace>/src/
git clone git@github.com:luxonis/depthai_ros2.git
```

6. Build the ros2 package
```
cd <your-ros-workspace>
colcon build --packages-select depthai_wrapper
. install/setup.bash
```

7. Get a list of attached devices. You’ll want to pay attention to the “on USB port: <devId>” part. We’ll use that to select a DepthAI device. You can skip this if you have a single DepthAI board. Just don't pass the -dev cli arg in the subseqent commands.
```
ros2 run depthai_wrapper talker --ros-args -p cliArgs:="-dev list"
```

8. Run the depthai_wrapper talker. This will create topics you can subscribe to and get depthai results. Not all topics are complete but I think you probably only care about the meta topic and maybe the preview. Both of those should work. Note that you can use the stock mobilenet_ssd if you leave off “-cnn kartDetection”.
```
ros2 run depthai_wrapper talker --ros-args -p cliArgs:="-dev <devId>"
```

9. Run the demo listener if you want an example of how to receive our topics.
```
ros2 run depthai_wrapper demoListen --ros-args -p cliArgs:="-dev <devId>"
```
