---
name: Bug report
about: Create a bug report to help us improve the DepthAI
title: "[BUG] {Title of the bug}"
labels: bug
assignees: ''

---

**Check if issue already exists**

- Google it (e.g. *error xy* github luxonis depthai)
- Check [troubleshooting](https://docs.luxonis.com/en/latest/pages/troubleshooting/) in documentation.

**Describe the bug**
A clear and concise description of what the bug is.

**Minimal Reproducible Example**
Append the MRE to the bug report, [instructions here](https://docs.luxonis.com/en/latest/pages/support/#creating-minimal-reproducible-example)

If available launch files don't work in your case, please check if you also get errors while running:
- `stereo_inertial_node` in `depthai_examples`
- `camera` in `depthai_ros_driver`
- In case both fail to run, please check if you can run [the default python demo app](https://docs.luxonis.com/en/latest/#demo-script)

**Expected behavior**
A clear and concise description of what you expected to happen.

**Screenshots**
If applicable, add screenshots to help explain your problem.

**Pipeline Graph**

Please also provide a screenshot of your pipeline using the [DepthAI Pipeline Graph](https://github.com/geaxgx/depthai_pipeline_graph).

You can save it in `depthai_ros_driver`, either by calling `/save_pipeline` ROS service, or by setting parameter `camera.i_pipeline_dump` in ROS 2 or `camera_i_pipeline_dump` in ROS. Pipeline dump is saved to `/tmp/pipeline.json`.

**Attach system log**
 - Provide output of [log_system_information.py](https://github.com/luxonis/depthai/blob/main/log_system_information.py)
 - Which OS/OS version are you using?
 - Which ROS version are you using?
 - Which ROS distribution are you using ?
 - Is `depthai-ros` built from source or installed from apt?
 - Is `depthai/depthai-core` library installed from rosdep or manually? For rosdep install, check if `ros-<rosdistro>-depthai` package is installed, manual install can be checked with `ldconfig -p | grep depthai`
 - Please include versions of following packages - `apt show ros-$ROS_DISTRO-depthai ros-$ROS_DISTRO-depthai-ros ros-$ROS_DISTRO-depthai-bridge ros-$ROS_DISTRO-depthai-ros-msgs ros-$ROS_DISTRO-depthai-ros-driver`
 - To get additional logs, set `DEPTHAI_DEBUG=1` and paste the logs, either from command line or from latest log in `~/.ros/log`

**Additional context**
Add any other context about the problem here.
