# Depthai ROS Repository
Hi and welcome to the main depthai-ros respository! Here you can find ROS related code for OAK cameras from Luxonis. Don't have one? You can get them [here!](https://shop.luxonis.com/)

You can find the newest documentation [here](https://docs.luxonis.com/software-v3/depthai/ros/)

### Telemetry
We collect some anonymized telemetry to understand how users use the library so we can improve it. This mainly includes which nodes are used most often, how long sessions last, and similar usage information. If you prefer, you can disable telemetry by setting the environment variable `DEPTHAI_TELEMETRY=0`.


### Sensor readout for 180-degree output rotation

`driver.i_output_rotation_deg: 180` uses `Camera::setImageOrientation()` on RVC2 for
publication-only cameras in the `RGB`, `RGBStereo`, `Stereo` and `CamArray`
pipelines. This avoids device ImageManip rotation passes. StereoDepth inputs,
VIO inputs, RGBD/alignment cameras and cameras with NN or feature-tracker
consumers retain native sensor orientation and rotate downstream. Quarter turns
also continue to use device ImageManip nodes. RVC4 keeps that path because its
current firmware does not apply the Camera sensor-orientation property.

The half turn is composed with the board's native readout orientation. When the
SDK reports that orientation as `AUTO`, the driver keeps downstream rotation
unless the native orientation is explicitly supplied. For example, after checking
that the camera's normal `AUTO` image matches `NORMAL`:

```yaml
/**:
  ros__parameters:
    driver.i_output_rotation_deg: 180
    pipeline_gen.i_pipeline_type: RGBStereo
    rgb.i_native_sensor_orientation: NORMAL
    left.i_native_sensor_orientation: NORMAL
    right.i_native_sensor_orientation: NORMAL
```

`i_native_sensor_orientation` describes the existing unrotated sensor readout;
it does not independently change the image orientation. Supported values are
`AUTO`, `NORMAL`, `ROTATE_180_DEG`, `HORIZONTAL_MIRROR`, and `VERTICAL_FLIP`.
Images and CameraInfo retain the rotated optical frame convention.

To compare a stationary scene on hardware, run
`depthai_ros_driver/test/test_sensor_orientation.py` with
`python3 -m launch_testing.launch_test`, first with `DEPTHAI_ROTATION_TEST_ANGLE=0`
and then `180`. Set `DEPTHAI_ROTATION_TEST_DEVICE` and a shared
`DEPTHAI_SENSOR_ORIENTATION_REFERENCE` output directory. The test saves images,
calibration and rates, and compares the half-turn image against the original
`AUTO` capture. `DEPTHAI_SENSOR_NATIVE_ORIENTATION` selects the assumed baseline
for the second run (default `NORMAL`). Keep the camera and scene stationary.
