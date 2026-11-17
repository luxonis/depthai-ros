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

### Experimental RVC2 stereo half turn

`stereo_half_turn_prototype` is a standalone experiment built only with
`BUILD_TESTING=ON` and `TEST_DEPTHAI_ROS_DRIVER=ON`. It does not change the ROS
driver or device calibration EEPROM. Its `fused-flip` mode combines a half turn
with StereoDepth's existing rectification mesh, swaps the logical stereo inputs
to retain positive disparity, and aligns depth to physical CAM_B. Sensor pixels
and stereo processing stay on-device; mesh generation happens once at startup.

Modes are `native` (SDK rectification), `current` (the driver's existing device
rotation helper on each published output), `mesh-native` (the custom rectification
reference), `fused-flip` (half turn within rectification), and `sensor-flip`
(exploratory flipped readout plus rectification). The readout variant permits an
unresolved native orientation only on the previously verified test device.

Measurements on RVC2 `1944301071834D1300`, 640×400 at requested 30 FPS, with
LR check, subpixel, FAST_ACCURACY and matching exposure/settings:

| Published streams | Current downstream rotation | Fused rectification |
| --- | ---: | ---: |
| Depth only | 30 FPS | 30 FPS |
| Depth and both rectified images | 11.5–11.9 FPS depth | 30 FPS |

The all-stream result repeated twice over 30-second measurement windows after
five seconds of warmup, and was confirmed with an explicit eight-frame source
ISP pool in a further 20-second comparison. It is a stereo-only benchmark, not the complete RGBD/NN
workload. The device was capped at 30 FPS, so depth-only headroom was not measured.

**This prototype is not ready for production.** After correcting an RVC2 mesh
sampling offset, rectified images match their expected half turns at >0.9999
correlation. Feature-triangulated depths agree well, but the dense depth gate
still fails: the 90th-percentile difference from native depth is about 23.5%,
versus 4.3% in a native repeat control. The root cause is unresolved; discrepancies
concentrate in weakly textured regions. ROS CameraInfo/TF, RGB alignment, RGBD and
spatial-NN metadata have not been integrated. Custom meshes define a different
rectification camera from the SDK's automatic maps, so geometry comparisons use
`mesh-native` as their reference. The emitted JSON records that reference camera;
it does not certify SDK frame metadata for downstream consumers.

To reproduce from the repository root after building and sourcing ROS Kilted
and this workspace, select the pinned SDK used to build the experiment:

```bash
export LD_LIBRARY_PATH="$PWD/install/sandbox/depthai-device-validation/lib:$LD_LIBRARY_PATH"
python3 depthai_ros_driver/test/benchmark_stereo_half_turn.py \
  --binary build/sandbox/depthai_ros_driver/stereo_half_turn_prototype \
  --device 1944301071834D1300 --seconds 30 \
  --output /tmp/stereo-half-turn-example
python3 depthai_ros_driver/test/analyze_stereo_half_turn.py \
  /tmp/stereo-half-turn-example/0-native /tmp/stereo-half-turn-example/1-current \
  /tmp/stereo-half-turn-example/2-mesh-native /tmp/stereo-half-turn-example/3-fused-flip \
  --output /tmp/stereo-half-turn-example/geometry.json
```

The analyzer intentionally returns nonzero when its geometry/depth agreement
criteria fail. Run a separate benchmark with `--streams depth` to measure the
single-output case. The runner requires a new output directory for each run and
retains raw depth/rectified PNG snapshots, frame counts, sequence gaps and results.
