#!/usr/bin/env python3

from depthai_sdk_ros.ros_integration.ros_camera import ROSCamera

def main():
    try:
        with ROSCamera() as oak:
            color = oak.create_camera('color', resolution='1080p', encode=False, fps=30)
            # color.config_color_camera(isp_scale=(2,3))
            left = oak.create_camera('left', resolution='400p', encode=False,fps=30)
            right = oak.create_camera('right', resolution='400p', encode=False,fps=30)
            stereo = oak.create_stereo('1080p', 30.0, left, right, 'stereo', encode=False)
            imu = oak.create_imu()
            imu.config_imu(report_rate=400, batch_report_threshold=5)

            oak.ros_stream([ color, imu, stereo])
            # oak.visualize(left)
            oak.start(blocking=True)
    except Exception as e:
        print(e) 
    print("aaa")
if __name__ == '__main__':
    main()
