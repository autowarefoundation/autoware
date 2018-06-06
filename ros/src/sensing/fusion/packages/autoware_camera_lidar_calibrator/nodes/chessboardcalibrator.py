#!/usr/bin/python
import rospy
from chessboard_camera_lidar_calibration.lidar_chessboard import LidarCameraCalibrator

def main():
    rospy.init_node('chessboardcalibrator')
    node = LidarCameraCalibrator()
    rospy.spin()


if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        import traceback
        traceback.print_exc()
