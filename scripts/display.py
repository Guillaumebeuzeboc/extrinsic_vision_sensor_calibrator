#!/usr/bin/env python

# ROS
import rospy
from extrinsic_vision_sensor_calibrator.calibrator import Calibrator


if __name__ == '__main__':
    rospy.init_node('Calibrator_display')
    calibrator = Calibrator()

    rospy.spin()
