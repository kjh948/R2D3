#!/usr/bin/env python3
"""
Motor control node for R2D3.
Handles wheel motor velocity control.
"""
import rospy
from r2d3_control import wheel_wrapper


def main():
    rospy.init_node('motor_node')
    
    ww = wheel_wrapper.WheelWrapper()
    ww.setup_service()
    
    rospy.loginfo("motor_node started")
    rospy.spin()


if __name__ == '__main__':
    main()
