#!/usr/bin/env python3
"""
Dome control node for R2D3.
Handles dome homing and position control.
"""
import rospy
from r2d3_control import dome_wrapper


def main():
    rospy.init_node('dome_node')
    
    dw = dome_wrapper.DomeWrapper()
    dw.setup_service()
    
    rospy.loginfo("dome_node started")
    rospy.spin()


if __name__ == '__main__':
    main()
