#!/usr/bin/env python3
"""
LED control node for R2D3.
Provides ROS services for emotion-based LED patterns.
"""
import rospy
from r2d3_control import led_wrapper


def main():
    rospy.init_node('led_node')
    
    lw = led_wrapper.LEDWrapper()
    lw.setup_services()
    
    rospy.loginfo("led_node started")
    rospy.spin()


if __name__ == '__main__':
    main()
