#!/usr/bin/env python3
"""
Publish the rattle/status msg for debugging.
TODO: needs msg content updating for RattleStatus
"""

import rospy
from rattle_msgs.msg import RattleStatusPrimary
import time


def pub_status():
    msg = RattleStatusPrimary()
    msg.stamp = rospy.get_rostime()
    msg.test_number = 0
    msg.coord_ok = True
    msg.default_control = False
    msg.control_mode = "debug"
    msg.flight_mode = "nominal"

    time.sleep(0.5)
    print('msg sent...')
    pub = rospy.Publisher('/rattle/status', RattleStatusPrimary, queue_size=10, latch=True)
    pub.publish(msg)
    rospy.spin()


if __name__ == '__main__':
    rospy.init_node('pub_node')
    pub_status()
