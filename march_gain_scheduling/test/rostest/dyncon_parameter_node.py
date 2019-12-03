#!/usr/bin/env python
from dynamic_reconfigure.server import Server
import rospy

from march_gain_scheduling.cfg import GainListConfig


def srv_callback(config, level):
    rospy.loginfo("""Reconfigure Request: {p}, {i}, {d}""".format(**config))
    return config


if __name__ == '__main__':
    rospy.init_node('dyncon_server_node')
    rospy.loginfo('server initializing')
    srv = Server(GainListConfig, srv_callback, namespace='/march/controller/trajectory/gains/test_joint1')
    rospy.spin()
