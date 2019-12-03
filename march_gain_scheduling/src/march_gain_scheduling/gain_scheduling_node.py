#!/usr/bin/env python
from dynamic_pid_reconfigurer import DynamicPIDReconfigurer
import rospy


def main():
    rospy.init_node('march_gain_scheduling_node')
    # The parameters Joint_list and gait_list should be obtained from the parameter server of the workspace.
    joint_list = rospy.get_param('/march/joint_names')
    DynamicPIDReconfigurer('random', joint_list)
    rospy.spin()
