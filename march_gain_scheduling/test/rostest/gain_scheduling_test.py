#!/usr/bin/env python
import unittest
import rospy
from march_shared_resources.msg import GaitActionGoal, GaitGoal
from march_gain_scheduling.dynamic_pid_reconfigurer import DynamicPIDReconfigurer

PKG = 'march_gain_scheduling'


class GainSchedulingTest(unittest.TestCase):
    # Load the "/march/controller/trajectory/gains/" + self._joint_list[i] parameters
    def __init__(self):
        self.gait_name = ""
        self.subgait = ""

        # Fix beun de action goal met een publisher zodat je niet op antwoord hoeft te wachten.
        self.pub = rospy.publisher("/march/gait/schedule/goal", GaitActionGoal, queue_size=1)
        rospy.init_node("gainschedulingtest")
        self.gait_action_goal = GaitGoal()
        self.gait_action_goal.name = self.gait_name
        self.gait_action_goal.current_subgait = self.subgait
        self.pub.publish(self.gait_action_goal)

    def test_empty_object_for_joint_list(self):
        dynamic_pid_reconfigurer = DynamicPIDReconfigurer()
        self.assertEqual(dynamic_pid_reconfigurer._joint_list, [], "empty object=empty joint list")

    def test_empty_object_for_linearization(self):
        dynamic_pid_reconfigurer = DynamicPIDReconfigurer()
        lin = dynamic_pid_reconfigurer._linearize
        self.assertEqual(rospy.get_param("/linearize_gain_scheduling"), lin, "linearize param is set correctly")


if __name__ == '__main__':
    import src.march.march_gain_scheduling.test.rostest
    src.march.march_gain_scheduling.test.rostest.rosrun(PKG, 'test_gain_scheduling', GainSchedulingTest)
