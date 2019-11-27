#!/usr/bin/env python
import unittest
import rospy
from march_gain_scheduling.dynamic_pid_reconfigurer import DynamicPIDReconfigurer

PKG = 'march_gain_scheduling'


class GainSchedulingTest(unittest.TestCase):

    def test_empty_object(self):
        # Load the "/march/controller/trajectory/gains/" + self._joint_list[i] parameters
        rospy.init_node("gainschedulingtest")

        self.assertEqual(1, 1, "1!=1")


if __name__ == '__main__':
    import rostest
    rostest.unitrun(PKG, 'test_gain_scheduling', GainSchedulingTest)
