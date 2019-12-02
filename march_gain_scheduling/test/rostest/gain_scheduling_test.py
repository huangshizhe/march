#!/usr/bin/env python
import unittest
import rospy

from march_shared_resources.msg import GaitActionGoal, GaitGoal
from march_gain_scheduling.dynamic_pid_reconfigurer import DynamicPIDReconfigurer
from dynamic_reconfigure.client import Client
from dynamic_reconfigure.server import Server
from march_gain_scheduling.cfg import GainListConfig

PKG = 'march_gain_scheduling'


def callback(config, level):
    rospy.loginfo("""Reconfiugre Request: {p}, {i}, {d}""".format(**config))
    return config


class GainSchedulingTest(unittest.TestCase):
    # Load the "/march/controller/trajectory/gains/" + self._joint_list[i] parameters
    #
    # def test_empty_object_for_joint_list(self):
    #     dynamic_pid_reconfigurer = DynamicPIDReconfigurer()
    #     self.assertEqual(dynamic_pid_reconfigurer._joint_list, [], "empty object=empty joint list")
    #
    # def test_empty_object_for_linearization(self):
    #     dynamic_pid_reconfigurer = DynamicPIDReconfigurer()
    #     lin = dynamic_pid_reconfigurer._linearize
    #     self.assertEqual(rospy.get_param("/linearize_gain_scheduling"), lin, "linearize param is set correctly")

    def __init__(self, *args, **kwargs):
        super(GainSchedulingTest, self).__init__(*args, **kwargs)
        self._client = Client("/march/controller/trajectory/gains/test_joint1", timeout=30)

    def setUp(self):
        self._client.update_configuration({"p": 1000, "i": 0, "d": 10})
        return True

    def tearDown(self):
        self._client.close()

    def test_subscription(self):
        pub = rospy.Publisher('/march/gait/schedule/goal', GaitActionGoal, queue_size=1)
        end_time = rospy.get_rostime() + rospy.Duration(5)
        while pub.get_num_connections() == 0 and rospy.get_rostime() < end_time:
            rospy.sleep(0.1)
        self.assertEqual(pub.get_num_connections(), 1, "No connections")

    def test_reconfigured_pid_values(self):
        old_val = self._client.get_configuration()
        new_val = {"p": 1001, "i": 0, "d": 10}
        self._client.update_configuration(new_val)
        updated_val = self._client.get_configuration()
        self.assertEqual(old_val['p']+1, updated_val['p'], "Client configuration did not update as expected old_val = \
        {0} and updated_val = {1}".format(old_val['p'], updated_val['p']))


if __name__ == '__main__':
    import rostest
    rospy.init_node("gainschedulingtest")
    # The service persists between tests, be careful with your expectations after executing a test using DynReCon
    srv = Server(GainListConfig, callback, namespace="/march/controller/trajectory/gains/test_joint1")
    rostest.rosrun(PKG, 'test_gain_scheduling', GainSchedulingTest)
