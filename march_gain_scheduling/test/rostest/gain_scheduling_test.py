#!/usr/bin/env python
import unittest

from dynamic_reconfigure.client import Client
import rospy

from march_shared_resources.msg import GaitActionGoal

PKG = 'march_gain_scheduling'


def srv_callback(config, level):
    rospy.loginfo('Reconfigure Request: {p}, {i}, {d}'.format(**config))
    return config


class GainSchedulingTest(unittest.TestCase):
    def __init__(self, *args, **kwargs):
        super(GainSchedulingTest, self).__init__(*args, **kwargs)
        self.client = None
        # self.srv = Server(GainListConfig, srv_callback, namespace='/march/controller/trajectory/gains/test_joint1')
        self.config = None

    def setUp(self):
        self.client = Client('/march/controller/trajectory/gains/test_joint1', timeout=10)
        self.client.update_configuration({'p': 1000, 'i': 0, 'd': 10})
        self.pub = rospy.Publisher('/march/gait/schedule/goal', GaitActionGoal, queue_size=1)
        # self.schedule_gait_client = actionlib.SimpleActionClient('/march/gait/schedule', GaitAction)

    # def tearDown(self):
    #     self._client.close()

    def gait_callback(self, config):
        self.config = config

    def test_reconfigured_pid_values(self):
        old_val = self.client.get_configuration()
        new_val = {'p': 1001, 'i': 0, 'd': 10}
        self.client.update_configuration(new_val)
        updated_val = self.client.get_configuration()
        self.assertEqual(old_val['p'] + 1, updated_val['p'], 'Client configuration did not update as expected old_val =\
        {0} and updated_val = {1}'.format(old_val['p'], updated_val['p']))

    def test_subscription(self):
        end_time = rospy.get_rostime() + rospy.Duration(5)
        while self.pub.get_num_connections() == 0 and rospy.get_rostime() < end_time:
            rospy.sleep(0.1)
        # 1 connection in the dyncon node and another in the second subscriber in this test
        self.assertEqual(self.pub.get_num_connections(), 2, 'No connections {0}'.format(self.pub.get_num_connections()))

    def test_node_reconfiguration(self):
        rospy.Subscriber('/march/gait/schedule/goal', GaitActionGoal, callback=self.gait_callback)
        update_message = GaitActionGoal()
        update_message.goal.current_subgait.gait_type = 'sit_like'
        self.pub.publish(update_message)
        rospy.sleep(2)
        updated_val = self.client.get_configuration()
        self.assertEqual(updated_val['p'], 10001, 'use of the dynamic reconfigure class did not update the pid values \
        current value is {0}, {1}, {2}'.format(updated_val['p'], self.config, update_message))


if __name__ == '__main__':
    import rostest
    rospy.init_node('gainschedulingtest')
    # The service persists between tests, be careful with your expectations after executing a test using DynReCon
    rostest.rosrun(PKG, 'test_gain_scheduling', GainSchedulingTest)
