#!/usr/bin/env python
import unittest, time

from dynamic_reconfigure.client import Client
import rospy
from actionlib import SimpleActionClient, SimpleActionServer

from march_shared_resources.msg import GaitActionGoal, GaitGoal, GaitAction

PKG = 'march_gain_scheduling'


def srv_callback(config, level):
    rospy.loginfo('Reconfigure Request: {p}, {i}, {d}'.format(**config))
    return config


class GainSchedulingTest(unittest.TestCase):
    def __init__(self, *args, **kwargs):
        super(GainSchedulingTest, self).__init__(*args, **kwargs)
        self.client = Client('/march/controller/trajectory/gains/test_joint1', timeout=10)
        self.pub = rospy.Publisher('/march/gait/schedule/goal', GaitActionGoal, queue_size=1)
        self._as = SimpleActionServer('/march/gait/schedule', GaitAction, auto_start=True)
        self.act_client = SimpleActionClient('/march/gait/schedule', GaitAction)
        # self.srv = Server(GainListConfig, srv_callback, namespace='/march/controller/trajectory/gains/test_joint1')
        self.config = None
        self.success = False

    def setUp(self):
        self.client.update_configuration({'p': 1000, 'i': 0, 'd': 10})

    # def tearDown(self):
    #     self.client.close()

    def gait_callback(self, config):
        self.config = config
        self.success = True

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
        update_message = GaitGoal()
        update_message.current_subgait.gait_type = 'sit_like'
        self.act_client.wait_for_server()
        self.act_client.send_goal(update_message)
        timeout_t = time.time() + 10.0  # 10 seconds
        rate = rospy.Rate(10)
        while not rospy.is_shutdown() and not self.success and time.time() < timeout_t:
            rate.sleep()
        updated_val = self.client.get_configuration()
        self.assertEqual(updated_val['p'], 10001, 'use of the dynamic reconfigure class did not update the pid values \
        current value is {0}, {1}, \n{2}, \nsuccess: {3} '.format(updated_val['p'], self.config, update_message,
                                                                  self.success))


if __name__ == '__main__':
    import rostest
    rospy.init_node('gainschedulingtest')
    rostest.rosrun(PKG, 'test_gain_scheduling', GainSchedulingTest)
    rospy.spin()
