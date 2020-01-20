#!/usr/bin/env python

# import rospy
# from std_msgs.msg import String
#
# def talker():
#     pub = rospy.Publisher('chatter', String, queue_size=10)
#     rospy.init_node('talker', anonymous=True)
#     rate = rospy.Rate(10) # 10hz
#     while not rospy.is_shutdown():
#         hello_str = "hello world %s" % rospy.get_time()
#         rospy.loginfo(hello_str)
#         pub.publish(hello_str)
#         rate.sleep()
#
# if __name__ == '__main__':
#     try:
#         talker()
#     except rospy.ROSInterruptException:
#         pass

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import String

import rospy

if __name__ == '__main__':
    rospy.init_node('joint_commander')
    pub = rospy.Publisher('/march/controller/trajectory/command', JointTrajectory, queue_size=10)
    command = JointTrajectory()
    command.joint_names = rospy.get_param('/march/controller/trajectory/joints', [])

    rotational = ['left_hip_fe', 'left_knee', 'right_hip_fe', 'right_knee']

    point1 = JointTrajectoryPoint(velocities=[], accelerations=[], effort=[])
    point1.positions = map(lambda j: 0.17 * 3 if j in rotational else 0.17, command.joint_names)
    point1.time_from_start = rospy.Duration(secs=3)

    point2 = JointTrajectoryPoint(velocities=[], accelerations=[], effort=[])
    point2.positions = [0] * len(command.joint_names)
    point2.time_from_start = rospy.Duration(secs=6)

    command.points = [point1, point2]

    rate = rospy.Rate(0.1)
    while not rospy.is_shutdown():
        pub.publish(command)
        rospy.loginfo('Published joint commands')
        try:
            rate.sleep()
        except rospy.ROSInterruptException:
            break
