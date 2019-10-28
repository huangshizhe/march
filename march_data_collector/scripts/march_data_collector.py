#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Temperature
from control_msgs.msg import JointTrajectoryControllerState
from march_shared_resources.msg import ImcErrorState

def TemperatureCallback(data, joint):
    rospy.loginfo("Temperature" + joint + " is " + str(data.temperature))

def TrajectoryStateCallback(data):
    rospy.loginfo('received trajectory state'+ str(data.desired))

def TrajectoryStateCallback(data):
    rospy.loginfo('received IMC message current is '+ str(data.current))
    
def main():
    rospy.init_node('data_collector', anonymous=True)
    #get joint names from temperature interface???
    joint_names = ["left_ankle", "left_hip_aa", "left_hip_fe", "left_knee","right_ankle", "right_hip_aa", "right_hip_fe", "right_knee" ]
    TemperatureSubscriber = [rospy.Subscriber('/march/temperature/'+joint, Temperature, TemperatureCallback, (joint)) for joint in joint_names]

    TrajectoryStateSubscriber = rospy.Subscriber('/march/controller/trajectory/state',JointTrajectoryControllerState, TrajectoryStateCallback)

    IMCStateSubscriber = rospy.Subscriber('/march/imc_states',ImcErrorState, ImcStateCallback)
    rospy.loginfo("node initiated")
    rospy.spin()


if __name__ == '__main__':
    main()
