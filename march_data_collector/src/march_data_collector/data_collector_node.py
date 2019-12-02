from math import pi

from control_msgs.msg import JointTrajectoryControllerState
from geometry_msgs.msg import TransformStamped
import rospy
from sensor_msgs.msg import Imu, Temperature
from tf.transformations import quaternion_from_euler, quaternion_multiply
import tf2_ros
from urdf_parser_py.urdf import URDF
from visualization_msgs.msg import Marker

from march_shared_resources.msg import ImcErrorState

from .com_calculator import CoMCalculator


class DataCollectorNode(object):

    def __init__(self, com_calculator):
        self._com_calculator = com_calculator
        joint_names = rospy.get_param('/march/joint_names')
        self._imu_broadcaster = tf2_ros.TransformBroadcaster()

        self._marker_publisher = rospy.Publisher('/march/com_marker', Marker, queue_size=1)

        self._temperature_subscriber = [rospy.Subscriber('/march/temperature/' + joint,
                                                         Temperature,
                                                         self.temperature_callback, joint) for joint in joint_names]

        self._trajectory_state_subscriber = rospy.Subscriber('/march/controller/trajectory/state',
                                                             JointTrajectoryControllerState,
                                                             self.trajectory_state_callback)

        self._imc_state_subscriber = rospy.Subscriber('/march/imc_states', ImcErrorState, self.imc_state_callback)

        self._imu_subscriber = rospy.Subscriber('/march/imu', Imu, self.imu_callback)

    def temperature_callback(self, data, joint):
        rospy.logdebug('Temperature' + joint + ' is ' + str(data.temperature))

    def trajectory_state_callback(self, data):
        rospy.logdebug('received trajectory state' + str(data.desired))
        self._marker_publisher.publish(self._com_calculator.calculate_com())

    def imc_state_callback(self, data):
        rospy.logdebug('received IMC message current is ' + str(data.current))

    def imu_callback(self, data):
        if data.header.frame_id == 'imu_link':
            transform = TransformStamped()

            transform.header.stamp = rospy.Time.now()
            transform.header.frame_id = 'world'
            transform.child_frame_id = 'imu_link'
            transform.transform.translation.x = 0.0
            transform.transform.translation.y = 0.0
            transform.transform.translation.z = 0.0

            imu_rotation = quaternion_multiply([-data.orientation.x, -data.orientation.y, data.orientation.z,
                                                data.orientation.w], quaternion_from_euler(0, -0.5 * pi, 0))
            transform.transform.rotation.x = imu_rotation[0]
            transform.transform.rotation.y = imu_rotation[1]
            transform.transform.rotation.z = imu_rotation[2]
            transform.transform.rotation.w = imu_rotation[3]

            self._imu_broadcaster.sendTransform(transform)


def main():
    rospy.init_node('data_collector', anonymous=True)

    robot = URDF.from_parameter_server()
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)  # noqa
    center_of_mass_calculator = CoMCalculator(robot, tf_buffer)

    march_data_collector = DataCollectorNode(center_of_mass_calculator)  # noqa
    rospy.spin()
