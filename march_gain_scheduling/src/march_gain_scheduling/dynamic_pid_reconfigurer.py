import rospy
from dynamic_reconfigure.client import Client
from march_shared_resources.msg import GaitActionGoal
from interpolate_pid_values import interpolate
from .dynamic_pid_reconfigurer_errors import InvalidJointListError


class DynamicPIDReconfigurer:
    def __init__(self, gait_name="no_gait", joint_list=None):
        self._gait_name = gait_name
        self._clients = []
        self._linearize = False
        if joint_list is None:
            self._joint_list = []
        elif not isinstance(joint_list, list):
            raise InvalidJointListError("joint list not of type list")
        else:
            self._joint_list = joint_list
        for i in range(len(self._joint_list)):
            self._clients.append(Client("/march/controller/trajectory/gains/" + self._joint_list[i], timeout=30))
        self._gait_sub = rospy.Subscriber("/march/gait/schedule/goal",
                                          GaitActionGoal,
                                          callback=self.gait_selection_callback)
        if rospy.has_param("/linearize_gain_scheduling"):
            self._linearize = rospy.get_param("/linearize_gain_scheduling")

    def gait_selection_callback(self, data):
        rospy.logdebug("This is the gait name: %s", data.goal.current_subgait.gait_type)
        if self._gait_name != data.goal.current_subgait.gait_type:
            rospy.logdebug("The selected gait: {0} is not the same as the previous gait: {1}".format(
                data.goal.current_subgait.gait_type, self._gait_name))
            self._gait_name = data.goal.current_subgait.gait_type
            self.client_update()

    def client_update(self):
        rospy.logdebug("self.linearize is: {0}".format(self._linearize))
        if self._linearize:
            for i in range(len(self._joint_list)):
                p_value, i_value, d_value = self.look_up_table(i)
                p_value_list = interpolate(p_value, self._clients[i].get_configuration()[0])
                for k in range(len(p_value_list)):
                    self._clients[i].update_configuration({"p": p_value_list[k],
                                                           "i": i_value,
                                                           "d": d_value})
                    rospy.logdebug("Config set to {0}, {1}, {2}".format(p_value, i_value, d_value))
        else:
            for i in range(len(self._joint_list)):
                p_value, i_value, d_value = self.look_up_table(i)
                if p_value is not None:
                    self._clients[i].update_configuration({"p": p_value,
                                                           "i": i_value,
                                                           "d": d_value})
                    rospy.logdebug("nonlinear Config set to {0}, {1}, {2}".format(p_value, i_value, d_value))

    # Method that pulls the PID values from the gains_per_gait_type.yaml config file
    def look_up_table(self, i):
        if rospy.has_param("~gait_types/" + self._gait_name):
            gains = rospy.get_param("~gait_types/" + self._gait_name + "/" + self._joint_list[i])
            return gains['p'], gains['i'], gains['d']
        else:
            return None, None, None
