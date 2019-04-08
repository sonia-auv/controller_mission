import rospy

from ..mission_state import MissionState, Parameter
from proc_control.srv import SetPositionTarget


class MoveSpeed(MissionState):

    def __init__(self):
        MissionState.__init__(self)
        self.set_local_target = None

        self.target_reached = False

    def define_parameters(self):
        self.parameters.append(Parameter('param_distance_x', 1.0, 'Travel speed'))
        self.parameters.append(Parameter('param_distance_z', 1.0, 'Depth'))
        self.parameters.append(Parameter('param_distance_yaw', 0.0, 'Travel orientation'))

    def get_outcomes(self):
        return ['succeeded', 'aborted', 'preempted']

    def initialize(self):
        rospy.wait_for_service('/proc_control/set_local_target')
        self.set_local_target = rospy.ServiceProxy('/proc_control/set_local_target', SetPositionTarget)

        try:
            self.set_local_target(self.param_distance_x,
                                  0.0,
                                  self.param_distance_z,
                                  0.0,
                                  0.0,
                                  self.param_distance_yaw)
        except rospy.ServiceException as exc:
            rospy.loginfo('Service did not process request: ' + str(exc))

        rospy.loginfo('Set relative position x = %f' % self.param_distance_x)
        rospy.loginfo('Set relative position z = %f' % self.param_distance_z)
        rospy.loginfo('Set relative orientation yaw = %f' % self.param_distance_yaw)

    def run(self, ud):
        if self.target_reached:
            return 'succeeded'

    def end(self):
	return
