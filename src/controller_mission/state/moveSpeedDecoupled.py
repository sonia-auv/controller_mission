import rospy

from ..mission_state import MissionState, Parameter
from proc_control.srv import SetDecoupledTarget


class MoveSpeedDecoupled(MissionState):

    def __init__(self):
        MissionState.__init__(self)
        self.set_local_target = None
        self.set_global_target = None

        self.target_reached = False

    def define_parameters(self):
        self.parameters.append(Parameter('param_distance_x', 1.0, 'Speed to use while traveling'))
        self.parameters.append(Parameter('param_distance_z', 1.0, 'Depth during traveling'))
        self.parameters.append(Parameter('param_distance_yaw', 0.0, 'Heading'))

    def get_outcomes(self):
        return ['succeeded', 'aborted', 'preempted']

    def initialize(self):
        rospy.wait_for_service('/proc_control/set_local_decoupled_target')
        rospy.wait_for_service('/proc_control/set_global_decoupled_target')
        self.set_local_target = rospy.ServiceProxy('/proc_control/set_local_decoupled_target', SetDecoupledTarget)
        self.set_global_target = rospy.ServiceProxy('/proc_control/set_global_decoupled_target', SetDecoupledTarget)

        try:
            self.set_local_target(self.param_distance_x,
                                  0.0,
                                  0.0,
                                  0.0,
                                  0.0,
                                  0.0,
                                  False, False, True, True, True, True)

            self.set_global_target(0.0,
                                   0.0,
                                   0.0,
                                   0.0,
                                   0.0,
                                   self.param_distance_yaw,
                                   True, True, True, True, True, False)
        except rospy.ServiceException as exc:
            rospy.loginfo('Service did not process request: ' + str(exc))

        rospy.loginfo('Set sub speed = %f' % self.param_distance_x)
        rospy.loginfo('Set depth = %f' % self.param_distance_z)
        rospy.loginfo('Set global orientation yaw = %f' % self.param_distance_yaw)

    def run(self, ud):
        if self.target_reached:
            return 'succeeded'

    def end(self):
	return