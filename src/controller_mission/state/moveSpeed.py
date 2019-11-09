import rospy

from ..mission_state import MissionState, Parameter
from proc_control.srv import SetPositionTarget


class MoveSpeed(MissionState):

    def __init__(self):
        MissionState.__init__(self)
        self.set_local_target = None

        self.target_reached = False

    def define_parameters(self):
        self.parameters.append(Parameter('param_speed_x', 1.0, 'Speed to use while traveling'))
        self.parameters.append(Parameter('param_orientation_yaw', 0.0, 'Heading'))

    def get_outcomes(self):
        return ['succeeded', 'aborted', 'preempted']

    def initialize(self):
        rospy.wait_for_service('/proc_control/set_local_target')
        self.set_local_target = rospy.ServiceProxy('/proc_control/set_local_target', SetPositionTarget)

        try:
            self.set_local_target(self.param_speed_x,
                                  0.0,
                                  0.0,
                                  0.0,
                                  0.0,
                                  self.param_orientation_yaw)
        except rospy.ServiceException as exc:
            rospy.loginfo('Service did not process request: ' + str(exc))

        rospy.loginfo('Set sub speed = %f' % self.param_speed_x)
        rospy.loginfo('Set global orientation yaw = %f' % self.param_orientation_yaw)

    def run(self, ud):
        if self.target_reached:
            return 'succeeded'

    def end(self):
	return
