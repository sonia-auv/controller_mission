import rospy
import math

from ..mission_state import MissionState, Parameter
from proc_control.msg import TargetReached
from proc_control.srv import SetPositionTarget


_author_ = 'Kevin Coombs'


class MoveWithHeading(MissionState):

    def __init__(self):
        MissionState.__init__(self)

    def define_parameters(self):
        self.parameters.append(Parameter('param_distance_x', 1.0, 'Distance to travel'))
        self.parameters.append(Parameter('param_distance_y', 1.0, 'Distance to travel'))
        self.parameters.append(Parameter('param_heading', 1.0, 'Heading for sub'))

    def get_outcomes(self):
        return ['succeeded', 'aborted']

    def target_reach_cb(self, data):
        self.target_reached = data.target_is_reached

    def initialize(self):
        rospy.wait_for_service('/proc_control/set_global_target')
        set_global_target = rospy.ServiceProxy('/proc_control/set_global_target', SetPositionTarget)

        try:
            response = set_global_target(self.param_distance_x,
                                        self.param_distance_y,
                                        1.0,
                                        0.0,
                                        0.0,
                                        self.param_heading)
            self.target_reached = False
        except rospy.ServiceException as exc:
            rospy.loginfo('Service did not process request: ' + str(exc))

        rospy.loginfo('Set position = %f' % self.param_distance_x)
        self.target_reach_sub = rospy.Subscriber('/proc_control/target_reached', TargetReached, self.target_reach_cb)


    def run(self, ud):
        if self.target_reached > 0:
            return 'succeeded'

    def end(self):
        self.target_reach_sub.unregister()
