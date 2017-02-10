import rospy

from ..mission_state import MissionState, Parameter
from proc_control.msg import TargetReached
from proc_control.srv import SetPositionTarget

_author_ = 'Kevin Coombs'


class Move(MissionState):

    def __init__(self):
        MissionState.__init__(self)

    def define_parameters(self):
        self.parameters.append(Parameter('param_distance_x',1.0,'Distance to travel'))
        self.parameters.append(Parameter('param_distance_y',1.0,'Distance to travel'))
        self.parameters.append(Parameter('param_distance_z',1.0,'Distance to travel'))
        self.parameters.append(Parameter('param_distance_w',1.0,'Distance to travel'))
        self.parameters.append(Parameter('param_distance_t',1.0,'Distance to travel'))

    def get_outcomes(self):
        return ['succeeded', 'aborted']

    def target_reach_cb(self, data):
        self.target_reached = data.target_is_reached

    def initialize(self):

        target_reach_sub = rospy.Subscriber('/proc_control/target_reached', TargetReached, self.target_reach_cb)
        self.target_reached = 0
        rospy.wait_for_service('/proc_control/set_global_target')
        set_local_target = rospy.ServiceProxy('/proc_control/set_global_target', SetPositionTarget)

        try:
                response = set_local_target(self.param_distance_x,
                                            self.param_distance_y,
                                            1.0,
                                            0.0,
                                            0.0,
                                            0.0)
        except rospy.ServiceException as exc:
            rospy.loginfo('Service did not process request: ' + str(exc))

        rospy.loginfo('Set position = %f' % self.param_distance_x)

    def run(self, ud):
        #if self.target_reached:
        return 'succeeded'
