import rospy

from ..mission_state import MissionState, Parameter
from proc_control.msg import TargetReached
from proc_control.srv import SetDecoupledTarget
from nav_msgs.msg import Odometry


class MoveToSavedOrientationFromGlobalParam(MissionState):

    def __init__(self):
        MissionState.__init__(self)

        self.set_global_target = None
        self.target_reach_sub = None
        self.target_reached = None

        self.orientation = None

        self.just_one_time = None

    def define_parameters(self):
        self.parameters.append(Parameter('param_global_param_name', '/save_orientation', 'Orientation param name'))

    def get_outcomes(self):
        return ['succeeded', 'aborted', 'preempted']

    def target_reach_cb(self, data):
        self.target_reached = data.target_is_reached

    def set_target(self, data):

        try:
            self.set_global_target(0.0,
                                   0.0,
                                   0.0,
                                   0.0,
                                   0.0,
                                   data,
                                   True, True, True, True, True, False)
            self.target_reached = False
        except rospy.ServiceException as exc:
            rospy.loginfo('Service did not process request: ' + str(exc))

        rospy.loginfo('Set position yaw = %f' % data)

    def initialize(self):

        rospy.wait_for_service('/proc_control/set_global_decoupled_target')
        self.set_global_target = rospy.ServiceProxy('/proc_control/set_global_decoupled_target', SetDecoupledTarget)

        self.target_reach_sub = rospy.Subscriber('/proc_control/target_reached', TargetReached, self.target_reach_cb)

        self.orientation = rospy.get_param(self.param_global_param_name)

        self.just_one_time = True

    def run(self, ud):
        if self.just_one_time:
            self.set_target(self.orientation)

        self.just_one_time = False

        if self.target_reached > 0:
            return 'succeeded'

    def end(self):
        self.target_reach_sub.unregister()
