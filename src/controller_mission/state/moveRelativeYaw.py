import rospy

from ..mission_state import MissionState, Parameter
from proc_control.msg import TargetReached
from proc_control.srv import SetYawTarget


class MoveRelativeYaw(MissionState):

    def __init__(self):
        MissionState.__init__(self)

        self.target_reached = False

    def define_parameters(self):
        self.parameters.append(Parameter('param_heading', 1.0, 'Distance to travel'))

    def get_outcomes(self):
        return ['succeeded', 'aborted', 'preempted']

    def target_reach_cb(self, data):
        self.target_reached = data.target_is_reached

    def initialize(self):

        rospy.wait_for_service('/proc_control/set_yaw_local_target')
        set_yaw_local_target = rospy.ServiceProxy('/proc_control/set_yaw_local_target', SetYawTarget)

        try:
            response = set_yaw_local_target(self.param_heading)
            self.target_reached = False
        except rospy.ServiceException as exc:
            rospy.loginfo('Service did not process request: ' + str(exc))

        rospy.loginfo('Set position Yaw = %f' % self.param_heading)

        self.target_reach_sub = rospy.Subscriber('/proc_control/target_reached', TargetReached, self.target_reach_cb)

    def run(self, ud):
        if self.target_reached > 0:
            return 'succeeded'

    def end(self):
        self.target_reach_sub.unregister()
