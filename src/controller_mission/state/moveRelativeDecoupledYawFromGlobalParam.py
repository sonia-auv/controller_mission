import rospy

from ..mission_state import MissionState, Parameter
from proc_control.msg import TargetReached
from proc_control.srv import SetDecoupledTarget


class MoveRelativeDecoupledYawFromGlobalParam(MissionState):

    def __init__(self):
        MissionState.__init__(self)
        self.target_reached = False

    def define_parameters(self):
        self.parameters.append(Parameter('param_yaw_global_param_name', 1.0, 'Angle change for Yaw'))

    def get_outcomes(self):
        return ['succeeded', 'aborted', 'preempted']

    def target_reach_cb(self, data):
        self.target_reached = data.target_is_reached

    def initialize(self):
        self.target_reached = 0
        rospy.wait_for_service('/proc_control/set_local_decoupled_target')
        set_local_target = rospy.ServiceProxy('/proc_control/set_local_decoupled_target', SetDecoupledTarget)

        relativeYaw = rospy.get_param(self.param_yaw_global_param_name, 0.0)

        if not rospy.has_param(self.param_yaw_global_param_name):
            rospy.logerr('No param defined for %s. A default value is used' % self.param_yaw_global_param_name)

        try:
            set_local_target(0.0, 0.0, 0.0,
                             0.0, 0.0, relativeYaw,
                             True, True, True, True, True, False)

        except rospy.ServiceException as exc:
            rospy.loginfo('Service did not process request: ' + str(exc))

        rospy.loginfo('Set relative angle yaw = %f' % relativeYaw)
        self.target_reach_sub = rospy.Subscriber('/proc_control/target_reached', TargetReached, self.target_reach_cb)

    def run(self, ud):
        if self.target_reached > 0:
            return 'succeeded'

    def end(self):
        self.target_reach_sub.unregister()
