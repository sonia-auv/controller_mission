import rospy

from ..mission_state import MissionState, Parameter
from proc_control.msg import TargetReached
from proc_control.srv import SetDecoupledTarget


class MoveDecoupledZFromGlobal(MissionState):

    def __init__(self):
        MissionState.__init__(self)
        self.target_reached = False

    def define_parameters(self):
        self.parameters.append(Parameter('param_distance_z_global_param_name', 1.0, 'Distance to travel'))

    def get_outcomes(self):
        return ['succeeded', 'aborted', 'preempted']

    def target_reach_cb(self, data):
        self.target_reached = data.target_is_reached

    def initialize(self):
        rospy.wait_for_service('/proc_control/set_global_decoupled_target')
        set_global_target = rospy.ServiceProxy('/proc_control/set_global_decoupled_target', SetDecoupledTarget)

        global_z = rospy.get_param(self.param_distance_z_global_param_name, 0.0)

        if not rospy.has_param(self.param_distance_z_global_param_name):
            raise Exception('%s param is no defined' % self.param_distance_z_global_param_name)


        try:
            response = set_global_target(0.0, 0.0, global_z,
                                         0.0, 0.0, 0.0,
                                         True, True, False, True, True, True)
            self.target_reached = False
        except rospy.ServiceException as exc:
            rospy.loginfo('Service did not process request: ' + str(exc))

        rospy.loginfo('Set position z = %f' % global_z)
        self.target_reach_sub = rospy.Subscriber('/proc_control/target_reached', TargetReached, self.target_reach_cb)

    def run(self, ud):
        if self.target_reached > 0:
            return 'succeeded'

    def end(self):
        self.target_reach_sub.unregister()
