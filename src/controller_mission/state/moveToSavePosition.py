import rospy

from ..mission_state import MissionState, Parameter
from proc_control.msg import TargetReached
from proc_control.srv import SetPositionTarget
from nav_msgs.msg import Odometry


class MoveToSavePosition(MissionState):

    def __init__(self):
        MissionState.__init__(self)

        self.set_global_target = None
        self.target_reach_sub = None
        self.target_reached = None

        self.just_one_time = None

    def define_parameters(self):
        pass

    def get_outcomes(self):
        return ['succeeded', 'aborted', 'preempted']

    def target_reach_cb(self, data):
        self.target_reached = data.target_is_reached

    def set_target(self, data):

        try:
            self.set_global_target(data.position.x,
                                   data.position.y,
                                   data.position.z,
                                   data.orientation.x,
                                   data.orientation.y,
                                   data.orientation.z)
            self.target_reached = False
        except rospy.ServiceException as exc:
            rospy.loginfo('Service did not process request: ' + str(exc))

        rospy.loginfo('Set position x = %f' % data.position.x)
        rospy.loginfo('Set position y = %f' % data.position.y)
        rospy.loginfo('Set position z = %f' % data.position.z)
        rospy.loginfo('Set position yaw = %f' % data.orientation.z)

    def initialize(self):

        rospy.wait_for_service('/proc_control/set_global_target')
        self.set_global_target = rospy.ServiceProxy('/proc_control/set_global_target', SetPositionTarget)

        self.target_reach_sub = rospy.Subscriber('/proc_control/target_reached', TargetReached, self.target_reach_cb)

        self.just_one_time = True

    def run(self, ud):
        if self.just_one_time:
            self.set_target(ud.generic_data_field_1)

        self.just_one_time = False

        if self.target_reached > 0:
            return 'succeeded'

    def end(self):
        self.target_reach_sub.unregister()
