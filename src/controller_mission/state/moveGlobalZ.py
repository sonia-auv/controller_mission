import rospy

from ..mission_state import MissionState, Parameter
from proc_control.msg import TargetReached
from proc_control.srv import SetPositionTarget
from nav_msgs.msg import Odometry


class MoveZ(MissionState):

    def __init__(self):
        MissionState.__init__(self)

        self.position = None
        self.orientation = None
        self.target_reached = False

    def define_parameters(self):
        self.parameters.append(Parameter('param_distance_z', 1.0, 'Distance to travel'))

    def get_outcomes(self):
        return ['succeeded', 'aborted', 'preempted']

    def target_reach_cb(self, data):
        self.target_reached = data.target_is_reached

    def current_position_cb(self, position):
        self.position = position.pose.pose.position
        self.orientation = position.pose.pose.orientation

    def wait_until_position_is_get(self):
        while not (self.position and self.orientation):
            pass
        return

    def initialize(self):

        rospy.wait_for_service('/proc_control/set_global_target')
        set_global_target = rospy.ServiceProxy('/proc_control/set_global_target', SetPositionTarget)

        self.current_position = rospy.Subscriber('/proc_navigation/odom', Odometry, self.current_position_cb)

        self.position = None
        self.orientation = None
        self.wait_until_position_is_get()
        self.target_reached = False
        try:
            response = set_global_target(self.position.x,
                                         self.position.y,
                                         self.param_distance_z,
                                         self.orientation.x,
                                         self.orientation.y,
                                         self.orientation.z)
            self.target_reached = False
        except rospy.ServiceException as exc:
            rospy.loginfo('Service did not process request: ' + str(exc))

        rospy.loginfo('Set position z = %f' % self.param_distance_z)

        self.target_reach_sub = rospy.Subscriber('/proc_control/target_reached', TargetReached, self.target_reach_cb)

    def run(self, ud):
        if self.target_reached > 0:
            return 'succeeded'

    def end(self):
        self.target_reach_sub.unregister()
        self.current_position.unregister()
