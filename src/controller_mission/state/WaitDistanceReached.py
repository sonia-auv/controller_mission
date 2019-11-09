import rospy

from ..mission_state import MissionState, Parameter
from proc_control.srv import SetPositionTarget
from nav_msgs.msg import Odometry
import math


class WaitDistanceReached(MissionState):

    def __init__(self):
        MissionState.__init__(self)
        self.set_local_target = None
        self.odom = None
        self.first_position = None
        self.position = None

    def define_parameters(self):
        self.parameters.append(Parameter('param_distance_x', 1.0, 'Distance to travel before succeeded'))

    def get_outcomes(self):
        return ['succeeded', 'aborted', 'preempted']

    def initialize(self):
        self.first_position = None
        self.position = None

        rospy.wait_for_service('/proc_control/set_local_target')
        self.set_local_target = rospy.ServiceProxy('/proc_control/set_local_target', SetPositionTarget)

        self.odom = rospy.Subscriber('/proc_navigation/odom', Odometry, self.odom_cb)

        self.wait_until_position_is_get()

        rospy.loginfo('Set distance to travel before succeeded = %f' % self.param_distance_x)

    def wait_until_position_is_get(self):
        while not self.first_position:
            pass
        return

    def odom_cb(self, odom_data):
        if self.first_position is None:
            self.first_position = odom_data.pose.pose.position

        self.position = odom_data.pose.pose.position

    def run(self, ud):
        # Check if both position are set. If not, check pass
        if self.first_position is None or self.position is None:
            return

        x = self.first_position.x - self.position.x
        y = self.first_position.y - self.position.y

        distance = math.sqrt(x * x + y * y)

        if distance >= self.param_distance_x:
            return 'succeeded'

    def end(self):
        self.odom.unregister()
