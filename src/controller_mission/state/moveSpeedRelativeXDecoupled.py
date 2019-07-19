import rospy

from ..mission_state import MissionState, Parameter
from proc_control.srv import SetDecoupledTarget
from nav_msgs.msg import Odometry
import math


class MoveRelativeSpeedXDecoupled(MissionState):

    def __init__(self):
        MissionState.__init__(self)
        self.set_local_target = None
        self.odom = None
        self.first_position = None
        self.position = None

    def define_parameters(self):
        self.parameters.append(Parameter('param_distance_x', 1.0, 'Distance to travel'))
        self.parameters.append(Parameter('param_speed_x', 1.0, 'Speed to use while traveling'))
        self.parameters.append(Parameter('param_orientation_yaw', 0.0, 'Heading'))

    def get_outcomes(self):
        return ['succeeded', 'aborted', 'preempted']

    def initialize(self):
        rospy.wait_for_service('/proc_control/set_local_decoupled_target')
        self.set_local_target = rospy.ServiceProxy('/proc_control/set_local_decoupled_target', SetDecoupledTarget)

        self.odom = rospy.Subscriber('/proc_navigation/odom', Odometry, self.odom_cb)

        self.wait_until_position_is_get()

        try:
            self.set_local_target(self.param_speed_x,
                                  0.0,
                                  self.position.z,
                                  0.0,
                                  0.0,
                                  self.param_orientation_yaw,
                                  False, False, False, True, True, False)
        except rospy.ServiceException as exc:
            rospy.loginfo('Service did not process request: ' + str(exc))

        rospy.loginfo('Set distance to travel x = %f' % self.param_distance_x)
        rospy.loginfo('Set relative speed x = %f' % self.param_speed_x)
        rospy.loginfo('Set global orientation yaw = %f' % self.param_orientation_yaw)

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
            try:
                self.set_local_target(0.0,
                                      0.0,
                                      0.0,
                                      0.0,
                                      0.0,
                                      0.0,
                                      False, False, False, True, True, False)
            except rospy.ServiceException as exc:
                rospy.loginfo('Service did not process request: ' + str(exc))
            return 'succeeded'

    def end(self):
        self.odom.unregister()
