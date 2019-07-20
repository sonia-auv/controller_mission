import rospy

from ..mission_state import MissionState, Parameter
from proc_control.srv import SetPositionTarget
from nav_msgs.msg import Odometry
import math


class MoveSpeedToPosition(MissionState):

    def __init__(self):
        MissionState.__init__(self)
        self.set_local_target = None
        self.odom = None
        self.position = None
        self.orientation = None

        self.distance_x = None
        self.distance_y = None
        self.yaw = None

        self.last_yaw_update_time = None

    def define_parameters(self):
        self.parameters.append(Parameter('param_x', 1.0, 'X to join'))
        self.parameters.append(Parameter('param_y', 1.0, 'Y to join'))
        self.parameters.append(Parameter('param_speed_x', 1.0, 'Speed to use while traveling'))
        self.parameters.append(Parameter('param_tolerance_radius', 1.0, 'Radius of tolerance between wanted position and reached position'))
        self.parameters.append(Parameter('param_time_between_yaw_adjustment', 2.0, 'Time between yaw adjustment in second'))

    def get_outcomes(self):
        return ['succeeded', 'aborted', 'preempted']

    def initialize(self):
        rospy.wait_for_service('/proc_control/set_global_target')
        self.set_local_target = rospy.ServiceProxy('/proc_control/set_global_target', SetPositionTarget)

        self.odom = rospy.Subscriber('/proc_navigation/odom', Odometry, self.odom_cb)

        self.wait_until_position_is_get()

        self.calculate_distance_yaw()

        try:
            self.set_local_target(self.param_speed_x,
                                  0.0,
                                  self.position.z,
                                  0.0,
                                  0.0,
                                  self.yaw)
        except rospy.ServiceException as exc:
            rospy.loginfo('Service did not process request: ' + str(exc))

        rospy.loginfo('Set position to join x = %f y = %f' % (self.param_x, self.param_y))
        rospy.loginfo('Set relative speed x = %f' % self.param_speed_x)
        rospy.loginfo('Set global orientation yaw = %f' % self.yaw)

        self.last_yaw_update_time = rospy.get_rostime()

    def wait_until_position_is_get(self):
        while not self.position:
            pass
        return

    def calculate_distance_yaw(self):
        self.distance_x = self.param_x - self.position.x
        self.distance_y = self.param_y - self.position.y
        self.yaw = math.atan2(self.distance_y, self.distance_x) * 180 / math.pi

    def odom_cb(self, odom_data):
        self.position = odom_data.pose.pose.position
        self.orientation = odom_data.pose.pose.orientation

    def run(self, ud):
        # Check if both position are set. If not, check pass

        self.calculate_distance_yaw()

        distance = math.sqrt(self.distance_x * self.distance_x + self.distance_y * self.distance_y)

        if distance <= self.param_tolerance_radius:
            try:
                self.set_local_target(0.0,
                                      0.0,
                                      0.0,
                                      0.0,
                                      0.0,
                                      0.0)
            except rospy.ServiceException as exc:
                rospy.loginfo('Service did not process request: ' + str(exc))
            return 'succeeded'

        elif rospy.get_rostime().secs - self.last_yaw_update_time.secs >= self.param_time_between_yaw_adjustment:
            try:
                self.set_local_target(self.param_speed_x,
                                      0.0,
                                      self.position.z,
                                      0.0,
                                      0.0,
                                      self.yaw)
            except rospy.ServiceException as exc:
                rospy.loginfo('Service did not process request: ' + str(exc))

            rospy.loginfo('Set position to join x = %f y = %f' % (self.param_x, self.param_y))
            rospy.loginfo('Set relative speed x = %f' % self.param_speed_x)
            rospy.loginfo('Set global orientation yaw = %f' % self.yaw)

            self.last_yaw_update_time = rospy.get_rostime()


    def end(self):
        self.odom.unregister()
