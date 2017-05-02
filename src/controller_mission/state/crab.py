import rospy
import math

from ..mission_state import MissionState, Parameter
from proc_control.msg import TargetReached
from proc_control.srv import SetPositionTarget
from nav_msgs.msg import Odometry


class Crab(MissionState):

    def __init__(self):
        MissionState.__init__(self)
        self.target_reached = False
        self.one_more_target_reached = 0
        self.count = 0

        self.actual_heading = 0.0
        self.actual_position_x = 0.0
        self.actual_position_y = 0.0
        self.position_x = 0.0
        self.position_y = 0.0
        self.base_heading = 0.0
        self.offset = []

    def define_parameters(self):
        self.parameters.append(Parameter('param_distance_x', 1.0, 'Distance to travel'))
        self.parameters.append(Parameter('param_distance_y', 1.0, 'Distance to travel'))
        self.parameters.append(Parameter('param_nb_time', 1.0, 'Distance to travel'))

    def get_outcomes(self):
        return ['succeeded', 'aborted']

    def referencial_change(self, x_relative, y_relative, x_absolute, y_absolute, heading):
        position_absolute = []

        theta = self.base_heading
        alpha = heading

        distance = math.hypot(x_relative, y_relative)

        phi = 90 - (theta + alpha)

        param_x = distance * math.sin(math.radians(phi))
        param_y = distance * math.cos(math.radians(phi))

        absolute_position_x = x_absolute + param_x
        absolute_position_y = y_absolute + param_y

        position_absolute.append(absolute_position_x)
        position_absolute.append(absolute_position_y)

        return position_absolute

    def set_offset(self, heading, distance):
        offset = []

        if heading <= 90:
            distance_x = distance * math.cos(math.radians(heading))
            distance_y = distance * math.sin(math.radians(heading))

        elif heading <= 180:
            distance_x = -(distance * math.sin(math.radians(heading - 90)))
            distance_y = distance * math.cos(math.radians(heading - 90))

        elif heading <= 270:
            distance_x = -(distance * math.cos(math.radians(heading - 180)))
            distance_y = -(distance * math.sin(math.radians(heading - 180)))

        elif heading <= 360:
            distance_x = distance * math.sin(math.radians(heading - 270))
            distance_y = -(distance * math.cos(math.radians(heading - 270)))

        offset.append(distance_x)
        offset.append(distance_y)

        return offset

    def set_target(self, pos_x, pos_y, heading):

        try:
            self.set_global_target(pos_x,
                                   pos_y,
                                   1.0,
                                   0.0,
                                   0.0,
                                   heading)

        except rospy.ServiceException as exc:
            rospy.loginfo('Service did not process request: ' + str(exc))

        rospy.loginfo('Set position x = %f' % pos_x)
        rospy.loginfo('Set position y = %f' % pos_y)
        rospy.loginfo('Set heading = %f' % heading)

    def target_reach_cb(self, data):
        self.target_reached = data.target_is_reached
        if data.target_is_reached:
            self.one_more_target_reached += 1

    def _get_actual_position(self, data):
        if self.just_one_time == 0:
            self.actual_position_x = data.pose.pose.position.x
            self.actual_position_y = data.pose.pose.position.y
            self.actual_heading = data.twist.twist.angular.z
            self.just_one_time = 1

    def initialize(self):
        rospy.wait_for_service('/proc_control/set_global_target')
        self.set_global_target = rospy.ServiceProxy('/proc_control/set_global_target', SetPositionTarget)

        rospy.wait_for_message('/proc_navigation/odom', Odometry)
        self.actual_position_sub = rospy.Subscriber('/proc_navigation/odom', Odometry, self._get_actual_position)

        self.target_reach_sub = rospy.Subscriber('/proc_control/target_reached', TargetReached, self.target_reach_cb)

        self.one_more_target_reached = 0
        self.count = 0
        self.just_one_time = 0

        while self.just_one_time < 2:
            if self.just_one_time == 1:
                position_x = self.actual_position_x
                position_y = self.actual_position_y
                heading = self.actual_heading
                self.just_one_time = 2

        self.position_x = position_x
        self.position_y = position_y
        self.base_heading = heading
        if self.base_heading <= 5 or self.base_heading >= 360:
            self.base_headin = 0.0

        self.offset = self.set_offset(heading, self.param_distance_x * 2)

    def run(self, ud):
        if self.target_reached:
            if self.one_more_target_reached == 1:
                actual_pos_x = self.position_x + self.offset[0] * self.count
                actual_pos_y = self.position_y + self.offset[1] * self.count
                self.count += 1
                self.position = self.referencial_change(0.0, self.param_distance_y/2, actual_pos_x, actual_pos_y, 90.0)
                self.set_target(self.position[0], self.position[1], self.base_heading)

            elif self.one_more_target_reached == 2:
                self.position = self.referencial_change(self.param_distance_x, 0.0, self.position[0], self.position[1], 0.0)
                self.set_target(self.position[0], self.position[1], self.base_heading)

            elif self.one_more_target_reached == 3:
                self.position = self.referencial_change(0.0, self.param_distance_y, self.position[0], self.position[1], -90.0)
                self.set_target(self.position[0], self.position[1], self.base_heading)

            elif self.one_more_target_reached == 4:
                self.position = self.referencial_change(self.param_distance_x, 0.0, self.position[0], self.position[1], 0.0)
                self.set_target(self.position[0], self.position[1], self.base_heading)

            elif self.one_more_target_reached == 5:
                self.position = self.referencial_change(0.0, self.param_distance_y/2, self.position[0], self.position[1], 90.0)
                self.set_target(self.position[0], self.position[1], self.base_heading)

            elif self.one_more_target_reached == 6:
                if self.count == self.param_nb_time:
                    return 'succeeded'
                else:
                    self.one_more_target_reached = 0

    def end(self):
        pass
