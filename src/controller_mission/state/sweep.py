import rospy
import math

from ..mission_state import MissionState, Parameter
from proc_control.msg import TargetReached
from proc_control.srv import SetPositionTarget
from nav_msgs.msg import Odometry


class Sweep(MissionState):

    def __init__(self):
        MissionState.__init__(self)
        self.actual_position_x = 0.0
        self.actual_position_y = 0.0
        self.actual_heading = 0.0
        self.just_one_time = 0
        self.one_more_target_reached = 0
        self.target_reached = False

        self.base_position_x = 0.0
        self.base_position_y = 0.0
        self.base_heading = 0.0

        self.center_position_x = 0.0
        self.center_position_y = 0.0

        self.count = 0

    def define_parameters(self):
        self.parameters.append(Parameter('param_heading', 1.0, 'heading'))
        self.parameters.append(Parameter('param_nb', 1.0, 'heading'))
        self.parameters.append(Parameter('param_distance', 1.0, 'Distance to travel'))

    def set_heading(self, pos_x, pos_y):

        heading = abs(math.degrees(math.atan(pos_x / pos_y)))

        if pos_x >= 0.0:
            if pos_y >= 0.0:
                heading = 90.0 - heading
            else:
                heading += 270.0
        else:
            if pos_y >= 0.0:
                heading += 90.0
            else:
                heading = (90 - heading) + 180

        if heading > 360:
            heading = heading - 360

        return heading

    def referencial_change(self, x_relative, y_relative, x_absolute, y_absolute):
        position_absolute = []

        theta = self.base_heading
        alpha = math.atan(y_relative / x_relative)

        distance = math.hypot(x_relative, y_relative)

        phi = 90 - (theta + math.degrees(alpha))

        absolute_x = distance * math.sin(math.radians(phi))
        absolute_y = distance * math.cos(math.radians(phi))

        absolute_position_x = x_absolute + absolute_x
        absolute_position_y = y_absolute + absolute_y

        heading = self.set_heading(absolute_x, absolute_y)

        position_absolute.append(absolute_position_x)
        position_absolute.append(absolute_position_y)
        position_absolute.append(heading)

        return position_absolute

    def check_heading(self, name, heading):
        if heading > 360:
            exec('self.{} = heading - 360'.format(name))

    def get_outcomes(self):
        return ['succeeded', 'aborted']

    def target_reach_cb(self, data):
        self.target_reached = data.target_is_reached
        if data.target_is_reached:
            self.one_more_target_reached += 1

    def get_actual_position(self, data):
        if self.just_one_time == 0:
            self.actual_position_x = data.pose.pose.position.x
            self.actual_position_y = data.pose.pose.position.y
            self.actual_heading = data.twist.twist.angular.z
            self.just_one_time = 1

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

    def initialize(self):

        rospy.wait_for_service('/proc_control/set_global_target')
        self.set_global_target = rospy.ServiceProxy('/proc_control/set_global_target', SetPositionTarget)

        rospy.wait_for_message('/proc_navigation/odom', Odometry)
        self.actual_position_sub = rospy.Subscriber('/proc_navigation/odom', Odometry, self.get_actual_position)

        self.target_reach_sub = rospy.Subscriber('/proc_control/target_reached', TargetReached, self.target_reach_cb)

        self.just_one_time = 0
        self.one_more_target_reached = 0
        self.target_reached = False
        self.count = 0
        self.nb_branch = 2 * self.param_nb + 1

        while self.just_one_time < 2:
            if self.just_one_time == 1:
                position_x = self.actual_position_x
                position_y = self.actual_position_y
                heading = self.actual_heading
                self.just_one_time = 2

        self.base_position_x = position_x
        self.base_position_y = position_y
        self.base_heading = heading

        nb = self.param_nb
        for i in range(1, int(self.param_nb) + 1):
            angle = self.param_heading * nb
            fraction_x = math.cos(math.radians(angle))
            fraction_y = math.sin(math.radians(angle))
            exec('self.position_{}_x = {} * {}'.format(i, self.param_distance, fraction_x))
            exec('self.position_{}_y = {} * {}'.format(i, self.param_distance, fraction_y))
            nb -= 1

        self.center_position_x = self.param_distance
        self.center_position_y = 0.0

    def run(self, ud):
        if self.target_reached:
            if self.one_more_target_reached % 2 != 0:
                self.count += 1
                print self.count
                if self.count <= self.param_nb:
                    exec ('position = self.referencial_change(self.position_{}_x, '
                          'self.position_{}_y, {}, {})'.format(self.count, self.count, self.base_position_x,
                                                               self.base_position_y))

                    self.set_target(position[0], position[1], position[2])

                elif self.count == self.param_nb + 1:
                    position = self.referencial_change(self.center_position_x, self.center_position_y,
                                                       self.base_position_x, self.base_position_y)

                    self.set_target(position[0], position[1], position[2])

                elif self.count == self.nb_branch + 1:
                    return 'succeeded'
                
                elif self.count > self.param_nb:
                    exec ('position = self.referencial_change(self.position_{}_x, '
                          '-self.position_{}_y, {}, {})'.format(int(self.count - (self.param_nb + 1)),
                                                                int(self.count - (self.param_nb + 1)), self.base_position_x,
                                                                self.base_position_y))

                    self.set_target(position[0], position[1], position[2])


            else:
                self.set_target(self.base_position_x, self.base_position_y, self.base_heading)

    def end(self):
        self.target_reach_sub.unregister()
        self.actual_position_sub.unregister()
