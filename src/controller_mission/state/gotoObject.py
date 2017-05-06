import rospy
import math

from ..mission_state import MissionState, Parameter
from controller_mission.srv import GetPositionObject
from proc_control.msg import TargetReached
from proc_control.srv import SetPositionTarget
from nav_msgs.msg import Odometry


class GotoObject(MissionState):

    def __init__(self):
        MissionState.__init__(self)
        self.actual_position_x = 0.0
        self.actual_position_y = 0.0
        self.actual_heading = 0.0
        self.target_reached = None
        self.object = {'buoys': 1, 'fence': 2, 'hydro': 3}

    def define_parameters(self):
        self.parameters.append(Parameter('go_to_object', 'buoys', 'Aligned to object'))

    def get_outcomes(self):
        return ['succeeded', 'aborted']

    def target_reach_cb(self, data):
        self.target_reached = data.target_is_reached

    def get_actual_position(self, data):
        if self.just_one_time == 0:
            self.actual_position_x = data.pose.pose.position.x
            self.actual_position_y = data.pose.pose.position.y
            self.just_one_time = 1

    def get_heading(self, pos_act_x, pos_act_y, next_pos_x, next_pos_y):
        param_x = next_pos_x - pos_act_x
        param_y = next_pos_y - pos_act_y

        heading = math.degrees(math.atan2(param_x / param_y))

        heading = (360.0 + heading) % 360.0

        return heading

    def initialize(self):
        rospy.wait_for_service('/proc_control/set_global_target')
        self.set_local_target = rospy.ServiceProxy('/proc_control/set_local_target', SetPositionTarget)

        rospy.wait_for_service('/controller_mission/getPositionObject')
        self.get_object_position = rospy.ServiceProxy('/controller_mission/getPositionObject', GetPositionObject)

        rospy.wait_for_message('/proc_navigation/odom', Odometry)
        self.actual_position_sub = rospy.Subscriber('/proc_navigation/odom', Odometry, self.get_actual_position)

        self.target_reach_sub = rospy.Subscriber('/proc_control/target_reached', TargetReached, self.target_reach_cb)

        self.just_one_time = 0

        while self.just_one_time < 2:
            if self.just_one_time == 1:
                act_pos_x = self.actual_position_x
                act_pos_y = self.actual_position_y
                self.just_one_time = 2

        object_position = self.get_object_position(str(self.object[self.go_to_object]))

        pos_x = object_position.position.x
        pos_y = object_position.position.y
        pos_z = object_position.position.z

        self.heading = self.get_heading(act_pos_x, act_pos_y, pos_x, pos_y)

        try:
            self.set_local_target(pos_x,
                                  pos_y,
                                  pos_z,
                                  0.0,
                                  0.0,
                                  self.heading)
        except rospy.ServiceException as exc:
            rospy.loginfo('Service did not process request: ' + str(exc))

    def run(self, ud):
        if self.target_reached > 0:
            return 'succeeded'

    def end(self):
        self.target_reach_sub.unregister()
        self.actual_position_sub.unregister()
