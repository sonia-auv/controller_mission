import rospy
import math
import numpy as np
from numpy.linalg import norm

from ..mission_state import MissionState, Parameter
from proc_control.msg import TargetReached
from proc_control.srv import SetPositionTarget
from proc_mapping.msg import MappingRequest
from nav_msgs.msg import Odometry


class GoToObject(MissionState):

    def __init__(self):
        MissionState.__init__(self)
        self.target_reached = False
        self.target_is_set = False

        self.current_pose_x = 0.0
        self.current_pose_y = 0.0

    def define_parameters(self):
        self.parameters.append(Parameter('param_go_to_object', 'buoys', 'Aligned to object'))

    def get_outcomes(self):
        return ['succeeded', 'aborted']

    def target_reach_cb(self, data):
        self.target_reached = data.target_is_reached

    def get_current_position(self, data):
        self.current_pose_x = data.pose.pose.position.x
        self.current_pose_y = data.pose.pose.position.y

    def go_to_object(self, good_object_position):
        reference = np.array([1.0, 0.0])
        pos_act_x = self.current_pose_x
        pos_act_y = self.current_pose_y

        pos_next_x = good_object_position[0]
        pos_next_y = good_object_position[1]

        vector_direction = np.array([pos_next_x, pos_next_y]) - np.array([pos_act_x, pos_act_y])

        heading = math.degrees(math.acos(np.dot(reference, vector_direction) / (norm(reference) * norm(vector_direction))))

        if not((vector_direction[0] >= 0 and vector_direction[1] >= 0) or
                   (vector_direction[0] < 0 and vector_direction[1] >= 0)):
            heading = 360.0 - heading

        self.set_target(good_object_position, heading)

    def set_target(self, position, heading):
        try:
            self.set_global_target(position[0],
                                   position[1],
                                   position[2],
                                   0.0,
                                   0.0,
                                   heading)
        except rospy.ServiceException as exc:
            rospy.loginfo('Service did not process request: ' + str(exc))

        self.target_is_set = True

        rospy.loginfo('Set global position x = %f' % position[0])
        rospy.loginfo('Set global position y = %f' % position[1])
        rospy.loginfo('Set global position z = %f' % position[2])
        rospy.loginfo('Set global position yaw = %f' % heading)

    def initialize(self):
        rospy.wait_for_service('/proc_control/set_global_target')
        self.set_global_target = rospy.ServiceProxy('/proc_control/set_global_target', SetPositionTarget)

        self.found_object = rospy.Publisher('/proc_mapping/mapping_request', MappingRequest, queue_size=10)

        self.getting_current_position = rospy.Subscriber('/proc_navigation/odom', Odometry,
                                                         self.get_current_position)

        self.target_reach_sub = rospy.Subscriber('/proc_control/target_reached', TargetReached, self.target_reach_cb)

        self.current_pose_x = 0.0
        self.current_pose_y = 0.0

        self.target_is_set = False

    def run(self, ud):
        if not self.target_is_set:

            self.go_to_object(ud.generic_data_field_1)

        if self.target_reached > 0:
            return 'succeeded'

    def end(self):
        self.target_reach_sub.unregister()
