import rospy
import numpy as np
from numpy.linalg import norm
import math
import threading

from ..mission_state import MissionState, Parameter
from proc_mapping.msg import MappingRequest, MappingResponse
from proc_control.msg import TargetReached
from proc_control.srv import SetPositionTarget
from nav_msgs.msg import Odometry


class FindObject(MissionState):

    def __init__(self):
        MissionState.__init__(self)
        self.param_to_object = {' buoys': MappingRequest.BUOY, ' fence': MappingRequest.FENCE, ' hydro': MappingRequest.PINGER}
        self.marker_position = []
        self.just_one_time = True
        self.object_is_found = False

        self.target_pose_x = 0.0
        self.target_pose_y = 0.0
        self.target_pose_yaw = 0.0

        self.good_object_position = np.array([0.0, 0.0, 0.0])

        self.target_reached = False
        self.target_is_set = False

    def define_parameters(self):
        self.parameters.append(Parameter('param_object_to_found', 'buoys', 'object to find'))
        self.parameters.append(Parameter('param_nb_of_marker_to_compute', 10, 'object to find'))
        self.parameters.append(Parameter('param_object_ID', 44, 'object to find'))

    def get_outcomes(self):
        return ['succeeded', 'aborted']

    def request(self):
        rate = rospy.Rate(5)
        while self.object_is_found:
            self.found_object.publish(self.param_to_object[self.param_object_to_found])
            rate.sleep()

    def get_marker(self, msg):
        if self.param_to_object[self.param_object_to_found] == msg.mapping_request.object_type:
            self.get_object_position(msg.data.markers)

    def get_object_position(self, position):
        if len(position) >= self.param_nb_of_marker_to_compute and self.just_one_time:
            for marker in position:
                self.marker_position.append(np.array(
                    [marker.pose.position.x, marker.pose.position.y, marker.pose.position.z]))
            self.just_one_time = False
            self.parse_object_position(self.marker_position)

    def parse_object_position(self, objects_position):
        sum_position = np.array([0.0, 0.0, 0.0])
        for position in objects_position:
            sum_position += position

        self.good_object_position = sum_position / len(objects_position)
        self.object_is_found = True

        self.goto_object()

    def target_reach_cb(self, data):
        self.target_reached = data.target_is_reached

    def get_current_position(self, data):
        self.target_pose_x = data.pose.pose.position.x
        self.target_pose_y = data.pose.pose.position.y

    def goto_object(self):
        reference = np.array([1.0, 0.0])
        pos_act_x = self.target_pose_x
        pos_act_y = self.target_pose_y

        pos_next_x = self.good_object_position[0]
        pos_next_y = self.good_object_position[1]

        vector_direction = np.array([pos_next_x, pos_next_y]) - np.array([pos_act_x, pos_act_y])

        heading = math.degrees(math.acos(np.dot(reference, vector_direction) / (norm(reference) * norm(vector_direction))))

        if vector_direction[0] < 0 or vector_direction[1] < 0:
            heading = 360.0 - heading

        self.set_target(self.good_object_position, heading)

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

        self.getting_marker = rospy.Subscriber('/proc_mapping/mapping_response', MappingResponse, self.get_marker)
        self.found_object = rospy.Publisher('/proc_mapping/mapping_request', MappingRequest, queue_size=10)

        self.getting_current_position = rospy.Subscriber('/proc_navigation/odom', Odometry,
                                                        self.get_current_position)

        self.target_reach = rospy.Subscriber('/proc_control/target_reached', TargetReached, self.target_reach_cb)

        self.thread_request = threading.Thread(target=self.request())
        self.thread_request.setDaemon(1)
        self.thread_request.start()

        self.just_one_time = True

        self.target_reached = False

    def run(self, ud):
        if self.object_is_found and self.target_reached and self.target_is_set:
            return 'succeeded'

    def end(self):
        self.getting_marker.unregister()
