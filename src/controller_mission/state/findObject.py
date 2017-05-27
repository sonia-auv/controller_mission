import rospy
import threading

from ..mission_state import MissionState, Parameter
from proc_mapping.msg import MappingRequest, MappingResponse
from proc_control.srv import SetPositionTarget
from geometry_msgs.msg import Point


class FindObject(MissionState):

    def __init__(self):
        MissionState.__init__(self)
        self.param_to_object = {' buoys': MappingRequest.BUOY, ' fence': MappingRequest.FENCE, ' hydro': MappingRequest.PINGER}
        self.marker_position = []
        self.just_one_time = True
        self.object_is_found = False

        self.target_reached = False
        self.target_is_set = False

        self.position = Point()

    def define_parameters(self):
        self.parameters.append(Parameter('param_object_to_found', 'buoys', 'object to find'))
        self.parameters.append(Parameter('param_nb_of_marker_to_compute', 10, 'object to find'))

    def get_outcomes(self):
        return ['succeeded', 'aborted']

    def request(self):
        rate = rospy.Rate(5)
        while not self.object_is_found:
            self.found_object.publish(self.param_to_object[self.param_object_to_found])
            rate.sleep()

    def get_markers(self, markers):
        if self.param_to_object[self.param_object_to_found] == markers.mapping_request.object_type:
            self.get_object_position(markers.data.markers)

    def get_object_position(self, markers):
        if len(markers) >= self.param_nb_of_marker_to_compute and self.just_one_time:
            for marker in markers:
                self.marker_position.append(marker.pose.position)
            self.just_one_time = False
            self.parse_object_position(self.marker_position)

    def parse_object_position(self, objects_position):
        sum_position = Point
        for position in objects_position:
            sum_position += position

        self.good_object_position = sum_position / len(objects_position)
        print self.good_object_position
        self.object_is_found = True

    def initialize(self):
        rospy.wait_for_service('/proc_control/set_global_target')
        self.set_global_target = rospy.ServiceProxy('/proc_control/set_global_target', SetPositionTarget)

        self.getting_marker = rospy.Subscriber('/proc_mapping/mapping_response', MappingResponse, self.get_markers)
        self.found_object = rospy.Publisher('/proc_mapping/mapping_request', MappingRequest, queue_size=10)

        self.thread_request = threading.Thread(target=self.request)
        self.thread_request.setDaemon(1)
        self.thread_request.start()

        self.just_one_time = True

        self.target_reached = False

    def run(self, ud):
        if self.object_is_found:
            ud.generic_data_field_1 = self.good_object_position
            return 'succeeded'

    def end(self):
        self.getting_marker.unregister()
