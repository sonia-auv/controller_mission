import rospy

from ..mission_state import MissionState, Parameter
from proc_mapping.msg import MappingRequest, MappingResponse
from geometry_msgs.msg import Pose


class FindObject(MissionState):

    def __init__(self):
        MissionState.__init__(self)
        self.param_to_object = {' buoys': MappingRequest.BUOY, ' fence': MappingRequest.FENCE, ' hydro': MappingRequest.PINGER}
        self.object_to_param = {MappingRequest.BUOY: ' buoys', MappingRequest.FENCE: ' fence', MappingRequest.PINGER: ' hydro'}
        self.object_is_found = False

        self.last_marker_position = Pose.position()
        self.count = 0

    def define_parameters(self):
        self.parameters.append(Parameter('param_object_to_found', 'buoys', 'object to find'))
        self.parameters.append(Parameter('param_redundancy', 3, 'object to find'))
        self.parameters.append(Parameter('param_bounding_box', 0.1, 'object to find'))

    def get_outcomes(self):
        return ['succeeded', 'aborted']

    def get_position_object(self, data):
        if self.object_to_param[self.param_object_to_found] == data.mapping_request.object_type:

            if self.last_marker_position - data.data <= self.param_bounding_box:
                self.count += 1
                self.object_is_found = True

    def initialize(self):
        rospy.Subscriber('/proc_mapping/mapping_response', MappingResponse, self.get_position_object)
        self.found_object = rospy.Publisher('/proc_mapping/mapping_request', MappingRequest, queue_size=10)

    def run(self, ud):
        self.found_object.publish(self.param_to_object[self.param_object_to_found])
        if self.count == self.param_redundancy:
            return 'succeeded'

    def end(self):
        self.found_object.unregister()
