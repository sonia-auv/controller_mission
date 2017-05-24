import rospy
import threading

from ..mission_state import MissionState, Parameter
from proc_mapping.msg import MappingRequest, MappingResponse


class ObjectConfirmation(MissionState):

    def __init__(self):
        MissionState.__init__(self)
        self.param_to_object = {' buoys': MappingRequest.BUOY, ' fence': MappingRequest.FENCE, ' hydro': MappingRequest.PINGER}
        self.object_is_confirm = False

    def define_parameters(self):
        self.parameters.append(Parameter('param_object_to_found', 'buoys', 'object to find'))

    def get_outcomes(self):
        return ['succeeded', 'aborted']

    def request(self):
        rate = rospy.Rate(5)
        while self.object_is_confirm:
            self.confirm_to_object.publish(self.param_to_object[self.param_object_to_found])
            rate.sleep()

    def get_markers(self, markers):
        if markers:
            self.object_is_confirm = True

    def initialize(self):
        self.mapping_response = rospy.Subscriber('/proc_mapping/mapping_response', MappingResponse, self.get_markers)

        self.confirm_to_object = rospy.Publisher('/proc_mapping/mapping_request', MappingRequest, queue_size=10)

        self.thread_request = threading.Thread(target=self.request())
        self.thread_request.setDaemon(1)
        self.thread_request.start()

    def run(self, ud):
        if self.object_is_confirm:
            return 'succeeded'

    def end(self):
        self.mapping_response.unregister()