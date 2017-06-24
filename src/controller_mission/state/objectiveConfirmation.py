import rospy
import threading

from ..mission_state import MissionState, Parameter
from proc_mapping.msg import GlobalMappingRequest, GlobalMappingResponse


class ObjectiveConfirmation(MissionState):

    def __init__(self):
        MissionState.__init__(self)
        self.param_to_object = {' buoys': GlobalMappingRequest.BUOY, ' fence': GlobalMappingRequest.FENCE,
                                ' hydro': GlobalMappingRequest.PINGER}
        self.object_is_confirm = False

        self.mapping_response = None
        self.request_object_to_confirm = None
        self.thread_request = None

    def define_parameters(self):
        self.parameters.append(Parameter('param_object_to_confirm', 'buoys', 'Object to confirm'))

    def get_outcomes(self):
        return ['succeeded', 'aborted', 'preempted']

    def request(self):
        rate = rospy.Rate(5)
        msg = GlobalMappingRequest()
        msg.object_type = self.param_to_object[self.param_object_to_comfirm]
        while not self.object_is_confirm:
            self.request_object_to_confirm.publish(msg)
            rate.sleep()

    def get_confirmation_cb(self, response):
        if self.param_to_object[self.param_object_to_found] == response.request.object_type:
            self.object_is_confirm = True

    def initialize(self):
        self.mapping_response = rospy.Subscriber('/proc_mapping/mapping_response',
                                                 GlobalMappingResponse, self.get_confirmation_cb)

        self.request_object_to_confirm = rospy.Publisher('/proc_mapping/mapping_request',
                                                         GlobalMappingRequest, queue_size=10)

        self.object_is_confirm = False

        self.thread_request = threading.Thread(target=self.request)
        self.thread_request.setDaemon(1)
        self.thread_request.start()

    def run(self, ud):
        if self.object_is_confirm:
            return 'succeeded'

    def end(self):
        self.mapping_response.unregister()
        del self.thread_request
