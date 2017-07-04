import rospy
import threading

from ..mission_state import MissionState, Parameter
from proc_mapping.msg import GlobalMappingRequest, GlobalMappingResponse
from geometry_msgs.msg import Pose


class GetObjective(MissionState):

    def __init__(self):
        MissionState.__init__(self)
        self.param_to_object = {'buoys': GlobalMappingRequest.BUOY, 'fence': GlobalMappingRequest.FENCE,
                                'hydro': GlobalMappingRequest.PINGER}
        self.just_one_time = True
        self.object_is_found = False

        self.getting_objective_position = None
        self.objective_request = None
        self.thread_request = None

        self.target_reached = False
        self.target_is_set = False

        self.objective_position = Pose()

    def define_parameters(self):
        self.parameters.append(Parameter('param_object_to_found', 'buoys', 'object to find'))

    def get_outcomes(self):
        return ['succeeded', 'aborted', 'preempted']

    def request(self):
        rate = rospy.Rate(5)
        msg = GlobalMappingRequest()
        msg.object_type = self.param_to_object[self.param_object_to_found]
        while not self.object_is_found:
            self.objective_request.publish(msg)
            rate.sleep()

    def get_objective_cb(self, response):
        if self.param_to_object[self.param_object_to_found] == response.request.object_type:
            self.objective_position.position = response.point

    def initialize(self):

        self.getting_objective_position = rospy.Subscriber('/proc_mapping/global_mapping_response',
                                                           GlobalMappingResponse, self.get_objective_cb)
        self.objective_request = rospy.Publisher('/proc_mapping/global_mapping_request',
                                                 GlobalMappingRequest, queue_size=10)

        self.just_one_time = True

        self.thread_request = threading.Thread(target=self.request)
        self.thread_request.setDaemon(1)
        self.thread_request.start()

        self.target_reached = False

    def run(self, ud):
        if self.object_is_found:
            ud.generic_data_field_1 = self.objective_position
            ud.generic_data_field_2 = 'global'
            return 'succeeded'

    def end(self):
        self.getting_objective_position.unregister()
        del self.thread_request

