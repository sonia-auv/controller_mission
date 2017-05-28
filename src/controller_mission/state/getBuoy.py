import rospy
import threading

from ..mission_state import MissionState, Parameter
from proc_mapping.msg import MappingRequest
from proc_control.srv import SetPositionTarget


class GetBuoy(MissionState):

    def __init__(self):
        MissionState.__init__(self)
        self.param_to_object = {' buoys': MappingRequest.BUOY, ' fence': MappingRequest.FENCE, ' hydro': MappingRequest.PINGER}
        self.buoy_is_not_found = True
        self.pose = None

    def define_parameters(self):
        self.parameters.append(Parameter('param_buoy_color', 'green', 'Buoy color'))

    def get_outcomes(self):
        return ['succeeded', 'aborted']

    def request(self):
        rate = rospy.Rate(5)
        while self.buoy_is_not_found:
            self.pub_buoy_request.publish()
            rate.sleep()

    def response(self, data):
        self.pose = data.posotion

    def set_target(self, position):
        try:
            self.set_global_target(position.x,
                                   position.y,
                                   position.z,
                                   0.0,
                                   0.0,
                                   0.0)
        except rospy.ServiceException as exc:
            rospy.loginfo('Service did not process request: ' + str(exc))

        self.target_is_set = True

        rospy.loginfo('Set global position x = %f' % position.x)
        rospy.loginfo('Set global position y = %f' % position.y)
        rospy.loginfo('Set global position z = %f' % position.z)

    def initialize(self):
        rospy.wait_for_service('/proc_control/set_global_target')
        self.set_global_target = rospy.ServiceProxy('/proc_control/set_global_target', SetPositionTarget)

        self.pub_buoy_request = rospy.Publisher('/proc_mapping/buoy_request', MappingRequest, queue_size=100)

        self.sub_buoy_response = rospy.Service('/proc_mapping/buoy_response', self.response)

        self.buoy_is_not_found = True
        self.thread_request = threading.Thread(target=self.request)
        self.thread_request.setDaemon(1)
        self.thread_request.start()

        self.pose = None

    def run(self, ud):
        position = self.pose
        if position:
            self.set_target(position)
            return 'succeeded'

    def end(self):
        pass
