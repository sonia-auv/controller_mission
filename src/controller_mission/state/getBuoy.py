import rospy
import threading

from ..mission_state import MissionState, Parameter
from proc_mapping.msg import LocalMappingRequest, LocalMappingResponse
from proc_control.srv import SetPositionTarget


class GetBuoy(MissionState):

    def __init__(self):
        MissionState.__init__(self)

        self.pose = None
        self.set_global_target = None
        self.pub_buoy_request = None
        self.sub_buoy_response = None
        self.thread_request = None

        self.target_is_set = False
        self.buoy_is_not_found = True

    def define_parameters(self):
        self.parameters.append(Parameter('param_pixel_strength_red', 255, 'Buoy color'))
        self.parameters.append(Parameter('param_pixel_strength_green', 255, 'Buoy color'))
        self.parameters.append(Parameter('param_pixel_strength_blue', 255, 'Buoy color'))

    def get_outcomes(self):
        return ['succeeded', 'aborted', 'preempted']

    def request(self):
        rate = rospy.Rate(5)
        msg = LocalMappingRequest()
        msg.color.r = self.param_pixel_strength_red
        msg.color.g = self.param_pixel_strength_green
        msg.color.b = self.param_pixel_strength_blue
        msg.object_type = LocalMappingRequest.BUOY
        while self.buoy_is_not_found:
            self.pub_buoy_request.publish(msg)
            rate.sleep()

    def response_cb(self, data):
        if data.request.color.r >= self.param_pixel_strength_red and \
           data.request.color.g >= self.param_pixel_strength_green and \
           data.request.color.b >= self.param_pixel_strength_blue:

            self.pose = data.posotion
            self.buoy_is_not_found = True

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

        self.pub_buoy_request = rospy.Publisher('/proc_mapping/local_mapping_request', LocalMappingRequest, queue_size=100)

        self.sub_buoy_response = rospy.Subscriber('/proc_mapping/local_mapping_response', LocalMappingResponse, self.response_cb)

        self.buoy_is_not_found = True
        self.target_is_set = False
        self.pose = None

        self.thread_request = threading.Thread(target=self.request)
        self.thread_request.setDaemon(1)
        self.thread_request.start()

    def run(self, ud):
        position = self.pose
        if position:
            self.set_target(position)
            return 'succeeded'

    def end(self):
        self.sub_buoy_response.unregister()
        del self.thread_request
