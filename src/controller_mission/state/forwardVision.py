import rospy

from ..mission_state import MissionState, Parameter
from proc_control.srv import SetPositionTarget
from proc_image_processing.msg import VisionTarget


class ForwardVision(MissionState):

    def __init__(self):
        MissionState.__init__(self)
        self.buoy_is_unreach = False

    def define_parameters(self):
        self.parameters.append(Parameter('param_distance_x', 1, 'Target'))
        self.parameters.append(Parameter('param_bounding_box', 1, 'Target'))
        self.parameters.append(Parameter('param_vision_target_width_in_meter', 1, 'Target'))

    def get_outcomes(self):
        return ['succeeded', 'aborted']

    def vision_callback(self, data):
        pixel_to_meter = data.width / self.param_vision_target_width_in_meter

        bounding_box = self.param_bounding_box * pixel_to_meter

        if abs(data.x) >= bounding_box or abs(data.y) >= bounding_box:
            self.buoy_is_unreach = True

    def set_target(self, posx):
        try:
            self.set_local_target(posx,
                                  0.0,
                                  0.0,
                                  0.0,
                                  0.0,
                                  0.0)
        except rospy.ServiceException as exc:
            rospy.loginfo('Service did not process request: ' + str(exc))

        rospy.loginfo('Set relative position y = %f' % posx)

    def initialize(self):
        rospy.wait_for_service('/proc_control/set_local_target')
        self.set_local_target = rospy.ServiceProxy('/proc_control/set_local_target', SetPositionTarget)

        self.buoy_position = rospy.Subscriber('/proc_image_processing/data', VisionTarget, self.vision_callback)

        self.set_target(self.param_distance_x)

    def run(self, ud):
        if self.buoy_is_unreach:
            self.set_target(0.0)
            return 'succeeded'

    def end(self):
        pass
