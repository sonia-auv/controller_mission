import rospy

from ..mission_state import MissionState, Parameter
from proc_control.srv import SetPositionTarget
from proc_control.msg import TargetReached
from proc_image_processing.msg import VisionTarget


class ForwardVision(MissionState):

    def __init__(self):
        MissionState.__init__(self)
        self.buoy_is_unreached = False
        self.target_reached = False
        self.victory = False

        self.set_local_target = None
        self.buoy_position = None
        self.target_reach_sub = None

    def define_parameters(self):
        self.parameters.append(Parameter('param_color', 'red', 'color of object to align'))
        self.parameters.append(Parameter('param_distance_x', 1, 'Target'))
        self.parameters.append(Parameter('param_bounding_box', 0.1, 'Target'))
        self.parameters.append(Parameter('param_vision_target_width_in_meter', 0.23, 'Target'))
        self.parameters.append(Parameter('param_nb_pixel_to_victory', 300, 'minimal nb of pixel to ram'))
        self.parameters.append(Parameter('param_topic_to_listen', '/proc_image_processing/buoy_red', 'Name of topic to listen'))

    def get_outcomes(self):
        return ['succeeded', 'aborted']

    def vision_cb(self, data):
        if self.param_color == data.desc_1:
            pixel_to_meter = data.width / self.param_vision_target_width_in_meter

            bounding_box = self.param_bounding_box * pixel_to_meter

            if abs(data.x) >= bounding_box or abs(data.y) >= bounding_box:
                self.buoy_is_unreached = True

            if data.width >= self.param_nb_pixel_to_victory:
                self.victory = False

    def target_reach_cb(self, data):
        self.target_reached = data.target_is_reached

    def set_target(self, pos_x):
        try:
            self.set_local_target(pos_x,
                                  0.0,
                                  0.0,
                                  0.0,
                                  0.0,
                                  0.0)
        except rospy.ServiceException as exc:
            rospy.loginfo('Service did not process request: ' + str(exc))

        rospy.loginfo('Set relative position x = %f' % pos_x)

    def initialize(self):
        rospy.wait_for_service('/proc_control/set_local_target')
        self.set_local_target = rospy.ServiceProxy('/proc_control/set_local_target', SetPositionTarget)

        self.buoy_position = rospy.Subscriber(str(self.param_topic_to_listen), VisionTarget, self.vision_cb)

        self.target_reach_sub = rospy.Subscriber('/proc_control/target_reached', TargetReached, self.target_reach_cb)

        self.buoy_is_unreached = False
        self.target_reached = False
        self.victory = False

        self.set_target(self.param_distance_x)

    def run(self, ud):
        if self.buoy_is_unreached or self.target_reached:
            self.set_target(0.0)
            return 'aborted'
        if self.victory:
            return 'succeeded'

    def end(self):
        self.buoy_position.unregister()
        self.target_reach_sub.unregister()
