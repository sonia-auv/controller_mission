import rospy

from ..mission_state import MissionState, Parameter
from proc_control.srv import SetPositionTarget, EnableControl
from proc_image_processing.msg import VisionTarget


class AlignToVision(MissionState):

    def __init__(self):
        MissionState.__init__(self)
        self.position_z = 0.0
        self.vision_position_y = 0
        self.vision_position_z = 0
        self.vision_width = 0
        self.vision_height = 0

        self.target_reached = False
        self.vision_is_reach_y = False
        self.vision_is_reach_z = False
        self.vision_is_reach = False
        self.is_align_with_heading_active = False
        self.victory = False

        self.set_local_target = None
        self.vision_subscriber = None

        self.count = 0

    def define_parameters(self):
        self.parameters.append(Parameter('param_bounding_box', 0.1, 'bounding box'))
        self.parameters.append(Parameter('param_color', 'red', 'color of object to align'))
        self.parameters.append(Parameter('param_threshold_width', 30, 'maximum nb of pixel to align with heading'))
        self.parameters.append(Parameter('param_heading', 30, 'Yaw rotation to align vision'))
        self.parameters.append(Parameter('param_vision_target_width_in_meter', 0.23, 'transform pixel to meter'))
        self.parameters.append(Parameter('param_topic_to_listen', '/proc_image_processing/buoy_red', 'Name of topic to listen'))
        self.parameters.append(Parameter('param_nb_pixel_to_victory', 300, 'Minimal nb of pixel to ram'))
        self.parameters.append(Parameter('param_maximum_nb_alignment', 4, 'Maximum number of alignment'))

    def get_outcomes(self):
        return ['succeeded', 'aborted', 'forward', 'preempted']

    def vision_cb(self, position):
        if self.param_color == position.desc_1:

            pixel_to_meter = position.width / self.param_vision_target_width_in_meter

            self.vision_position_y = position.x / pixel_to_meter
            self.vision_position_z = position.y / pixel_to_meter

            if position.width >= self.param_nb_pixel_to_victory:
                self.victory = True

            if position.width <= self.param_threshold_width:
                self.is_align_with_heading_active = False
            else:
                self.is_align_with_heading_active = False

            if abs(self.vision_position_y) <= self.param_bounding_box:
                self.vision_is_reach_y = True
            else:
                self.vision_is_reach_y = False

            if abs(self.vision_position_z) <= self.param_bounding_box:
                self.vision_is_reach_z = True
            else:
                self.vision_is_reach_z = False

    def align_submarine(self):
        if self.vision_is_reach_y and self.vision_is_reach_z:
            self.vision_is_reach = True
        elif not self.vision_is_reach_y:
            self.set_y_local_target(self.vision_position_y)
        elif not self.vision_is_reach_z and self.vision_position_y:
            self.set_z_local_target(self.vision_position_z)

    def set_y_local_target(self, position_y):
        try:
            self.set_local_target(0.0,
                                  position_y,
                                  0.0,
                                  0.0,
                                  0.0,
                                  0.0)
        except rospy.ServiceException as exc:
            rospy.loginfo('Service did not process request: ' + str(exc))

        rospy.loginfo('Set relative position y = %f' % position_y)

    def set_z_local_target(self, position_z):
        try:
            self.set_local_target(0.0,
                                  position_z,
                                  0.0,
                                  0.0,
                                  0.0,
                                  0.0)
        except rospy.ServiceException as exc:
            rospy.loginfo('Service did not process request: ' + str(exc))

        rospy.loginfo('Set relative position y = %f' % position_z)

    def initialize(self):
        rospy.wait_for_service('/proc_control/set_local_target')
        self.set_local_target = rospy.ServiceProxy('/proc_control/set_local_target', SetPositionTarget)

        self.vision_subscriber = rospy.Subscriber(self.param_topic_to_listen, VisionTarget, self.vision_cb)

        self.vision_is_reach_y = False
        self.vision_is_reach_z = False
        self.vision_is_reach = False

        self.count += 1

    def run(self, ud):
        self.align_submarine()
        if self.vision_is_reach and not self.victory:
            return 'forward'
        if self.victory and self.vision_is_reach:
            self.set_xy_target(0.0)
            self.set_z_local_target(0.0)
            return 'succeeded'
        if self.count >= self.param_maximum_nb_alignment:
            return 'aborted'

    def end(self):
        self.vision_subscriber.unregister()
