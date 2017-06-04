import rospy

from ..mission_state import MissionState, Parameter
from proc_control.srv import SetPositionTarget
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
        self.buoy_position = None
        self.sub_position = None

    def define_parameters(self):
        self.parameters.append(Parameter('param_bounding_box', 1.0, 'bounding box'))
        self.parameters.append(Parameter('param_color', 'green', 'color of object to align'))
        self.parameters.append(Parameter('param_threshold_width', 1.0, 'maximum nb of pixel to align with heading'))
        self.parameters.append(Parameter('param_heading', 1.0, 'Yaw rotation to align vision'))
        self.parameters.append(Parameter('param_vision_target_width_in_meter', 0.23, 'transform pixel to meter'))
        self.parameters.append(Parameter('param_topic_to_listen', '/proc_image_processing/data', 'Name of topic to listen'))
        self.parameters.append(Parameter('param_nb_pixel_to_victory', 300, 'Minimal nb of pixel to ram'))

    def get_outcomes(self):
        return ['succeeded', 'aborted', 'forward']

    def vision_cb(self, position):
        if self.param_color == position.desc_1:
            pixel_to_meter = position.width / self.param_vision_target_width_in_meter

            self.vision_position_y = position.x / pixel_to_meter
            self.vision_position_z = position.y / pixel_to_meter

            if position.width >= self.param_nb_pixel_to_victory:
                self.victory = True

            if position.width <= self.param_threshold_width:
                self.is_align_with_heading_active = True

            if abs(self.vision_position_y) <= self.param_bounding_box:
                self.vision_is_reach_y = True

            if abs(self.vision_position_z) <= self.param_bounding_box:
                self.vision_is_reach_z = True

    def align_submarine(self):
        alignment_type = self.is_align_with_heading_active

        stare_pos_y = self.vision_position_y
        stare_pos_z = self.vision_position_z

        if alignment_type and stare_pos_y < 0:
            pos_yaw = -self.param_heading
            stare_pos_y = 0.0
        elif alignment_type:
            pos_yaw = self.param_heading
            stare_pos_y = 0.0
        else:
            pos_yaw = 0.0

        if self.vision_is_reach_y:
            pos_yaw = 0.0
            stare_pos_y = 0.0
        if self.vision_is_reach_z:
            stare_pos_z = 0.0

        self.set_target(stare_pos_y, stare_pos_z, pos_yaw)

        if self.vision_is_reach_y and self.vision_is_reach_z:
            self.vision_is_reach = True

    def set_target(self, position_y, position_z, position_yaw):
        try:
            self.set_local_target(0.0,
                                  position_y,
                                  position_z,
                                  0.0,
                                  0.0,
                                  position_yaw)
        except rospy.ServiceException as exc:
            rospy.loginfo('Service did not process request: ' + str(exc))

        rospy.loginfo('Set relative position y = %f' % position_y)
        rospy.loginfo('Set global position z = %f' % position_z)
        rospy.loginfo('Set relative position yaw = %f' % position_yaw)

    def initialize(self):
        rospy.wait_for_service('/proc_control/set_local_target')
        self.set_local_target = rospy.ServiceProxy('/proc_control/set_local_target', SetPositionTarget)

        self.buoy_position = rospy.Subscriber(str(self.param_topic_to_listen[1:]), VisionTarget, self.vision_cb)

        self.vision_is_reach_y = False
        self.vision_is_reach_z = False
        self.vision_is_reach = False

    def run(self, ud):
        self.align_submarine()
        if self.vision_is_reach and not self.victory:
            return 'forward'
        if self.victory and self.vision_is_reach:
            return 'succeeded'

    def end(self):
        self.buoy_position.unregister()
        self.sub_position.unregister()
