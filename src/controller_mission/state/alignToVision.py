import rospy

from ..mission_state import MissionState, Parameter
from proc_control.srv import SetPositionTarget
from proc_image_processing.msg import VisionTarget
from nav_msgs.msg import Odometry


class AlignToVision(MissionState):

    def __init__(self):
        MissionState.__init__(self)
        self.position_x = 0.0
        self.vision_position_y = 0
        self.vision_position_z = 0
        self.vision_width = 0
        self.vision_height = 0
        self.target_reached = False
        self.keep_calm = 1
        self.count = 0

        self.buoy_is_reach_y = False
        self.buoy_is_reach_z = False
        self.buoy_is_reach = False

        self.is_align_with_heading_active = False

        self.victory = False

        self.count = 0

    def define_parameters(self):
        self.parameters.append(Parameter('param_bounding_box', 1.0, 'bounding box'))
        self.parameters.append(Parameter('param_color', 'green', 'color of object to align'))
        self.parameters.append(Parameter('param_threshold_width', 1.0, 'maximum nb of pixel to align with heading'))
        self.parameters.append(Parameter('param_heading', 1.0, 'Yaw rotation to align vision'))
        self.parameters.append(Parameter('param_vision_target_width_in_meter', 1.0, 'transform pixel to meter'))
        self.parameters.append(Parameter('param_topic_to_listen', '/proc_image_processing/data', 'name of topic to listen'))
        self.parameters.append(Parameter('param_nb_pixel_to_victory', 300, 'minimal nb of pixel to ram'))

    def get_outcomes(self):
        return ['succeeded', 'aborted', 'forward']

    def vision_callback(self, position):
        if self.param_color == position.desc_1:
            pixel_to_meter = position.width / self.param_vision_target_width_in_meter

            self.vision_position_y = position.x / pixel_to_meter
            self.vision_position_z = position.y / pixel_to_meter

            if position.width >= self.param_nb_pixel_to_victory:
                self.victory = True

            if position.width <= self.param_threshold_width:
                self.is_align_with_heading_active = True

            if abs(self.vision_position_y) <= self.param_bounding_box:
                self.buoy_is_reach_y = True

            if abs(self.vision_position_z) <= self.param_bounding_box:
                self.buoy_is_reach_z = True

    def position_callback(self, position):
        self.position_x = position.pose.pose.position.z

    def set_z_position(self, pos_z, actual_pos_z):
        if pos_z < 0:
            return actual_pos_z - abs(pos_z)
        else:
            return actual_pos_z + pos_z

    def align_submarine(self):
        alignment_type = self.is_align_with_heading_active

        posy = self.vision_position_y
        posz = self.vision_position_z
        sub_posz = self.position_x

        if alignment_type:
            if posy < 0:
                posyaw = -self.param_heading
            else:
                posyaw = self.param_heading
            posy = 0.0
        else:
            posyaw = 0.0

        posz = self.set_z_position(posz, sub_posz)

        if self.buoy_is_reach_y:
            posyaw = 0.0
            posy = 0.0
        if self.buoy_is_reach_z:
            pass

        self.set_target(posy, posz, posyaw)

        if self.buoy_is_reach_y and self.buoy_is_reach_z:
            self.buoy_is_reach = True

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

        self.buoy_position = rospy.Subscriber('/proc_image_processing/data', VisionTarget, self.vision_callback)

        self.sub_position = rospy.Subscriber('/proc_navigation/odom', Odometry, self.position_callback)

        self.buoy_is_reach_y = False
        self.buoy_is_reach_z = False
        self.buoy_is_reach = False

    def run(self, ud):
        self.align_submarine()
        if self.buoy_is_reach and not self.victory:
            return 'forward'
        if self.victory and self.buoy_is_reach:
            return 'succeeded'

    def end(self):
        self.buoy_position.unregister()
