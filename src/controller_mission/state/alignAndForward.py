import rospy

from ..mission_state import MissionState, Parameter
from proc_control.srv import SetPositionTarget
from proc_image_processing.msg import VisionTarget
from nav_msgs.msg import Odometry


class AlignToVisionAndForward(MissionState):

    buoy_diameter = 0.23
    image_y = 300
    image_x = 250

    def __init__(self):
        MissionState.__init__(self)
        self.position_x = 0.0
        self.position_y = 0.0
        self.vision_position_x = 0
        self.vision_position_y = 0
        self.vision_position_z = 0
        self.vision_width = 0
        self.vision_height = 0
        self.count = 0

        self.buoy_position_z_is_reach = 0
        self.buoy = False

        self.buoy_is_reach_y = False
        self.buoy_is_reach_z = False
        self.buoy_is_reach = False

    def define_parameters(self):
        self.parameters.append(Parameter('param_target_zy', 1.0, 'target'))
        self.parameters.append(Parameter('param_target_x', 1.0, 'target'))
        self.parameters.append(Parameter('param_distance_x', 1.0, 'target'))
        self.parameters.append(Parameter('param_color', 'green', 'target'))

    def get_outcomes(self):
        return ['succeeded', 'aborted']

    def vision_callback(self, position):
        if self.param_color == position.desc_1:
            pixel_to_meter = self.vision_scaling(position.width, position.height)

            self.vision_position_y = (position.y / pixel_to_meter) - (self.image_y / pixel_to_meter)
            self.vision_position_z = (position.x / pixel_to_meter) - (self.image_x / pixel_to_meter)

            if abs(self.vision_position_y) <= self.param_target_zy and abs(pixel_to_meter) >= self.param_target_x:
                self.buoy_is_reach_y = True

            if abs(self.vision_position_z) <= self.param_target_zy and abs(pixel_to_meter) >= self.param_target_x:
                self.buoy_is_reach_z = True
                if self.buoy:
                    self.vision_position_z = self.buoy_position_z_is_reach

    def position_callback(self, position):
        self.position_x = position.pose.pose.position.z
        self.position_y = position.pose.pose.position.y

    def vision_scaling(self, width, height):
        return (width / self.buoy_diameter + height / self.buoy_diameter) / 2

    def set_z_position(self, pos_z, actual_pos_z):
        if pos_z < 0:
            return actual_pos_z - abs(pos_z)
        else:
            return actual_pos_z + pos_z

    def align_submarine(self):
        posy = self.vision_position_y
        posz = self.vision_position_z
        sub_posz = self.position_x

        if self.buoy_is_reach_y:
            posy = 0.0
        if self.buoy_is_reach_z:
            self.buoy = True
            posz = self.buoy_position_z_is_reach

        posz = self.set_z_position(posz, sub_posz)

        self.set_target(posy, posz)

        if self.buoy_is_reach_y and self.buoy_is_reach_z:
            self.buoy_is_reach = True

    def set_target(self, position_y, position_z):
        try:
            self.set_local_target(self.param_distance_x,
                                  position_y,
                                  position_z,
                                  0.0,
                                  0.0,
                                  0.0)
        except rospy.ServiceException as exc:
            rospy.loginfo('Service did not process request: ' + str(exc))

        rospy.loginfo('Set relative position y = %f' % position_y)
        rospy.loginfo('Set global position z = %f' % position_z)

    def initialize(self):
        rospy.wait_for_service('/proc_control/set_local_target')
        self.set_local_target = rospy.ServiceProxy('/proc_control/set_local_target', SetPositionTarget)

        self.buoy_position = rospy.Subscriber('/proc_image_processing/data', VisionTarget, self.vision_callback)

        self.sub_position = rospy.Subscriber('/proc_navigation/odom', Odometry, self.position_callback)

    def run(self, ud):
        self.align_submarine()
        if self.buoy_is_reach:
            return 'succeeded'

    def end(self):
        pass
