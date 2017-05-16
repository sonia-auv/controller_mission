import rospy

from ..mission_state import MissionState, Parameter
from proc_control.srv import SetPositionTarget
from proc_image_processing.msg import VisionTarget
from nav_msgs.msg import Odometry


class AlignToVision(MissionState):

    buoy_diameter = 0.23

    def __init__(self):
        MissionState.__init__(self)
        self.position_x = 0.0
        self.position_y = 0.0
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

        self.rotate = False

    def define_parameters(self):
        self.parameters.append(Parameter('param_target', 1.0, 'target'))
        self.parameters.append(Parameter('param_color', 'green', 'target'))
        self.parameters.append(Parameter('param_min_width', 1.0, 'target'))
        self.parameters.append(Parameter('param_heading', 1.0, 'target'))

    def get_outcomes(self):
        return ['succeeded', 'aborted']

    def vision_callback(self, position):
        if self.param_color == position.desc_1:
            pixel_to_meter = position.width / self.buoy_diameter

            self.vision_position_y = (position.y / pixel_to_meter) - (300 / pixel_to_meter)
            self.vision_position_z = (position.x / pixel_to_meter) - (250 / pixel_to_meter)

            if position.width <= self.param_min_width:
                self.rotate = True

            if abs(self.vision_position_y) <= self.param_target:
                self.buoy_is_reach_y = True

            if abs(self.vision_position_z) <= self.param_target:
                self.buoy_is_reach_z = True

    def position_callback(self, position):
        self.position_x = position.pose.pose.position.z
        self.position_y = position.pose.pose.position.y

    def set_z_position(self, pos_z, actual_pos_z):
        if pos_z < 0:
            return actual_pos_z - abs(pos_z)
        else:
            return actual_pos_z + pos_z

    def align_submarine(self):
        align_type = self.rotate

        posy = self.vision_position_y
        posz = self.vision_position_z
        sub_posz = self.position_x

        if align_type:
            if posy < 0:
                posyaw = -40.0
            else:
                posyaw = 40.0
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
        rospy.loginfo('Set relative position yaw = %f' % position_yaw)
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
