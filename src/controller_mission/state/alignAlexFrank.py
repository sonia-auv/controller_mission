import rospy
import math

from Queue import deque
from ..mission_state import MissionState, Parameter
from proc_control.msg import TargetReached
from proc_control.srv import SetPositionTarget, SetDecoupledTarget, SetControlMode, SetControlModeRequest
from proc_image_processing.msg import VisionTarget
from nav_msgs.msg import Odometry

"""
The vision part of this code consider use the normal axis for image parsing. 
(e.g X --> Horizontal
     Y --> Vertical)

The control part use the normal axis of the sub.
"""

class AlignAlexFrank(MissionState):
    def __init__(self):
        MissionState.__init__(self)
        self.set_local_target = None
        self.vision_subscriber = None
        self.target_reach_sub = None
        self.set_local_target_topic = None

        self.vision_position_y = 0
        self.vision_position_z = 0
        self.heading = 0

        # Average variables
        self.averaging_vision_x_pixel = 0.0
        self.averaging_vision_y_pixel = 0.0
        self.averaging_vision_width_pixel = 0.0
        self.averaging_vision_height_pixel = 0.0

        self.target_distance = 0.0
        self.target_width = 0.0
        self.target_height = 0.0
        self.vision_distance = 0.5

        self.distance_to_cover_y = 0.0
        self.distance_to_cover_z = 0.0
        self.distance_to_cover_x = 0.0

        self.target_reached = False
        self.vision_is_reach_y = False
        self.vision_is_reach_z = False
        self.vision_is_reach = False

        self.is_align_with_heading_active = False
        self.victory = False
        self.ready_to_advance = False

        # List to grab vision data for position x, y, width and height
        self.vision_data = None

        self.vision_x_pixel = None
        self.vision_y_pixel = None
        self.vision_width_pixel = None
        self.vision_height_pixel = None

        # Parameters to determine target distance.
        self.focal_size = 12.5  # mm
        self.sensor_height = 5.3  # mm
        self.sensor_width = 7.1  # mm

        self.count = 0

        # Control mode options.
        self.mode = SetControlModeRequest()
        self.mode_dic = {'0': self.mode.PositionModePID, '1': self.mode.PositionModePPI, '2': self.mode.VelocityModeB}

        # Position parameters.
        self.odom = None
        self.first_position = None
        self.position = None

        # Bounding box
        self.y_bounding_box = None
        self.x_bounding_box = None

        # Variable for allignment
        self.z_adjustment = None
        self.basic_z_ajustment = 0.5
        self.minimum_z_adjustment = 0.1

        self.minimum_y_adjustment = 0.2
        self.y_adjustment = None
        self.basic_y_adjustment = 1.0

        self.yaw_adjustment = None
        self.minimum_yaw_adjustment = 3.0
        self.basic_y_adjustment = 10.0

    def define_parameters(self):
        self.parameters.append(Parameter('param_heading', 10, 'Yaw rotation to align vision'))
        self.parameters.append(Parameter('param_topic_to_listen', '/proc_image_processing/buoy_red', 'Topic to listen'))
        self.parameters.append(Parameter('param_distance_to_victory', 2, 'Minimal distance to ram (m)'))
        self.parameters.append(Parameter('param_maximum_nb_alignment', 4, 'Maximum number of alignment'))
        self.parameters.append(Parameter('param_max_queue_size', 10, 'Maximum size of queue'))
        self.parameters.append(Parameter('param_object_real_height', 100, 'Object height (mm)'))
        self.parameters.append(Parameter('param_object_real_width', 100, 'Object width (mm)'))
        self.parameters.append(Parameter('param_image_height', 1544, 'Image height (px)'))
        self.parameters.append(Parameter('param_image_width', 2064, 'Image width (px)'))
        self.parameters.append(Parameter('param_offset_y', 0, 'Lateral offset'))
        self.parameters.append(Parameter('param_offset_z', 0, 'Vertical offset'))
        self.parameters.append(Parameter('param_speed_x',0.1,'Speed to travel'))

    def initialize(self):
        rospy.wait_for_service('/proc_control/set_local_decoupled_target')
        self.set_local_target = rospy.ServiceProxy('/proc_control/set_local_decoupled_target', SetDecoupledTarget,
                                                   persistent=True)

        self.target_reach_sub = rospy.Subscriber('/proc_control/target_reached', TargetReached, self.target_reach_cb)

        self.vision_subscriber = rospy.Subscriber(self.param_topic_to_listen, VisionTarget, self.vision_cb)

        self.vision_data = deque([], maxlen=self.param_max_queue_size)

        self.vision_data.clear()

        self.vision_is_reach_y = False
        self.vision_is_reach_z = False
        self.vision_is_reach = False

        # Switch control mode service parameter
        rospy.wait_for_service('/proc_control/set_control_mode')
        self.set_mode = rospy.ServiceProxy('/proc_control/set_control_mode', SetControlMode)
        try:
            # Initialise to position mode to reach depth first
            self.set_mode(self.mode_dic[str(int(0))])
        except rospy.ServiceException as exc:
            rospy.loginfo('Service did not process request: ' + str(exc))

        # Setup odometry service
        self.odom = rospy.Subscriber('/proc_navigation/odom', Odometry, self.odom_cb)
        self.get_first_position()

        # Setup bounding boxes
        self.x_bounding_box = BoundingBox(self.param_image_height, self.param_image_width * 0.15)
        self.y_bounding_box = BoundingBox(self.param_image_height * 0.15, self.param_image_width)

    def run(self, ud):
        self.vision_is_reach = self.vision_is_reach_y and self.vision_is_reach_z

        if self.victory and self.vision_is_reach:
            rospy.loginfo('Vision is Reach : %s', str(self.vision_is_reach))
            rospy.loginfo('Pixel Vision in x : %f', self.averaging_vision_x_pixel)
            rospy.loginfo('Pixel Vision in y : %f', self.averaging_vision_y_pixel)
            self.switch_control_mode(0)
            self.set_target(0.0, 0.0, 0.0, 0.0, False, False, False, False)
            return 'succeeded'

        if self.count >= self.param_maximum_nb_alignment:
            return 'aborted'

    def set_target(self, position_x, position_y, position_z, position_yaw, keepX, keepY, keepZ, keepYaw):
        rospy.loginfo('Set relative position x = %f' % position_x)
        rospy.loginfo('Set relative position y = %f' % position_y)
        rospy.loginfo('Set relative position z = %f' % position_z)
        rospy.loginfo('Set relative position yaw = %f' % position_yaw)

    def target_reach_cb(self, data):
        self.target_reached = data.target_is_reached

    def vision_cb(self, data):
        self.vision_data.append(VisionData(data.x, data.y, data.height, data.width))

        if len(self.vision_data) == self.param_max_queue_size:
            self.parse_vision_data()

    def parse_vision_data(self):
        # Average position of the object (image)
        for data in self.vision_data:
            self.averaging_vision_x_pixel += data.x
            self.averaging_vision_y_pixel += data.y
            self.averaging_vision_width_pixel += data.width
            self.averaging_vision_height_pixel += data.height

        self.averaging_vision_x_pixel /= self.param_max_queue_size
        self.averaging_vision_y_pixel /= self.param_max_queue_size
        self.averaging_vision_width_pixel /= self.param_max_queue_size
        self.averaging_vision_height_pixel /= self.param_max_queue_size

        # Show the average
        rospy.loginfo('Position x : %f' % self.averaging_vision_x_pixel)
        rospy.loginfo('Position y : %f' % self.averaging_vision_y_pixel)
        rospy.loginfo('Height of the object : %f' % self.averaging_vision_width_pixel)
        rospy.loginfo('Width of the object : %f' % self.averaging_vision_height_pixel)

    def switch_control_mode(self,mode):
        try:
            self.set_local_target(0.0,
                                  0.0,
                                  0.0,
                                  0.0,
                                  0.0,
                                  0.0,
                                False,
                                False,
                                False,
                                True,
                                True,
                                False)
            self.set_mode(self.mode_dic[str(int(mode))])
        except rospy.ServiceException as exc:
            rospy.loginfo('Service did not process request: ' + str(exc))

    def odom_cb(self, data):
        if self.first_position is None:
            self.first_position = data.pose.pose.position

        self.position = data.pose.pose.position

    def forward_speed(self):
        try:
            self.switch_control_mode(2)
            self.set_local_target(self.param_speed_x,
                                  0.0,
                                  self.position.z,
                                  0.0,
                                  0.0,
                                  0.0)
        except rospy.ServiceException as exc:
            rospy.loginfo('Service did not process request: ' + str(exc))

    def get_first_position(self):
        while not self.first_position:
            pass
        return

    def is_align_y(self):
        if self.y_bounding_box.is_inside(self.averaging_vision_x_pixel,self.averaging_vision_y_pixel):
            return True
        return False

    def is_align_x(self):
        if self.x_bounding_box.is_inside(self.averaging_vision_x_pixel,self.averaging_vision_y_pixel):
            return True
        return False

    def get_target_distance(self):
        return (self.focal_size * self.param_object_real_height * self.param_image_height) / \
               (self.averaging_vision_height_pixel * self.sensor_height)

    def align_submarine(self):
        rospy.loginfo('Align number %i' % self.count)
        self.count += 1
        if not self.is_align_y():
            self.align_depth()
        else:
            self.forward_speed()
            if not self.is_align_x():
                self.align_yaw()

    def align_depth(self):
        self.switch_control_mode(0)
        self.z_adjustment = (self.averaging_vision_y_pixel / (self.param_image_height/2)) * self.basic_z_ajustment /1000

        # Take the higest value between min and the calculated adjustment and keep the sign
        self.z_adjustment = self.z_adjustment if abs(self.z_adjustment) >= (self.z_adjustment/abs(self.z_adjustment)) * \
        self.minimum_z_adjustment else (self.z_adjustment/abs(self.z_adjustment)) * self.minimum_z_adjustment

        self.set_local_target (0.0,
                               0.0,
                               self.z_adjustment,
                               0.0,
                               0.0,
                               0.0)

    def align_yaw(self):
        self.yaw_adjustment = (self.averaging_vision_x_pixel / (self.param_object_real_width / 2)) * self.basic_yaw_ajustment

        # take the highest value between min and the calculated adjustment and keep the sign
        self.yaw_adjustment = self.yaw_adjustment if abs(self.yaw_adjustment) >= (self.yaw_adjustment /
            abs(self.yaw_adjustment)) * self.minimum_yaw_adjustment else (self.yaw_adjustment / abs(self.yaw_adjustment)) \
            * self.minimum_yaw_adjustment

        self.set_local_target(0.0,
                              0.0,
                              self.position.z,
                              0.0,
                              0.0,
                              self.yaw_adjustment)

    def get_outcomes(self):
        return ['succeeded', 'aborted', 'preempted']

    def end(self):
        self.vision_subscriber.unregister()
        self.target_reach_sub.unregister()


class VisionData:
    # Vision data container
    def __init__(self, x=0.0, y=0.0, height=0.0, width=0.0):
        self.x = x
        self.y = y
        self.height = height
        self.width = width


class BoundingBox:
    # Bounding box object
    def __init__(self, height, width):
        self.set_width(width)
        self.set_height(height)

    def is_inside(self, x, y):
        if -(self.width / 2) < x < (self.width / 2):
            if -(self.height / 2) < y < (self.height / 2):
                return True
        return False

    def set_width(self, new_width):
        self.width = new_width

    def set_height(self, new_height):
        self.height = new_height
