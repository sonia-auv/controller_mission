import rospy
import math as math

from Queue import deque
from ..mission_state import MissionState, Parameter
from proc_control.msg import TargetReached
from proc_control.srv import SetPositionTarget, SetDecoupledTarget
from proc_image_processing.msg import VisionTarget
from nav_msgs.msg import Odometry

class AlignBottomAlexFrank(MissionState):
    def __init__(self):
        MissionState.__init__(self)

        self.set_local_target = None
        self.vision_subscriber = None
        self.target_reach_sub = None
        self.set_local_target_topic = None

        # Average variables
        self.averaging_vision_x_pixel = 0.0
        self.averaging_vision_y_pixel = 0.0
        self.averaging_vision_width_pixel = 0.0
        self.averaging_vision_height_pixel = 0.0
        self.data_ready = False

        # List that contain last target distance and current target distance
        self.target_distance = {'last': 0.0, 'current': 0.0}
        self.target_reached = False
        self.moved_distance_from_vision = 0.0
        self.moved_distance_from_odom = 0.0

        # List to grab vision data for position x, y, width and height
        self.vision_data = None
        self.vision_position_y = 0
        self.vision_position_z = 0

        # Parameters to determine target distance.
        self.focal_size = 4.5  # mm
        self.sensor_height = 5.3  # mm
        self.sensor_width = 7.1  # mm

        self.count = 0

        # Control mode options.
        self.set_mode = None
        self.mode = SetControlModeRequest()
        self.mode_dic = {'0': self.mode.PositionModePID, '1': self.mode.PositionModePPI, '2': self.mode.VelocityModeB}
        self.is_moving = False

        # Position parameters.
        self.odom = None
        self.first_position = None
        self.position = None
        self.orientation = None

        # Bounding box
        self.y_bounding_box = None
        self.x_bounding_box = None

        # Variable for alignment
        self.x_adjustment = None
        self.basic_x_adjustment = 0.5
        self.minimum_x_adjustment = 0.1

        self.minimum_y_adjustment = 0.2
        self.y_adjustment = None
        self.basic_y_adjustment = 1.0

        self.yaw_adjustment = None
        self.minimum_yaw_adjustment = 3.0
        self.basic_yaw_adjustment = 10.0

        self.alex_frank_magic = 1.0


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
            self.parameters.append(Parameter('param_speed_x', 0.1, 'Speed to travel'))

        def initialize(self):
            rospy.wait_for_service('/proc_control/set_local_decoupled_target')
            self.set_local_target = rospy.ServiceProxy('/proc_control/set_local_decoupled_target', SetDecoupledTarget,
                                                       persistent=True)

            self.target_reach_sub = rospy.Subscriber('/proc_control/target_reached', TargetReached,
                                                     self.target_reach_cb)
            self.vision_subscriber = rospy.Subscriber(self.param_topic_to_listen, VisionTarget, self.vision_cb)
            self.vision_data = deque([], maxlen=self.param_max_queue_size)
            self.vision_data.clear()

            # Setup odometry service
            self.odom = rospy.Subscriber('/proc_navigation/odom', Odometry, self.odom_cb)
            self.get_first_position()

            # Setup bounding boxes
            self.x_bounding_box = BoundingBox(self.param_image_height, self.param_image_width * 0.01)
            self.y_bounding_box = BoundingBox(self.param_image_height * 1, self.param_image_width)

        def run(self):
            pass

        def vision_cb(self, data):
            self.vision_data.append(VisionData(data.x, data.y, data.height, data.width))

            if len(self.vision_data) == self.param_max_queue_size:
                self.parse_vision_data()

        def is_align_y(self):
            if self.y_bounding_box.is_inside(self.averaging_vision_x_pixel, self.averaging_vision_y_pixel):
                return True
            return False

        def is_align_x(self):
            if self.x_bounding_box.is_inside(self.averaging_vision_x_pixel, self.averaging_vision_y_pixel):
                return True
            return False

        def align_submarine(self):
            rospy.loginfo('Align number %i' % self.count)
            # self.count += 1
            if not self.is_align_y():
                rospy.loginfo('Y alignment.')
                if self.target_reached:
                    self.align_y()
            if not self.is_align_x():
                rospy.loginfo('X alignment.')
                if self.target_reached:
                    self.align_x()


        def align_x(self):
            self.z_adjustment = (self.averaging_vision_y_pixel / (self.param_image_height / 2)) * self.basic_y_adjustment
            rospy.loginfo('Y adjustment: ' + str(self.y_adjustment))
            # Take the highest value between min and the calculated adjustment and keep the sign
            self.y_adjustment = self.y_adjustment if abs(self.y_adjustment) >= (
                        self.y_adjustment / abs(self.y_adjustment)) * self.minimum_y_adjustment else (self.y_adjustment/\
                        abs(self.y_adjustment)) * self.minimum_y_adjustment
            rospy.loginfo('New y adjustment: ' + str(self.y_adjustment))
            self.set_local_target(0.0,self.y_adjustment,0.0, 0.0, 0.0, 0.0, False, False, True, False, False, False)

        def align_y(self):
            self.x_adjustment = (self.averaging_vision_x_pixel / (self.param_image_width / 2)) * self.basic_x_adjustment
            rospy.loginfo('X adjustment: ' + str(self.x_adjustment))
            # Take the highest value between min and the calculated adjustment and keep the sign
            self.x_adjustment = self.x_adjustment if abs(self.x_adjustment) >= (
                        self.x_adjustment / abs(self.x_adjustment)) * self.minimum_x_adjustment else (self.x_adjustment/\
                        abs(self.x_adjustment)) * self.minimum_x_adjustment
            rospy.loginfo('New x adjustment: ' + str(self.x_adjustment))
            self.set_local_target(self.x_adjustment, 0.0,0.0, 0.0, 0.0, 0.0, False, False, True, False, False, False)

        def target_reach_cb(self, data):
            self.target_reached = data.target_is_reached

        def odom_cb(self, data):
            if not self.first_position:
                self.first_position = data.pose.pose.position
            self.position = data.pose.pose.position
            self.orientation = data.pose.pose.orientation

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