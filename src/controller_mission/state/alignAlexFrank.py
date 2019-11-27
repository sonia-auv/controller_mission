import rospy

from Queue import deque
from ..mission_state import MissionState, Parameter
from proc_control.msg import TargetReached
from proc_control.srv import SetPositionTarget, SetDecoupledTarget
from proc_image_processing.msg import VisionTarget
from nav_msgs.msg import Odometry


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

    def run(self, ud):
        self.vision_is_reach = self.vision_is_reach_y and self.vision_is_reach_z

        if self.victory and self.vision_is_reach:
            rospy.loginfo('Vision is Reach : %s', str(self.vision_is_reach))
            rospy.loginfo('Pixel Vision in x : %f', self.averaging_vision_x_pixel)
            rospy.loginfo('Pixel Vision in y : %f', self.averaging_vision_y_pixel)
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

    def is_align(self):
        pass

    def get_target_distance(self):
        return (self.focal_size * self.param_object_real_height * self.param_image_height) / (
                    self.averaging_vision_height_pixel * self.sensor_height)

    def align_submarine(self):
        rospy.loginfo('Align number %i' % self.count)
        self.count += 1

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
    def __init__(self):
        pass
