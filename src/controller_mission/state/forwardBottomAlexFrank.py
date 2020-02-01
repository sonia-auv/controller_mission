import rospy

from ..mission_state import MissionState, Parameter
from proc_control.msg import TargetReached
from proc_control.srv import SetPositionTarget, SetDecoupledTarget
from proc_image_processing.msg import VisionTarget
from nav_msgs.msg import Odometry

"""
The vision part of this code consider use the normal axis for image parsing. 
(e.g X --> Horizontal
     Y --> Vertical)

The control part use the normal axis of the sub.
"""


class ForwardBottomAlexFrank(MissionState):

    def __init__(self):
        MissionState.__init__(self)
        self.vision_is_unreached = False
        self.target_reached = False
        self.victory = False

        self.set_local_target = None
        self.buoy_position = None
        self.target_reach_sub = None
        self.sub_position = None

        # Average variables
        self.averaging_vision_x_pixel = 0.0
        self.averaging_vision_y_pixel = 0.0
        self.averaging_vision_width_pixel = 0.0
        self.averaging_vision_height_pixel = 0.0
        self.data_ready = False
        self.vision_data = None

        # List that contain last target distance and current target distance
        self.target_distance = {'last': 0.0, 'current': 0.0}
        self.moved_distance_from_vision = 0.0
        self.moved_distance_from_odom = 0.0

        # Position parameters.
        self.odom = None
        self.first_position = None
        self.position = None
        self.orientation = None
        self.position_in_z = None

        # Parameters to determine target distance.
        self.focal_size = 4.5  # mm
        self.sensor_height = 5.3  # mm
        self.sensor_width = 7.1  # mm

        # Bounding box
        self.bounding_box = None

    def define_parameters(self):
        self.parameters.append(Parameter('param_distance_z', 0.3, 'Target distance (each iterations)'))
        self.parameters.append(Parameter('param_distance_from_target', 1.0, 'Target distance to reach the target'))
        self.parameters.append(Parameter('param_bounding_box', 200, 'Bounding box size (side)'))
        self.parameters.append(Parameter('param_distance_max_z', 2.5, 'Distance max allowed from surface.'))
        self.parameters.append(Parameter('param_nb_pixel_to_victory', 200, 'minimal nb of pixel to ram'))
        self.parameters.append(Parameter('param_max_queue_size', 10, 'Maximum size of queue'))
        self.parameters.append(Parameter('param_topic_to_listen', '/proc_image_processing/bin_result', 'Name of topic to listen'))

    def initialize(self):
        rospy.wait_for_service('/proc_control/set_local_decoupled_target')
        # Subscribe to the
        self.set_local_target = rospy.ServiceProxy('/proc_control/set_local_decoupled_target', SetDecoupledTarget)
        self.buoy_position = rospy.Subscriber(str(self.param_topic_to_listen), VisionTarget, self.vision_cb)
        self.target_reach_sub = rospy.Subscriber('/proc_control/target_reached', TargetReached, self.target_reach_cb)
        self.sub_position = rospy.Subscriber('/proc_navigation/odom', Odometry, self.sub_position_cb)

        # Setup odometry service
        self.odom = rospy.Subscriber('/proc_navigation/odom', Odometry, self.odom_cb)
        self.get_first_position()

        self.set_target(self.param_distance_z)
        self.bounding_box = BoundingBox(self.param_bounding_box, self.param_bounding_box, 0, 0)

    def run(self, ud):
        if self.position_in_z >= self.param_distance_max_z or self.is_victory():
            self.set_target(0.0)
            return 'succeeded'

        if self.position_in_z is not None and self.target_reached:
            if self.param_distance_z + self.position_in_z >= self.param_distance_max_z:
                self.set_target(self.param_distance_max_z - self.position_in_z)
            else:
                self.set_target(self.param_distance_z)

        if self.vision_is_unreached or self.target_reached:
            self.set_target(0.0)
            return 'aborted'

    def get_outcomes(self):
        return ['succeeded', 'aborted', 'preempted']

    def sub_position_cb(self, position_data):
        if not self.first_position:
            self.first_position = position_data.pose.pose.position
        self.position_in_z = position_data.pose.pose.position.z
        self.orientation = position_data.pose.pose.orientation

    def is_victory(self):
        if self.target_distance["current"] <= self.param_distance_from_target and self.target_distance["current"] != 0.0:
            self.victory = True

    def vision_cb(self, data):
        # Call to parse the data after seeing this object n times.
        self.vision_data.append(VisionData(data.x, data.y, data.height, data.width))
        if len(self.vision_data) == self.param_max_queue_size:
            self.parse_vision_data()

        # if data.width >= self.param_nb_pixel_to_victory:
        #     self.victory = True
        # Check if the data is inside the bounding box.
        if not self.bounding_box.isInside(data.x, data.y):
            self.vision_is_unreached = True

        # if abs(data.x) >= self.param_bounding_box or abs(data.y) >= self.param_bounding_box:
        #    self.vision_is_unreached = True

    def target_reach_cb(self, data):
        self.target_reached = data.target_is_reached

    def set_target(self, position_z):
        try:
            self.set_local_target(0.0, 0.0, position_z, 0.0, 0.0, 0.0,
                                  True, True, False, True, True, True)
        except rospy.ServiceException as exc:
            rospy.loginfo('Service did not process request: ' + str(exc))

        rospy.loginfo('Set relative position z = %f' % position_z)

    def get_target_distance(self):
        return ((self.focal_size * self.param_object_real_height * self.param_image_height) / \
                (self.averaging_vision_height_pixel * self.sensor_height)) / 1000

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

        self.target_distance['last'] = self.target_distance['current']
        self.target_distance['current'] = self.get_target_distance()

        # self.refresh_vision_timer()
        # self.align_submarine()
        # self.get_distance_error()

        # Show the average
        rospy.loginfo('--------------------------------------------------')
        rospy.loginfo('Position x : %f' % self.averaging_vision_x_pixel)
        rospy.loginfo('Position y : %f' % self.averaging_vision_y_pixel)
        rospy.loginfo('Height of the object : %f' % self.averaging_vision_width_pixel)
        rospy.loginfo('Width of the object : %f' % self.averaging_vision_height_pixel)
        rospy.loginfo('Target distance : %f' % self.target_distance['current'])
        rospy.loginfo('Moved distance (vision): %f' % self.moved_distance_from_vision)
        rospy.loginfo('Moved distance (odom): %f' % self.moved_distance_from_odom)

    def end(self):
        self.buoy_position.unregister()
        self.target_reach_sub.unregister()

    def get_first_position(self):
        while not self.first_position:
            pass
        return


class VisionData:
    # Vision data container
    def __init__(self, x=0.0, y=0.0, height=0.0, width=0.0):
        self.x = x
        self.y = y
        self.height = height
        self.width = width


class BoundingBox:
    # Bounding box object
    def __init__(self, height, width, center_x, center_y):
        self.set_width(width)
        self.set_height(height)
        self.center_x = center_x
        self.center_y = center_y

    def is_inside(self, x, y):
        rospy.loginfo('testing')
        rospy.loginfo('Bounding Box X -> width:{0} height:{1}'.format(self.width, self.height))
        rospy.loginfo('Bounding Box center X :{0} center Y: {1}'.format(self.center_x, self.center_y))
        if (self.center_x - (self.width / 2)) < x < (self.center_x + (self.width / 2)):
            rospy.loginfo('inside x')
            if (self.center_y - (self.height / 2)) < y < (self.center_y + (self.height / 2)):
                rospy.loginfo('is inside xy')
                return True
        return False

    def set_width(self, new_width):
        self.width = new_width

    def set_height(self, new_height):
        self.height = new_height
