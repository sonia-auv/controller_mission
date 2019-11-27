import rospy

from Queue import deque
from ..mission_state import MissionState, Parameter
from proc_control.msg import TargetReached
from proc_control.srv import SetPositionTarget, SetDecoupledTarget
from proc_image_processing.msg import VisionTarget

class AlignToVisionTest(MissionState):
    def __init__(self):
        MissionState.__init__(self)
        self.set_local_target = None
        self.vision_subscriber = None
        self.target_reach_sub = None
        self.set_local_target_topic = None

        self.vision_position_y = 0
        self.vision_position_z = 0

        self.heading = 0

        self.averaging_vision_x_pixel = 0.0
        self.averaging_vision_y_pixel = 0.0

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

        #List to grab vision data for position x and y.
        self.vision_x_pixel = None
        self.vision_y_pixel = None
        self.vision_width_pixel = None
        self.vision_height_pixel = None

        #Parameters to determine target distance.
        self.focal_size = 50 #mm
        self.sensor_height = 10 #mm

        self.count = 0

    def define_parameters(self):
        self.parameters.append(Parameter('param_bounding_box', 200, 'bounding box in pixel'))
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