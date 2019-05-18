import rospy

from ..mission_state import MissionState, Parameter

from Queue import deque
from proc_control.srv import SetPositionTarget, SetDecoupledTarget
from proc_control.msg import TargetReached
from proc_image_processing.msg import VisionTarget


class AlignXYVision(MissionState):

    def __init__(self):
        MissionState.__init__(self)
        self.vision_subscriber = None
        self.target_reach_sub = None
        self.set_local_target = None

        self.vision_x_pixel = None
        self.vision_y_pixel = None

        self.target_reached = False

        self.averaging_vision_x_pixel = 0.0
        self.averaging_vision_y_pixel = 0.0

        self.vision_position_x = 0.0
        self.vision_position_y = 0.0

        self.vision_is_reach_x = False
        self.vision_is_reach_y = False
        self.submarine_is_align = False

    def define_parameters(self):
        self.parameters.append(Parameter('param_bounding_box', 100, 'bounding box in pixel'))
        self.parameters.append(Parameter('param_vision_target_width_in_meter', 0.15, 'transform pixel to meter'))
        self.parameters.append(Parameter('param_vision_target_height_in_meter', 1.2, 'transform pixel to meter'))
        self.parameters.append(Parameter('param_topic_to_listen', '/proc_image_processing/align_path_result', 'Name of topic to listen'))
        self.parameters.append(Parameter('param_max_queue_size', 10, 'Maximum size of queue'))
        self.parameters.append(Parameter('param_control_bounding_box_in_y', 0.5, 'Control bounding box in y'))

    def get_outcomes(self):
        return ['succeeded', 'aborted', 'preempted']

    def target_reach_cb(self, data):
        self.target_reached = data.target_is_reached

    def find_y_pos_to_matches_to_control_bounding_box(self, pos_y):
        if pos_y <= self.param_control_bounding_box_in_y:
            return self.param_control_bounding_box_in_y + 0.1
        else:
            return pos_y

    def vision_subscriber_cb(self, vision_data):
        self.vision_x_pixel.append(vision_data.x)
        self.vision_y_pixel.append(vision_data.y)

        if len(self.vision_x_pixel) == self.param_max_queue_size and \
           len(self.vision_y_pixel) == self.param_max_queue_size:

            self.parse_vision_data(vision_data.width, vision_data.height)

    def parse_vision_data(self, width, height):
        for i in self.vision_x_pixel:
            self.averaging_vision_x_pixel += i

        for i in self.vision_y_pixel:
            self.averaging_vision_y_pixel += i

        self.averaging_vision_x_pixel /= len(self.vision_x_pixel)
        self.averaging_vision_y_pixel /= len(self.vision_y_pixel)

        if abs(self.averaging_vision_y_pixel) <= self.param_bounding_box:
            self.vision_is_reach_x = True
        else:
            self.vision_is_reach_x = False

        if abs(self.averaging_vision_x_pixel) <= self.param_bounding_box:
            self.vision_is_reach_y = True
        else:
            self.vision_is_reach_y = False

        if self.target_reached:
            pixel_to_meter = (width / self.param_vision_target_width_in_meter +
                              height / self.param_vision_target_height_in_meter) / 2

            self.vision_position_x = self.averaging_vision_y_pixel / pixel_to_meter
            self.vision_position_y = self.averaging_vision_x_pixel / pixel_to_meter

            self.align_path()

    def align_path(self):
        if self.vision_position_y == 0 or self.vision_position_x == 0:
            return
        vision_position_y = self.find_y_pos_to_matches_to_control_bounding_box(self.vision_position_y)
        vision_position_x = self.find_y_pos_to_matches_to_control_bounding_box(self.vision_position_x)

        vision_position_y = vision_position_y * (self.vision_position_y / abs(self.vision_position_y))
        vision_position_x = vision_position_x * (self.vision_position_x / abs(self.vision_position_x))

        if not self.submarine_is_align:
            self.set_target(vision_position_x, vision_position_y)

    def set_target(self, position_x, position_y):
        try:
            self.set_local_target(position_x, position_y, 0, 0, 0, 0,
                                  False, False, True, True, True, True)
        except rospy.ServiceException as exc:
            rospy.loginfo('Service did not process request: ' + str(exc))

        rospy.loginfo('Set relative position x = %f' % position_x)
        rospy.loginfo('Set relative position y = %f' % position_y)

    def initialize(self):
        rospy.wait_for_service('/proc_control/set_local_decoupled_target')
        self.set_local_target = rospy.ServiceProxy('/proc_control/set_local_decoupled_target', SetDecoupledTarget, persistent= True)

        self.target_reach_sub = rospy.Subscriber('/proc_control/target_reached', TargetReached, self.target_reach_cb)

        self.vision_subscriber = rospy.Subscriber(self.param_topic_to_listen, VisionTarget, self.vision_subscriber_cb)

        self.vision_x_pixel = deque([], maxlen=self.param_max_queue_size)
        self.vision_y_pixel = deque([], maxlen=self.param_max_queue_size)

        self.vision_x_pixel.clear()
        self.vision_y_pixel.clear()

        self.vision_is_reach_x = False
        self.vision_is_reach_y = False

        self.submarine_is_align = False

    def run(self, ud):
        self.submarine_is_align = self.vision_is_reach_x and self.vision_is_reach_y
        if self.submarine_is_align:
            self.set_target(0.0, 0.0)
            return 'succeeded'

    def end(self):
        self.vision_subscriber.unregister()
        self.target_reach_sub.unregister()
