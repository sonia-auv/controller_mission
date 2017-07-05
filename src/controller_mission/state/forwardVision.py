import rospy

from ..mission_state import MissionState, Parameter
from proc_control.msg import TargetReached
from geometry_msgs.msg import Pose
from proc_image_processing.msg import VisionTarget


class ForwardVision(MissionState):

    def __init__(self):
        MissionState.__init__(self)
        self.buoy_is_unreached = False
        self.target_reached = False
        self.victory = False

        self.set_local_target_topic = None
        self.buoy_position = None
        self.target_reach_sub = None

        self.adjust_bounding_box = 0.0
        self.initial_bounding_box = 0.0

    def define_parameters(self):
        self.parameters.append(Parameter('param_color', 'red', 'color of object to align'))
        self.parameters.append(Parameter('param_distance_x', 1, 'Target'))
        self.parameters.append(Parameter('param_initial_bounding_box', 600, 'Initial bounding Box'))
        self.parameters.append(Parameter('param_final_bounding_box', 100, 'Final bounding Box'))
        self.parameters.append(Parameter('param_threshold_width', 100, 'maximum nb of pixel to align with heading'))
        self.parameters.append(Parameter('param_vision_target_width_in_meter', 0.23, 'Target'))
        self.parameters.append(Parameter('param_nb_pixel_to_victory', 300, 'minimal nb of pixel to ram'))
        self.parameters.append(Parameter('param_topic_to_listen', '/proc_image_processing/buoy_red', 'Name of topic to listen'))

    def get_outcomes(self):
        return ['succeeded', 'aborted', 'preempted']

    def vision_cb(self, vision_data):
        if self.param_color == vision_data.desc_1:

            bounding_box = self.adjust_bounding_box * vision_data.width + self.initial_bounding_box

            if abs(vision_data.x) >= bounding_box or abs(vision_data.y) >= bounding_box:
                self.buoy_is_unreached = True

            if vision_data.width >= self.param_nb_pixel_to_victory:
                self.victory = False

    def target_reach_cb(self, data):
        self.target_reached = data.target_is_reached

    def set_target(self, position_y):
        try:
            pose = Pose()
            pose.position.x = position_y
            pose.position.y = 0.0
            pose.position.z = 0.0
            pose.orientation.z = 0.0
            self.set_local_target_topic.publish(pose)
        except rospy.ROSException as exc:
            rospy.loginfo('Service did not process request: ' + str(exc))

        rospy.loginfo('Set relative position y = %f' % position_y)

    def initialize(self):
        self.set_local_target_topic = rospy.Publisher('/proc_control/set_target', Pose, queue_size=10)

        self.buoy_position = rospy.Subscriber(str(self.param_topic_to_listen), VisionTarget, self.vision_cb)

        self.target_reach_sub = rospy.Subscriber('/proc_control/target_reached', TargetReached, self.target_reach_cb)

        self.adjust_bounding_box = (self.param_final_bounding_box - self.param_initial_bounding_box) / (self.victory -
                                                                                            self.param_threshold_width)

        self.initial_bounding_box = self.param_initial_bounding_box - self.param_threshold_width * self.adjust_bounding_box

        self.buoy_is_unreached = False
        self.target_reached = False
        self.victory = False

        self.set_target(self.param_distance_x)

    def run(self, ud):
        if self.buoy_is_unreached or self.target_reached:
            self.set_target(0.0)
            return 'aborted'
        if self.victory:
            return 'succeeded'

    def end(self):
        self.buoy_position.unregister()
        self.target_reach_sub.unregister()
