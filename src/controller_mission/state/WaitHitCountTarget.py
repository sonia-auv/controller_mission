import rospy
import math

from ..mission_state import MissionState, Parameter
from proc_image_processing.msg import VisionTarget
from proc_control.srv import SetDecoupledTarget


class WaitHitCountTarget(MissionState):

    def __init__(self):
        MissionState.__init__(self)
        self.set_local_target = None
        self.vision_subscriber = None
        self.set_local_target_topic = None

        self.last_vision_target = None
        self.nb_hit_count = 0

        self.target_reached = False

    def define_parameters(self):
        self.parameters.append(Parameter('param_topic_to_listen', '/proc_image_processing/buoy_red_result', 'Name of topic to listen'))
        self.parameters.append(Parameter('param_nb_hit_to_victory', 30, 'nb of hit to finish this state'))
        self.parameters.append(Parameter('param_max_distance_between_vision_target', 50, 'max distance in pixel to have a consecutive count'))

    def get_outcomes(self):
        return ['succeeded', 'aborted', 'preempted']

    def vision_cb(self, vision_data):
        if self.last_vision_target is None:
            self.last_vision_target = vision_data
            return

        distance = math.sqrt(((vision_data.x - self.last_vision_target.x) ** 2)+((vision_data.y - self.last_vision_target.y) ** 2))
        if distance <= self.param_max_distance_between_vision_target:
            self.nb_hit_count += 1
            rospy.loginfo('Hit count = %f at %s' % (self.nb_hit_count, vision_data.desc_2))
        else:
            self.nb_hit_count = 0
            rospy.loginfo('Lost count !!!!')
        self.last_vision_target = vision_data

    def set_target(self, position_y, position_z, position_yaw):
        try:
            self.set_local_target(0.0, position_y, position_z, 0.0, 0.0, position_yaw,
                                  False, False, True, True, True, False)
        except rospy.ServiceException as exc:
            rospy.loginfo('Service did not process request: ' + str(exc))

        rospy.loginfo('Set relative position y = %f' % position_y)
        rospy.loginfo('Set relative position z = %f' % position_z)
        rospy.loginfo('Set relative position yaw = %f' % position_yaw)

    def initialize(self):
        rospy.wait_for_service('/proc_control/set_local_decoupled_target')
        self.set_local_target = rospy.ServiceProxy('/proc_control/set_local_decoupled_target', SetDecoupledTarget)

        self.vision_subscriber = rospy.Subscriber(self.param_topic_to_listen, VisionTarget, self.vision_cb)
        self.last_vision_target = None
        self.nb_hit_count = 0

    def run(self, ud):
        if self.nb_hit_count >= self.param_nb_hit_to_victory:
            self.set_target(0.0, 0.0, 0.0)
            return 'succeeded'

    def end(self):
        self.vision_subscriber.unregister()

