import rospy
import math

from ..mission_state import MissionState, Parameter
from proc_image_processing.msg import VisionTarget


class WaitHitCountTarget(MissionState):

    def __init__(self):
        MissionState.__init__(self)
        self.set_local_target = None
        self.vision_subscriber = None
        self.set_local_target_topic = None

        self.time = None

        self.last_vision_target = None
        self.nb_hit_count = 0

        self.target_reached = False

    def define_parameters(self):
        self.parameters.append(Parameter('param_topic_to_listen', '/proc_image_processing/buoy_red_result', 'Name of topic to listen'))
        self.parameters.append(Parameter('param_nb_hit_to_fail', 30, 'nb of hit to finish this state'))
        self.parameters.append(Parameter('param_nb_', 50, 'max distance in pixel to have a consecutive count'))

    def get_outcomes(self):
        return ['succeeded', 'aborted', 'preempted']

    def vision_cb(self, vision_data):
        pass

    def initialize(self):
        self.vision_subscriber = rospy.Subscriber(self.param_topic_to_listen, VisionTarget, self.vision_cb)
        self.last_vision_target = None
        self.nb_hit_count = 0
        self.time = None

    def run(self, ud):
        if self.nb_hit_count >= self.param_nb_hit_to_fail:
            self.set_target(0.0, 0.0, 0.0)
            return 'succeeded'

    def end(self):
        self.vision_subscriber.unregister()

