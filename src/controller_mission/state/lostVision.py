import rospy
import time

from ..mission_state import MissionState, Parameter
from proc_image_processing.msg import VisionTarget


class LostVision(MissionState):

    def __init__(self):
        MissionState.__init__(self)
        self.vision_subscriber = None

        self.last_target_received_time = None

    def define_parameters(self):
        self.parameters.append(Parameter('param_topic_to_listen', '/proc_image_processing/buoy_red_result', 'Name of topic to listen'))
        self.parameters.append(Parameter('param_max_time_lost_in_sec', 10, 'Maximum time without target'))

    def get_outcomes(self):
        return ['succeeded', 'aborted', 'preempted']

    def vision_cb(self, vision_data):
        self.last_target_received_time = time.time()

    def initialize(self):
        self.vision_subscriber = rospy.Subscriber(self.param_topic_to_listen, VisionTarget, self.vision_cb)
        self.count += 1
        self.last_target_received_time = time.time()

    def run(self, ud):
        if time.time() - self.last_target_received_time > self.param_max_time_lost_in_sec:
            return 'succeeded'

    def end(self):
        self.vision_subscriber.unregister()
