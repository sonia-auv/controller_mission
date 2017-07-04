import rospy

from Queue import deque
from ..mission_state import MissionState, Parameter
from proc_control.srv import SetPositionTarget
from proc_control.msg import TargetReached
from proc_image_processing.msg import VisionTarget


class ConfirmBuoy(MissionState):

    def __init__(self):
        MissionState.__init__(self)
        self.vision_subscriber = None

        self.nb_of_hit = 0

    def define_parameters(self):
        self.parameters.append(Parameter('param_color', 'green', 'color of object to align'))
        self.parameters.append(Parameter('param_topic_to_listen', '/proc_image_processing/buoy_red', 'Name of topic to listen'))
        self.parameters.append(Parameter('param_nb_of_hit_min', 10, 'Maximum number of alignment'))

    def get_outcomes(self):
        return ['succeeded', 'aborted', 'preempted']

    def vision_cb(self, vision_data):
        if self.param_color == vision_data.desc_1:
            self.nb_of_hit += 1

    def initialize(self):
        self.vision_subscriber = rospy.Subscriber(str(self.param_topic_to_listen), VisionTarget, self.vision_cb)

    def run(self, ud):
        if self.nb_of_hit >= self.param_nb_of_hit_min:
            return 'succeeded'

    def end(self):
        self.vision_subscriber.unregister()
