import rospy
import math

from ..mission_state import MissionState, Parameter
from proc_image_processing.msg import VisionTarget
from proc_control.srv import SetPositionTarget


class WaitHitCountTargetForRandomPinger(MissionState):

    def __init__(self):
        MissionState.__init__(self)
        self.set_local_target = None
        self.vision_subscriber = None
        self.vision_deepLearning = None
        self.set_local_target_topic = None
        self.deep_learning_present = False
        self.last_vision_target = None
        self.nb_hit_count = 0

        self.target_reached = False

    def define_parameters(self):
        self.parameters.append(Parameter('param_topic_to_listen_DeepLearning', '/proc_image_processing/find_bin_result', 'topic of deep learning'))
        self.parameters.append(Parameter('param_topic_to_listen', '/proc_image_processing/align_roulette_result', 'Name of topic to listen'))
        self.parameters.append(Parameter('param_nb_hit_to_victory', 30, 'nb of hit to finish this state'))
        self.parameters.append(Parameter('param_max_distance_between_vision_target', 50, 'max distance in pixel to have a consecutive count'))


    def get_outcomes(self):
        return ['succeeded', 'aborted', 'preempted']

    def vision_cb(self, vision_data):
        if self.last_vision_target is None:
            self.last_vision_target = vision_data
            return
        self.decision_mission_pinger(vision_data)

    def vision_deep_learning_cb(self, vision_data):
        self.deep_learning_present = True

    def decision_mission_pinger(self, vision_data):
        distance = math.sqrt(((vision_data.x - self.last_vision_target.x) ** 2)+((vision_data.y - self.last_vision_target.y) ** 2))
        
        if self.deep_learning_present:
            self.nb_hit_count += -10
        else:
            if distance <= self.param_max_distance_between_vision_target:
                self.nb_hit_count += 1
                rospy.loginfo('Hit count = %f' % self.nb_hit_count)
            else:
                self.nb_hit_count = min(self.nb_hit_count, 0)
                rospy.loginfo('Lost count !!!!')
        self.last_vision_target = vision_data

        self.deep_learning_present = False

    def set_target(self, position_y, position_z, position_yaw):
        try:
            self.set_local_target(0.0,
                                  position_y,
                                  position_z,
                                  0.0,
                                  0.0,
                                  position_yaw)
        except rospy.ServiceException as exc:
            rospy.loginfo('Service did not process request: ' + str(exc))

        rospy.loginfo('Set relative position y = %f' % position_y)
        rospy.loginfo('Set relative position z = %f' % position_z)
        rospy.loginfo('Set relative position yaw = %f' % position_yaw)

    def initialize(self):
        rospy.wait_for_service('/proc_control/set_local_target')
        self.set_local_target = rospy.ServiceProxy('/proc_control/set_local_target', SetPositionTarget)

        self.vision_subscriber = rospy.Subscriber(self.param_topic_to_listen, VisionTarget, self.vision_cb)
        self.vision_deepLearning = rospy.Subscriber(self.param_topic_to_listen_DeepLearning, VisionTarget, self.vision_deep_learning_cb)
        self.last_vision_target = None
        self.nb_hit_count = 0
        self.deep_learning_present = False

    def run(self, ud):
        if self.nb_hit_count >= self.param_nb_hit_to_victory:
            self.set_target(0.0, 0.0, 0.0)
            return 'succeeded'

    def end(self):
        self.vision_subscriber.unregister()

