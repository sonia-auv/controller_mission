import rospy
import math

from ..mission_state import MissionState, Parameter
from geometry_msgs.msg import Pose
from proc_image_processing.msg import VisionTarget


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

        distance = math.sqrt(((vision_data.x - self.last_vision_target.x) ** 2) + ((vision_data.y - self.last_vision_target.y) ** 2))

        if distance <= self.param_max_distance_between_vision_target:
            self.nb_hit_count += 1
            rospy.logdebug("Hit count =" + self.nb_hit_count)
        else:
            self.nb_hit_count = 0
            rospy.logdebug('Hit count lost !!!')

        self.last_vision_target = vision_data

    def set_target(self, position_y, position_z, position_yaw):
        try:
            pose = Pose()
            pose.position.x = 0.0
            pose.position.y = position_y
            pose.position.z = position_z
            pose.orientation.z = position_yaw
            self.set_local_target_topic.publish(pose)
        except rospy.ROSException as exc:
            rospy.loginfo('Service did not process request: ' + str(exc))

        rospy.loginfo('Set relative position y = %f' % position_y)
        rospy.loginfo('Set relative position z = %f' % position_z)
        rospy.loginfo('Set relative position yaw = %f' % position_yaw)

    def initialize(self):
        self.set_local_target_topic = rospy.Publisher('/proc_control/set_target', Pose, queue_size=10)
        self.vision_subscriber = rospy.Subscriber(self.param_topic_to_listen, VisionTarget, self.vision_cb)
        self.last_vision_target = None
        self.nb_hit_count = 0

    def run(self, ud):
        if self.nb_hit_count >= self.param_nb_hit_to_victory:
            self.set_target(0.0, 0.0, 0.0)
            return 'succeeded'

    def end(self):
        self.vision_subscriber.unregister()
        self.target_reach_sub.unregister()
