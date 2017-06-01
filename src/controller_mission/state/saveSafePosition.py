import rospy
from numpy import uint8

from ..mission_state import MissionState, Parameter
from proc_mapping.msg import KeyValueIdPose
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose


class SaveSafePosition(MissionState):

    def __init__(self):
        MissionState.__init__(self)
        self.pub_safe_position = None
        self.sub_act_pose = None
        self.act_pose = Pose()
        self.msg = KeyValueIdPose()
        self.count = 0

    def define_parameters(self):
        self.parameters.append(Parameter('param_position_id', 1, 'Position ID'))

    def get_outcomes(self):
        return ['succeeded', 'aborted']

    def pose_callback(self, data):
        self.act_pose = data.pose.pose

    def initialize(self):
        self.pub_safe_position = rospy.Publisher('/proc_mapping/SavePose', KeyValueIdPose, queue_size=100)

        self.sub_act_pose = rospy.Subscriber('/proc_navigation/odom', Odometry, self.pose_callback)

    def run(self, ud):
        self.msg.id = uint8(self.param_position_id)
        self.msg.pose = self.act_pose
        self.pub_safe_position.publish(self.msg)

        self.count += 1

        if self.count == 20:
            return 'succeeded'

    def end(self):
        self.sub_act_pose.unregister()
