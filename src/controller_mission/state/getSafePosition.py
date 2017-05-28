import rospy
from numpy import uint8
import threading

from ..mission_state import MissionState, Parameter
from proc_mapping.msg import KeyValueIdPose, PoseSavedRequest


class GetSafePosition(MissionState):

    def __init__(self):
        MissionState.__init__(self)
        self.pub_request_position = None
        self.sub_get_pose = None
        self.pose = None
        self.msg = KeyValueIdPose()
        self.pose_is_not_get = False
        self.count = 0

    def define_parameters(self):
        self.parameters.append(Parameter('param_position_id', 1, 'Position ID'))

    def get_outcomes(self):
        return ['succeeded', 'aborted']

    def request_pose(self):
        rate = rospy.Rate(1)
        while self.pose_is_not_get:
            self.pub_request_position.publish(uint8(self.param_position_id))
            rate.sleep()

    def get_pose_callback(self, data):
        self.pose = data

    def initialize(self):
        self.pub_request_position = rospy.Publisher('/proc_mapping/GetSavedPoseRequest', PoseSavedRequest, queue_size=100)

        self.sub_get_pose = rospy.Subscriber('/proc_mapping/GetSavedPoseResponse', KeyValueIdPose, self.get_pose_callback)

        self.pose_is_not_get = True
        self.thread_request_pose = threading.Thread(target=self.request_pose)
        self.thread_request_pose.setDaemon(1)
        self.thread_request_pose.start()

    def run(self, ud):
        position = self.pose
        if position is not None:
            self.pose_is_not_get = False
            ud.generic_data_field_1 = position  # Pose
            ud.generic_data_field_2 = 'local'
            return 'succeeded'

    def end(self):
        self.sub_get_pose.unregister()
