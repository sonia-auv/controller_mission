import rospy

from ..mission_state import MissionState
from nav_msgs.msg import Odometry


class SavePosition(MissionState):

    def __init__(self):
        MissionState.__init__(self)

        self.odom =None
        self.position = None

    def define_parameters(self):
        pass

    def get_outcomes(self):
        return ['succeeded', 'aborted', 'preempted']

    def odom_cb(self, odom_data):
        self.position = odom_data.pose.pose

    def initialize(self):
        self.odom = rospy.Subscriber('/proc_navigation/odom', Odometry, self.odom_cb)

    def run(self, ud):
        position = self.position
        if position is not None:
            ud.generic_data_field_1 = position
            return 'succeeded'

    def end(self):
        self.odom.unregister()
