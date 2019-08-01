import rospy

from ..mission_state import MissionState
from nav_msgs.msg import Odometry
import time

class SaveOrientation(MissionState):

    def __init__(self):
        MissionState.__init__(self)

        self.odom = None
        self.orientation = None

    def define_parameters(self):
        pass

    def get_outcomes(self):
        return ['succeeded', 'aborted', 'preempted']

    def odom_cb(self, odom_data):
        self.orientation = odom_data.pose.pose.orientation.z

    def initialize(self):
        self.odom = rospy.Subscriber('/proc_navigation/odom', Odometry, self.odom_cb)

    def run(self, ud):
        time.sleep(3)
        orientation = self.orientation
        rospy.set_param('/save_orientation',orientation)
        return 'succeeded'

    def end(self):
        self.odom.unregister()
