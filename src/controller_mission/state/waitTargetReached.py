import rospy
import math

from Queue import deque
from ..mission_state import MissionState, Parameter
from proc_control.srv import SetPositionTarget
from proc_control.msg import TargetReached
from nav_msgs.msg import Odometry


class WaitTargetReached(MissionState):

    def __init__(self):
        MissionState.__init__(self)

    def define_parameters(self):
        pass
        ##self.parameters.append(Parameter('param_queu_size', 10, 'Maximum size of queue'))

    def get_outcomes(self):
        return ['succeeded', 'aborted', 'preempted']

    def target_reach_cb(self, data):
        self.target_reached = data.target_is_reached

    def initialize(self):
        self.target_reached = 0
        self.target_reach_sub = rospy.Subscriber('/proc_control/target_reached', TargetReached, self.target_reach_cb)
        # rospy.wait_for_service('/proc_control/set_global_target')
        # self.set_global_target = rospy.ServiceProxy('/proc_control/set_global_target', SetPositionTarget)

        # rospy.wait_for_service('/proc_mapping/pinger_location_service')
        # self.pinger_location_service = rospy.ServiceProxy('/proc_mapping/pinger_location_service', PingerLocationService)

    def run(self, ud):
        if self.target_reached > 0:
            return 'succeeded'
        
    def end(self):
        pass