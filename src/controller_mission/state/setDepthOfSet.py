import rospy

from ..mission_state import MissionState, Parameter
from proc_navigation.srv import SetDepthOffset


class SetDepthOfSet(MissionState):

    def __init__(self):
        MissionState.__init__(self)
        self.set_initial_depth = None

    def define_parameters(self):
        pass

    def get_outcomes(self):
        return ['succeeded', 'aborted', 'preempted']

    def initialize(self):
        rospy.wait_for_service('/proc_navigation/set_depth_offset', timeout=2)
        self.set_initial_depth = rospy.ServiceProxy('/proc_navigation/set_depth_offset', SetDepthOffset)
        try:
            self.set_initial_depth()
        except rospy.ServiceException as err:
            rospy.logerr(err)

    def run(self, ud):
        return 'succeeded'

    def end(self):
        pass
