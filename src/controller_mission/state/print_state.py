import rospy

from ..mission_state import MissionState, Parameter
from proc_control.msg import TargetReached
from proc_control.srv import SetPositionTarget

_author_ = 'Kevin Coombs'


class PrintState(MissionState):

    def __init__(self):
        MissionState.__init__(self)

    def define_parameters(self):
        self.parameters.append(Parameter('parameter_text','allo','Text to print'))

    def get_outcomes(self):
        return ['succeeded', 'aborted']

    def target_reach_cb(self, data):
        self.target_reached = data.target_is_reached

    def initialize(self):

        print self.parameter_text

    def run(self, ud):
        print self.parameter_text

    def end(self):
        self.target_reach_sub.unregister()
