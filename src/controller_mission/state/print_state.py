import rospy

from ..mission_state import MissionState, Parameter

_author_ = 'Kevin Coombs'


class PrintState(MissionState):

    def __init__(self):
        MissionState.__init__(self)

    def define_parameters(self):
        self.parameters.append(Parameter('parameter_text','allo','Text to print'))

    def get_outcomes(self):
        return ['succeeded', 'aborted']

    def initialize(self):
        print self.parameter_text

    def run(self, ud):
        print self.parameter_text

