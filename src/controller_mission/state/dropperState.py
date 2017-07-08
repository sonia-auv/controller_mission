import rospy

from ..mission_state import MissionState, Parameter


class Droppers(MissionState):

    def __init__(self):
        MissionState.__init__(self)
        self.start_time = None

    def define_parameters(self):
        self.parameters.append(Parameter('param_id', 1, 'Times Out'))

    def get_outcomes(self):
        return ['succeeded', 'aborted', 'preempted']

    def initialize(self):
        pass

    def run(self, ud):
        rospy.loginfo('Dropper : %f is launch' % self.param_id)
        return 'succeeded'

    def end(self):
        pass
