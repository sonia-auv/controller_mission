import rospy

from ..mission_state import MissionState, Parameter


class TimesOut(MissionState):

    def __init__(self):
        MissionState.__init__(self)
        self.start_time = None

    def define_parameters(self):
        self.parameters.append(Parameter('time', '1', 'Times Out'))

    def get_outcomes(self):
        return ['succeeded', 'aborted']

    def initialize(self):
        self.start_time = rospy.get_time()

    def run(self, ud):
        if (rospy.get_time() - self.start_time) >= self.time:
            return 'succeeded'

    def end(self):
        pass
