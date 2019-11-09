import rospy

from ..mission_state import MissionState, Parameter


class TimesOut(MissionState):

    def __init__(self):
        MissionState.__init__(self)
        self.start_time = None
        self.current_second = 0

    def define_parameters(self):
        self.parameters.append(Parameter('param_time', 1, 'Times Out'))
        self.parameters.append(Parameter('param_to_return', 'succeeded', 'Times Out'))

    def get_outcomes(self):
        return ['succeeded', 'aborted', 'preempted']

    def initialize(self):
        self.start_time = rospy.get_time()
        rospy.loginfo("Asked time : {} sec".format(self.param_time))

    def run(self, ud):
        time = rospy.get_time() - self.start_time

        int_time = int(time)

        if int_time > self.current_second:
            self.current_second = int_time

            rospy.loginfo("Time left : {} sec".format(float(self.param_time) - int_time))

        self.current_second = int(time)

        if time >= self.param_time:
            return str(self.param_to_return)

    def end(self):
        pass
