import rospy
import smach


class UiInfo:
    def __init__(self):
        self.pos_x, self.pox_y = 0, 0
        self.state_name = ''


class Parameter:
    def __init__(self, variable_name, value, description):
        self.variable_name, self.value, self.description = variable_name, value, description


class MissionState(smach.State):
    parameters = []
    name = ''

    def __init__(self):
        smach.State.__init__(self,
                             outcomes=self.get_outcomes())

        self.define_parameters()
        self.declare_parameters()

    def define_parameters(self):
        # self.parameters.append(Parameter('Distance',2,'distance to travel'))
        None

    def declare_parameters(self):
        for param in self.parameters:
            self.__dict__[param.variable_name] = param.value

    def execute(self, ud):
        self.initialize()
        rate = rospy.Rate(50)
        while not rospy.is_shutdown():
            result = self.run(ud)
            print result
            if result:
                self.end()
                return result
            rate.sleep()

    def end(self):
        pass

    def initialize(self):
        raise NotImplementedError

    def run(self, ud):
        raise NotImplementedError

    def get_outcomes(self):
        return ['succeeded', 'aborted']
