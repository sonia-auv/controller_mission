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
                             outcomes=self.get_outcomes(),
                             input_keys=['generic_data_field_1','generic_data_field_2','generic_data_field_3','generic_data_field_4','generic_data_field_5','generic_data_field_6'],
                             output_keys=['generic_data_field_1','generic_data_field_2','generic_data_field_3','generic_data_field_4','generic_data_field_5','generic_data_field_6'])

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
            if self.preempt_requested():
                self.service_preempt()
                return 'preempted'
            result = self.run(ud)
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
