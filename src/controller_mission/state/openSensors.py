import rospy

from ..mission_state import MissionState, Parameter
from proc_mapping.srv import Activation


class OpenSensors(MissionState):

    def __init__(self):
        MissionState.__init__(self)
        self.sensor_state = []
        self.sensor_type = []
        self.msg_is_publish = False

    def define_parameters(self):
        self.parameters.append(Parameter('param_camera_on', True, 'Open camera'))
        self.parameters.append(Parameter('param_sonar_on', True, 'Open sonar'))
        self.parameters.append(Parameter('param_hydro_on', True, 'Open hydrophone'))

    def get_outcomes(self):
        return ['succeeded', 'aborted']

    def initialize(self):
        rospy.wait_for_service('/proc_mapping/provider_activation_request')
        self.open_sensor = rospy.ServiceProxy("/proc_mapping/provider_activation_request", Activation)

        self.sensor_state = []
        self.sensor_type = []

        self.sensor_type.append(0)
        self.sensor_type.append(1)
        self.sensor_type.append(2)

        self.sensor_state.append(self.param_camera_on)
        self.sensor_state.append(self.param_sonar_on)
        self.sensor_state.append(self.param_hydro_on)

        for i in self.sensor_type:
            provider = i
            active = self.sensor_state[i]
            self.open_sensor(provider, active)

        self.msg_is_publish = True

    def run(self, ud):
        if self.msg_is_publish:
            return 'succeeded'

    def end(self):
        pass
