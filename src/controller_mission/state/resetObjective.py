import rospy

from ..mission_state import MissionState, Parameter
from proc_mapping.srv import ObjectiveResetRequest, ObjectiveReset


class ResetObjective(MissionState):

    def __init__(self):
        MissionState.__init__(self)
        self.reset_objective = None
        self.objectives_to_reset_state = []
        self.objective_type = []
        self.msg_is_publish = False
        self.msg = ObjectiveResetRequest()

    def define_parameters(self):
        self.parameters.append(Parameter('param_reset_buoys', 1, 'Reset Buoys'))
        self.parameters.append(Parameter('param_reset_fence', 1, 'Reset Fence'))
        self.parameters.append(Parameter('param_reset_ping', 1, 'Reset Ping'))
        self.parameters.append(Parameter('param_reset_all', 1, 'Reset All'))

    def get_outcomes(self):
        return ['succeeded', 'aborted']

    def reset_objectives(self):
        self.objective_type.append(ObjectiveResetRequest.BUOY)
        self.objective_type.append(ObjectiveResetRequest.FENCE)
        self.objective_type.append(ObjectiveResetRequest.PINGER)

        self.objectives_to_reset_state.append(self.param_reset_buoys)
        self.objectives_to_reset_state.append(self.param_reset_fence)
        self.objectives_to_reset_state.append(self.param_reset_ping)

        i = ObjectiveResetRequest.BUOY

        if self.param_reset_all:

            self.reset_objective(ObjectiveResetRequest.ALL)

        else:

            for objective_state in self.objectives_to_reset_state:
                if int(objective_state):
                    self.reset_objective(self.objective_type[i])
                i += 1

    def initialize(self):
        rospy.wait_for_service('/proc_mapping/objective_reset/')
        self.reset_objective = rospy.ServiceProxy("/proc_mapping/objective_reset/", ObjectiveReset)

        self.objectives_to_reset_state = []
        self.objective_type = []

        self.reset_objectives()

    def run(self, ud):
        return 'succeeded'

    def end(self):
        pass
