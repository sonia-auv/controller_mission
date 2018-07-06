import rospy

from ..mission_state import MissionState, Parameter
from proc_mapping.srv import ObjectiveReset


class ResetMappingObjective(MissionState):

    def __init__(self):
        MissionState.__init__(self)

    def define_parameters(self):
        self.parameters.append(Parameter('param_objective_type', 2, 'Type'))

    def get_outcomes(self):
        return ['succeeded', 'aborted', 'preempted']

    def initialize(self):
        rospy.wait_for_service('/proc_mapping/objective_reset')
        self.objective_reset = rospy.ServiceProxy('/proc_mapping/objective_reset', ObjectiveReset)

        try:
            self.objective_reset(self.param_objective_type)
        except rospy.ServiceException as exc:
            rospy.loginfo('Service did not process request: ' + str(exc))

        typeName = "All" if self.param_objective_type == -1 else "Buoy" if self.param_objective_type == 0 else "Fence" if self.param_objective_type == 1 else "Pinger" if self.param_objective_type == 2 else "UNKNOWN"
        
        rospy.loginfo('Reset objective %d = %s', self.param_objective_type, typeName)

    def run(self, ud):
        return 'succeeded'

    def end(self):
        pass
