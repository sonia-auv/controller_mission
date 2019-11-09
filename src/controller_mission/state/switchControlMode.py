import rospy

from ..mission_state import MissionState, Parameter
from proc_control.srv import SetControlMode, SetControlModeRequest, SetDecoupledTarget

class Switch(MissionState):

    def __init__(self):
        MissionState.__init__(self)
        self.start_time = None
        self.set_mode = None

        self.mode = SetControlModeRequest()
        self.mode_dic = {'0': self.mode.PositionModePID, '1': self.mode.PositionModePPI, '2': self.mode.VelocityModeB}

    def define_parameters(self):
        self.parameters.append(Parameter('param_mode', 0, 'Control Mode'))

    def get_outcomes(self):
        return ['succeeded', 'aborted', 'preempted']

    def initialize(self):
        rospy.wait_for_service('/proc_control/set_control_mode')
        self.set_mode = rospy.ServiceProxy('/proc_control/set_control_mode', SetControlMode)
        try:

            self.set_mode(self.mode_dic[str(int(self.param_mode))])
            if self.param_mode==0:
                rospy.wait_for_service('/proc_control/set_local_decoupled_target')
                set_local_target = rospy.ServiceProxy('/proc_control/set_local_decoupled_target', SetDecoupledTarget)
                set_local_target(0.0,
                                 0.0, 0.0, 0.0, 0.0, 0.0,
                                 False, False, False, True, True, False)
        except rospy.ServiceException as exc:
            rospy.loginfo('Service did not process request: ' + str(exc))

    def run(self, ud):
        rospy.loginfo('Switch : %i is executed' % int(self.param_mode))
        return 'succeeded'

    def end(self):
        pass