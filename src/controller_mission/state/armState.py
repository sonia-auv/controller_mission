import rospy

from ..mission_state import MissionState, Parameter
from proc_actuators.srv import cmActionSrv, cmActionSrvRequest


class Arm(MissionState):

    def __init__(self):
        MissionState.__init__(self)
        self.start_time = None
        self.do_action = None
        self.action = cmActionSrvRequest()
        self.action_dic = {'0': self.action.ARM_CLOSE,
                           '1': self.action.ARM_OPEN}

    def define_parameters(self):
        self.parameters.append(Parameter('param_id', 1, 'Times Out'))

    def get_outcomes(self):
        return ['succeeded', 'aborted', 'preempted']

    def initialize(self):
        rospy.wait_for_service('/proc_actuators/cm_action_srv')
        self.do_action = \
            rospy.ServiceProxy('/proc_actuators/cm_action_srv',
                               cmActionSrv)
        try:
            self.do_action(self.action.ELEMENT_ARM,
                           self.action_dic[str(int(self.param_id))],
                           self.action.ACTION_ARM_EXEC)
        except rospy.ServiceException, exc:

            rospy.loginfo('Service did not process request: '
                          + str(exc))

    def run(self, ud):
        rospy.loginfo('arm : %i is executed' % int(self.param_id))
        return 'succeeded'

    def end(self):
        pass