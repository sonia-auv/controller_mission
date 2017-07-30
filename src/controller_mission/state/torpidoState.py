import rospy

from ..mission_state import MissionState, Parameter
from provider_actuators.srv import DoActionSrv, DoActionSrvRequest


class Torpido(MissionState):

    def __init__(self):
        MissionState.__init__(self)
        self.start_time = None
        self.do_action = None

        self.action = DoActionSrvRequest()
        self.action_dic = {'1': self.action.SIDE_PORT, '2': self.action.SIDE_STARBOARD}
        self.arme_dic = {'1': self.action.ACTION_TORPEDO_ARMED, '2': self.action.ACTION_DROPPER_LAUNCH}

    def define_parameters(self):
        self.parameters.append(Parameter('param_id', 1, 'Torpido id'))
        self.parameters.append(Parameter('param_launch', 1, 'Launch'))

    def get_outcomes(self):
        return ['succeeded', 'aborted', 'preempted']

    def initialize(self):
        rospy.wait_for_service('/provider_actuators/do_action_srv')
        self.do_action = rospy.ServiceProxy('/provider_actuators/do_action_srv', DoActionSrv)
        try:

            self.do_action(self.action.ELEMENT_TORPEDO, self.action_dic[str(int(self.param_id))], self.arme_dic[str(int(self.param_launch))])

        except rospy.ServiceException as exc:
            rospy.loginfo('Service did not process request: ' + str(exc))

    def run(self, ud):
        rospy.loginfo('Torpido : %i is launch' % int(self.param_id))
        return 'succeeded'

    def end(self):
        pass