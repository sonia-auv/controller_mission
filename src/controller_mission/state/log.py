import rospy

from ..mission_state import MissionState, Parameter
from controller_mission.srv import CurrentMission, CurrentMissionResponse
class Log(MissionState):

    def __init__(self):
        MissionState.__init__(self)
        self.do_action = None

    def define_parameters(self):
        self.parameters.append(Parameter('param_start', 1, 'Started(1) or finished(2)'))
        self.parameters.append(Parameter('param_message', 'The message', 'The message to log'))


    def get_outcomes(self):
        return ['succeeded', 'aborted', 'preempted']

    def initialize(self):
        rospy.wait_for_service('/mission_executor/current_mission')
        self.do_action = rospy.ServiceProxy('/mission_executor/current_mission', CurrentMission)
        pass

    def run(self, ud):

        try:

            mission_name = self.do_action().mission
            rospy.loginfo('Mission: \'%s\' Message: \'%s\'  State: \'%s\'' % (mission_name, self.param_message, 'started' if self.param_start == 1 else 'finished'))

        except rospy.ServiceException as exc:
            rospy.loginfo('Service did not process request: ' + str(exc))

        return 'succeeded'

    def end(self):
        pass