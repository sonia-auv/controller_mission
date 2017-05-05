import rospy
import math

from ..mission_state import MissionState, Parameter
from controller_mission.srv import TrustingPositionObject


class FindObject(MissionState):

    def __init__(self):
        MissionState.__init__(self)
        self.actual_position_x = 0.0
        self.actual_position_y = 0.0
        self.actual_heading = 0.0
        self.target_reached = None
        self.object = {' buoys': 1, ' fence': 2, ' hydro': 3}

        self.found_object = None
        self.trusting_position = None

    def define_parameters(self):
        self.parameters.append(Parameter('object_to_found', 'buoys', 'Aligned to object'))
        self.parameters.append(Parameter('trusting_param', 50, 'Aligned to object'))

    def get_outcomes(self):
        return ['succeeded', 'aborted']

    def initialize(self):
        rospy.wait_for_service('/controller_mission/find_object')
        self.to_know_the_trusting = rospy.ServiceProxy('/controller_mission/find_object', TrustingPositionObject)

        self.found_object = self.object_to_found
        self.trusting_position = self.trusting_param

    def run(self, ud):

        trust = self.to_know_the_trusting(self.object[self.found_object])

        rospy.sleep(1)

        print trust.trust

        if trust.trust >= int(self.trusting_position):
            return 'succeeded'
