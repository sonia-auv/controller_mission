import rospy

from ..mission_state import MissionState, Parameter
from proc_control.srv import SetDecoupledTarget, SetControlMode, SetControlModeRequest
from proc_control.msg import TargetReached
from proc_mapping.msg import PingerLocation
import math


class MoveYawHydro(MissionState):

    def __init__(self):
        MissionState.__init__(self)
        self.target_reached = False
        self.set_local_target = None
        self.pinger_location = None
        self.heading = None
        self.mode = SetControlModeRequest()
        self.mode_dic = {'0': self.mode.PositionModePID, '1': self.mode.PositionModePPI, '2': self.mode.VelocityModeB}

    def define_parameters(self):
        self.parameters.append(Parameter('param_pinger_frequency', 0.0, 'Pinger frequency'))

    def get_outcomes(self):
        return ['succeeded', 'aborted', 'preempted']

    def target_reach_cb(self, data):
        self.target_reached = data.target_is_reached

    def pinger_location_cb(self, pinger_location_data):
        if pinger_location_data.frequency == self.param_pinger_frequency:
            self.heading = pinger_location_data.pose.orientation.z
            self.heading = self.heading * 180 / math.pi

            try:
                self.set_local_target(0.0, 0.0, 0.0,
                                      0.0, 0.0, self.heading,
                                      True, True, True, True, True, False)
            except rospy.ServiceException as exc:
                rospy.loginfo('Service did not process request: ' + str(exc))

            rospy.loginfo('Set pinger heading yaw = %f' % self.heading)
            self.target_reach_sub = rospy.Subscriber('/proc_control/target_reached', TargetReached,
                                                     self.target_reach_cb)

    def initialize(self):

        rospy.wait_for_service('/proc_control/set_control_mode')
        self.set_mode = rospy.ServiceProxy('/proc_control/set_control_mode', SetControlMode)
        try:
            self.set_mode(self.mode_dic[str(int(0))])

        except rospy.ServiceException as exc:
            rospy.loginfo('Service did not process request: ' + str(exc))

        rospy.wait_for_service('/proc_control/set_global_target')
        self.set_local_target = rospy.ServiceProxy('/proc_control/set_global_target', SetDecoupledTarget)

        self.target_reached = False

        self.pinger_location = rospy.Subscriber('/proc_mapping/pinger_location', PingerLocation,
                                                self.pinger_location_cb)

    def run(self, ud):
        if self.target_reached > 0:
            return 'succeded'

    def end(self):
        self.pinger_location.unregister()
        self.target_reach_sub.unregister()

