import rospy

from ..mission_state import MissionState, Parameter
from proc_control.srv import SetDecoupledTarget
from proc_mapping.msg import PingerLocation


class MoveSpeedHydro(MissionState):

    def __init__(self):
        MissionState.__init__(self)
        self.set_local_target = None
        self.pinger_location = None
        self.heading = None

    def define_parameters(self):
        self.parameters.append(Parameter('param_speed_x', 1.0, 'Speed to use while traveling'))
        self.parameters.append(Parameter('param_pinger_frequency', 0.0, 'Pinger frequency'))

    def get_outcomes(self):
        return ['succeeded', 'aborted', 'preempted']

    def initialize(self):
        rospy.wait_for_service('/proc_control/set_local_target')
        self.set_local_target = rospy.ServiceProxy('/proc_control/set_local_target', SetDecoupledTarget)

        self.pinger_location = rospy.Subscriber('/proc_mapping/pinger_location', PingerLocation,
                                                self.pinger_location_cb)

        try:
            self.set_local_target(self.param_speed_x,
                                  0.0,
                                  0.0,
                                  0.0,
                                  0.0,
                                  0.0,
                                  False, False, True, True, True, True)
        except rospy.ServiceException as exc:
            rospy.loginfo('Service did not process request: ' + str(exc))

        rospy.loginfo('Set speed x = %f' % self.param_distance_x)
        rospy.loginfo('Set pinger heading yaw = %f' % self.heading)

    def pinger_location_cb(self, pinger_location_data):
        if pinger_location_data.frequency == self.param_pinger_frequency:
            self.heading = pinger_location_data.pose.orientation.z

            try:
                self.set_local_target(self.param_speed_x,
                                      0.0,
                                      0.0,
                                      0.0,
                                      0.0,
                                      self.heading,
                                      False, False, True, True, True, False)
            except rospy.ServiceException as exc:
                rospy.loginfo('Service did not process request: ' + str(exc))

            rospy.loginfo('Set speed x = %f' % self.param_distance_x)
            rospy.loginfo('Set pinger heading yaw = %f' % self.heading)

    def run(self, ud):
        if self.target_reached:
            return 'succeeded'

    def end(self):
        self.pinger_location.unregister()
