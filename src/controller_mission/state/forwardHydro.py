import rospy
import math

from ..mission_state import MissionState, Parameter
from proc_mapping.srv import PingerLocationService
from proc_control.srv import SetDecoupledTarget
from proc_control.msg import TargetReached


class ForwardHydro(MissionState):

    def __init__(self):
        MissionState.__init__(self)
        self.target_reached = False

    def define_parameters(self):
        self.parameters.append(Parameter('param_frequency_global_param_name', '/mission_params/00_global/pinger_frequency', 'Global param name with frequency'))

    def get_outcomes(self):
        return ['succeeded', 'aborted', 'preempted']

    def target_reach_cb(self, data):
        self.target_reached = data.target_is_reached

    def initialize(self):
        self.target_reached = 0

        rospy.wait_for_service('/proc_control/set_global_decoupled_target')
        self.set_global_target = rospy.ServiceProxy('/proc_control/set_global_decoupled_target', SetDecoupledTarget)

        rospy.wait_for_service('/proc_mapping/pinger_location_service')
        self.pinger_location_service = rospy.ServiceProxy('/proc_mapping/pinger_location_service', PingerLocationService)

        frequency = rospy.get_param(self.param_frequency_global_param_name, 0.0)

        if not rospy.has_param(self.param_frequency_global_param_name):
            rospy.logerr('No param defined for %s. A default value is used' % self.param_frequency_global_param_name)


        try:
            response = self.pinger_location_service(frequency)

            pose = response.pingerLocation.pose

            self.set_global_target(pose.position.x,
                                   pose.position.y,
                                   pose.position.z,
                                   0,
                                   0,
                                   pose.orientation.z * 180 / math.pi,
                                   False, False, False, True, True, False)
        except rospy.ServiceException as exc:
            rospy.loginfo('Service did not process request: ' + str(exc))
        
        rospy.loginfo('Position received : ' + str(pose))
        self.target_reach_sub = rospy.Subscriber('/proc_control/target_reached', TargetReached, self.target_reach_cb)

    def run(self, ud):
        if self.target_reached > 0:
            return 'succeeded'
        pass

        
    def end(self):
        self.target_reach_sub.unregister()
        pass