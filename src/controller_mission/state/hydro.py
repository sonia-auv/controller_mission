import rospy
import math

from Queue import deque
from ..mission_state import MissionState, Parameter
from proc_mapping.srv import PingerLocationService
from proc_control.srv import SetPositionTarget
from proc_control.msg import TargetReached
from nav_msgs.msg import Odometry


class Hydro(MissionState):

    def __init__(self):
        MissionState.__init__(self)
        self.target_reached = False

    def define_parameters(self):
        self.parameters.append(Parameter('param_queu_size', 10, 'Maximum size of queue'))

    def get_outcomes(self):
        return ['succeeded', 'aborted', 'preempted']

    def target_reach_cb(self, data):
        self.target_reached = data.target_is_reached

    def initialize(self):
        rospy.wait_for_service('/proc_control/set_global_target')
        self.set_global_target = rospy.ServiceProxy('/proc_control/set_global_target', SetPositionTarget)

        rospy.wait_for_service('/proc_mapping/pinger_location_service')
        self.pinger_location_service = rospy.ServiceProxy('/proc_mapping/pinger_location_service', PingerLocationService)

        try:
            response = self.pinger_location_service(40) #TODO param_frequency

            pose = response.pingerLocation.pose

            #rospy.loginfo('Position received : ' + str(pose))

            self.set_global_target(pose.position.x,
                                   pose.position.y,
                                   pose.position.z,
                                   0,
                                   0,
                                   pose.orientation.z * 180 / math.pi)
        except rospy.ServiceException as exc:
            rospy.loginfo('Service did not process request: ' + str(exc))
        
        self.target_reach_sub = rospy.Subscriber('/proc_control/target_reached', TargetReached, self.target_reach_cb)

    def run(self, ud):
        if self.target_reached > 0:
            return 'succeeded'
        pass

        
    def end(self):
        self.target_reach_sub.unregister()
        pass