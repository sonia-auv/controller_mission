import rospy
import math

from Queue import deque
from ..mission_state import MissionState, Parameter
from proc_mapping.srv import PingerLocationService
from proc_control.srv import SetDecoupledTarget
from proc_control.msg import TargetReached
from nav_msgs.msg import Odometry


class HydroYawFirst(MissionState):

    def __init__(self):
        MissionState.__init__(self)
        self.target_reached = False
        self.response = None
        self.yaw_reach = False

    def define_parameters(self):
        self.parameters.append(Parameter('param_frequency', '$/mission_params/00_global/pinger_frequency', 'Pigner frequency'))

    def get_outcomes(self):
        return ['succeeded', 'aborted', 'preempted']

    def target_reach_cb(self, data):
        self.target_reached = data.target_is_reached

    def initialize(self):
        rospy.wait_for_service('/proc_control/set_global_decoupled_target')
        self.set_global_target = rospy.ServiceProxy('/proc_control/set_global_decoupled_target', SetDecoupledTarget)

        rospy.wait_for_service('/proc_mapping/pinger_location_service')
        self.pinger_location_service = rospy.ServiceProxy('/proc_mapping/pinger_location_service', PingerLocationService)

        self.target_reached = 0

        self.yaw_reach = False

        try:
            self.response = self.pinger_location_service(self.param_frequency)

            pose = self.response.pingerLocation.pose

            yaw = pose.orientation.z * 180 / math.pi

            self.set_global_target(0.0,
                                   0.0,
                                   0.0,
                                   0.0,
                                   0.0,
                                   yaw,
                                   True, True, True, True, True, False)
            
            rospy.loginfo('Set global orientation yaw = %f' % yaw)

        except rospy.ServiceException as exc:
            rospy.loginfo('Service did not process request: ' + str(exc))
        
        self.target_reach_sub = rospy.Subscriber('/proc_control/target_reached', TargetReached, self.target_reach_cb)

    def run(self, ud):
        if self.target_reached > 0:
            if not self.yaw_reach:
                self.yaw_reach = True
                self.target_reached = False
                try:
                    pose = self.response.pingerLocation.pose
                    self.set_global_target(pose.position.x,
                                           pose.position.y,
                                           0.0,
                                           0.0,
                                           0.0,
                                           0.0,
                                           False, False, True, True, True, True)

                    rospy.loginfo('Set global position x = %f y = %f' % (pose.position.x, pose.position.y))

                except rospy.ServiceException as exc:
                    rospy.loginfo('Service did not process request: ' + str(exc))

            else:
                return 'succeeded'
        pass

        
    def end(self):
        self.target_reach_sub.unregister()
        pass