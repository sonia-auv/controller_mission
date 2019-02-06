import rospy

from ..mission_state import MissionState, Parameter
from proc_control.msg import TargetReached
from proc_control.srv import SetDecoupledTarget
from nav_msgs.msg import Odometry
import math


class MoveToXYGradualy(MissionState):

    def __init__(self):
        MissionState.__init__(self)

        self.position = None
        self.orientation = None
        self.target_reached = False

    def define_parameters(self):
        self.parameters.append(Parameter('param_distance_x', 1.0, 'Distance to travel'))
        self.parameters.append(Parameter('param_distance_y', 1.0, 'Distance to travel'))
        self.parameters.append(Parameter('param_max_distance', 3.0, 'Distance to travel'))

    def get_outcomes(self):
        return ['succeeded', 'aborted', 'preempted']

    def target_reach_cb(self, data):
        self.target_reached = data.target_is_reached

    def current_position_cb(self, position):
        self.position = position.pose.pose.position
        self.orientation = position.pose.pose.orientation

    def wait_until_position_is_get(self):
        while not self.position:
            pass
        return

    def initialize(self):

        self.target_reach_sub = None

        rospy.wait_for_service('/proc_control/set_global_decoupled_target')
        set_global_target = rospy.ServiceProxy('/proc_control/set_global_decoupled_target', SetDecoupledTarget)

        self.current_position = rospy.Subscriber('/proc_navigation/odom', Odometry, self.current_position_cb)

        self.position = None
        self.orientation = None
        self.wait_until_position_is_get()
        self.target_reached = False

        distance = math.sqrt(math.pow(self.param_distance_x - self.position.x,2) + math.pow(self.param_distance_y - self.position.y,2))

        rospy.loginfo('distance %f' % distance)

        if distance <= self.param_max_distance:
            try:
                
                
                response = set_global_target(self.param_distance_x,
                                            self.param_distance_y,
                                            0,0,0,0,False,False,True,True,True,True)
                self.target_reached = False
            except rospy.ServiceException as exc:
                rospy.loginfo('Service did not process request: ' + str(exc))

            rospy.loginfo('Set position x = %f' % self.param_distance_x)
            rospy.loginfo('Set position y = %f' % self.param_distance_y)

            self.target_reach_sub = rospy.Subscriber('/proc_control/target_reached', TargetReached, self.target_reach_cb)
        
        else:
            self.target_reached = 1

    def run(self, ud):
        if self.target_reached > 0:
            return 'succeeded'

    def end(self):
        if self.target_reach_sub != None:
            self.target_reach_sub.unregister()
        self.current_position.unregister()
