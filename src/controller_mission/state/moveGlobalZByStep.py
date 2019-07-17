import rospy

from ..mission_state import MissionState, Parameter
from proc_control.msg import TargetReached
from proc_control.srv import SetDecoupledTarget
from nav_msgs.msg import Odometry
import math


class MoveZByStep(MissionState):

    def __init__(self):
        MissionState.__init__(self)

        self.position = None
        self.orientation = None
        self.target_reached = False
        self.current_position = None

        self.move_queue = None

        self.set_global_target = None
        self.target_reach_sub = None

    def define_parameters(self):
        self.parameters.append(Parameter('param_z', 1.0, 'Distance to travel'))
        self.parameters.append(Parameter('param_z_step', 0.5, 'Distance to travel'))

    def get_outcomes(self):
        return ['succeeded', 'aborted', 'preempted']

    def target_reach_cb(self, data):

        if data.target_is_reached > 0 and len(self.move_queue) > 0:
            self.move_z(self.move_queue.pop(0))
        else:
            self.target_reached = data.target_is_reached

    def current_position_cb(self, position):
        self.position = position.pose.pose.position

    def wait_until_position_is_get(self):
        while not self.position:
            pass
        return

    def initialize(self):

        rospy.wait_for_service('/proc_control/set_global_decoupled_target')
        self.set_global_target = rospy.ServiceProxy('/proc_control/set_global_decoupled_target', SetDecoupledTarget)

        self.move_queue = []

        self.current_position = rospy.Subscriber('/proc_navigation/odom', Odometry, self.current_position_cb)

        self.wait_until_position_is_get()

        delta_z = self.param_z - self.position.z

        nb_step = 0
        mod = 0
        sign = 0

        if delta_z > 0:
            nb_step = int(math.floor(delta_z/self.param_z_step))
            mod = delta_z % self.param_z_step
            sign = 1
        else:
            nb_step = int(math.ceil(delta_z/self.param_z_step))
            mod = delta_z % self.param_z_step - self.param_z_step
            sign = -1

        abs_nb_step = abs(nb_step)

        for i in range(1, abs_nb_step + 1):
            self.move_queue.append(self.position.z + self.param_z_step * i * sign)

        if mod != 0:
            if abs_nb_step > 0:
                self.move_queue.append(self.move_queue[-1] + mod)
            else:
                self.move_queue.append(self.position.z + mod)

        print(self.move_queue)

        self.target_reached = False

        self.target_reach_sub = rospy.Subscriber('/proc_control/target_reached', TargetReached, self.target_reach_cb)

        self.move_z(self.move_queue.pop(0))

    def move_z(self, z_distance):

        try:
            self.set_global_target(0,
                                   0,
                                   z_distance,
                                   0,
                                   0,
                                   0,
                                   True, True, False, True, True, True)
            self.target_reached = False
        except rospy.ServiceException as exc:
            rospy.loginfo('Service did not process request: ' + str(exc))

        rospy.loginfo('Set position z = %f' % z_distance)

        pass

    def run(self, ud):
        if self.target_reached > 0:
            return 'succeeded'

    def end(self):
        self.target_reach_sub.unregister()
        self.current_position.unregister()
