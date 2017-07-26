import rospy
import numpy as np
import math

from Queue import deque
from ..mission_state import MissionState, Parameter
from proc_hydrophone.msg import PingPose
from proc_control.srv import SetPositionTarget
from proc_control.msg import TargetReached
from nav_msgs.msg import Odometry


class Hydro(MissionState):

    def __init__(self):
        MissionState.__init__(self)
        self.start_time = None

        self.target_reached = False
        self.ping_heading = None
        self.ping = None

    def define_parameters(self):
        self.parameters.append(Parameter('param_queu_size', 10, 'Maximum size of queue'))

    def get_outcomes(self):
        return ['succeeded', 'aborted', 'preempted']

    def ping_cb(self, data):
        self.ping_heading.append(data.pose.oriantation.z)
        if len(self.ping_heading) == self.param_queu_size:
            self.parse_ping_data()

    def parse_ping_data(self):
        x = []
        y = []
        for heading in self.ping_heading:
            x.append(math.cos(heading))
            y.append(math.sin(heading))

        sorted(x)
        sorted(y)

        if len(x) % 2 == 1:

            median_x = x[math.floor(len(x)/2.0)]

        else:

            median_x = (x[len(x)/2 - 1] / 2) + (x[len(x)/2] / 2)

        if len(y) % 2 == 1:

            median_y = y[math.floor(len(y) / 2.0)]

        else:

            median_y = (y[len(y) / 2 - 1] / 2) + (y[len(y) / 2] / 2)

        heading = math.degrees(math.atan2(median_y, median_x))

        self.set_target(heading)

    def target_reach_cb(self, data):
        self.target_reached = data.target_is_reached

    def current_position_cb(self, position):
        self.position = position.pose.pose.position
        self.orientation = position.pose.pose.orientation

    def wait_until_position_is_get(self):
        while not (self.position and self.orientation):
            pass
        return

    def set_target(self, heading):

        try:
            response = self.set_global_target(self.position.x,
                                              self.position.y,
                                              self.position.z,
                                              self.orientation.x,
                                              self.orientation.y,
                                              heading)
            self.target_reached = False
        except rospy.ServiceException as exc:
            rospy.loginfo('Service did not process request: ' + str(exc))

        rospy.loginfo('Set position Yaw = %f' % self.param_heading)

        self.target_reach_sub = rospy.Subscriber('/proc_control/target_reached', TargetReached, self.target_reach_cb)

    def run(self, ud):
        if self.target_reached > 0:
            return 'succeeded'

    def initialize(self):
        rospy.wait_for_service('/proc_control/set_global_target')
        self.set_global_target = rospy.ServiceProxy('/proc_control/set_global_target', SetPositionTarget)

        self.current_position = rospy.Subscriber('/proc_navigation/odom', Odometry, self.current_position_cb)

        self.position = None
        self.orientation = None
        self.wait_until_position_is_get()

        self.ping = rospy.Subscriber('/proc_hydrophone/ping', PingPose, self.ping_cb)

        self.ping_heading = deque([], maxlen=self.param_queu_size)

        self.ping_heading.clear()

    def end(self):
        pass
