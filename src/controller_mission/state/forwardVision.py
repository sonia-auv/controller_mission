import rospy

from ..mission_state import MissionState, Parameter
from proc_control.srv import SetPositionTarget
from proc_image_processing.msg import VisionTarget
from nav_msgs.msg import Odometry


class ForwardVision(MissionState):

    def __init__(self):
        MissionState.__init__(self)
        self.start_time = None

    def define_parameters(self):
        self.parameters.append(Parameter('param_bounding_box', 1, 'Target'))

    def get_outcomes(self):
        return ['succeeded', 'aborted']

    def initialize(self):
        rospy.wait_for_service('/proc_control/set_local_target')
        self.set_local_target = rospy.ServiceProxy('/proc_control/set_local_target', SetPositionTarget)

        self.buoy_position = rospy.Subscriber('/proc_image_processing/data', VisionTarget, self.vision_callback)

        self.sub_position = rospy.Subscriber('/proc_navigation/odom', Odometry, self.position_callback)

    def run(self, ud):
        if (rospy.get_time() - self.start_time) >= self.time:
            return 'succeeded'

    def end(self):
        pass
