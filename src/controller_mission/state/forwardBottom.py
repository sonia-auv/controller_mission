import rospy

from ..mission_state import MissionState, Parameter
from proc_control.msg import TargetReached
from proc_control.srv import SetPositionTarget
from proc_image_processing.msg import VisionTarget
from nav_msgs.msg import Odometry


class ForwardBottom(MissionState):

    def __init__(self):
        MissionState.__init__(self)
        self.vision_is_unreached = False
        self.target_reached = False
        self.victory = False

        self.set_local_target = None
        self.buoy_position = None
        self.target_reach_sub = None
        self.sub_position = None

        self.position_in_z = None

    def define_parameters(self):
        self.parameters.append(Parameter('param_distance_z', 0.3, 'Target'))
        self.parameters.append(Parameter('param_bounding_box', 200, 'Initial bounding Box'))
        self.parameters.append(Parameter('param_distance_max_z', 2.5, 'minimal nb of pixel to ram'))
        self.parameters.append(Parameter('param_nb_pixel_to_victory', 200, 'minimal nb of pixel to ram'))
        self.parameters.append(Parameter('param_topic_to_listen', '/proc_image_processing/bin_result', 'Name of topic to listen'))

    def get_outcomes(self):
        return ['succeeded', 'aborted', 'preempted']

    def sub_position_cb(self, position_data):
        self.position_in_z = position_data.pose.pose.position.z

    def vision_cb(self, vision_data):

        if vision_data.width >= self.param_nb_pixel_to_victory:
            self.victory = True

        if abs(vision_data.x) >= self.param_bounding_box or abs(vision_data.y) >= self.param_bounding_box:
            self.vision_is_unreached = True

    def target_reach_cb(self, data):
        self.target_reached = data.target_is_reached

    def set_target(self, position_z):
        try:
            self.set_local_target(0.0,
                                  0.0,
                                  position_z,
                                  0.0,
                                  0.0,
                                  0.0)
        except rospy.ServiceException as exc:
            rospy.loginfo('Service did not process request: ' + str(exc))

        rospy.loginfo('Set relative position z = %f' % position_z)

    def initialize(self):
        rospy.wait_for_service('/proc_control/set_local_target')
        self.set_local_target = rospy.ServiceProxy('/proc_control/set_local_target', SetPositionTarget)

        self.buoy_position = rospy.Subscriber(str(self.param_topic_to_listen), VisionTarget, self.vision_cb)

        self.target_reach_sub = rospy.Subscriber('/proc_control/target_reached', TargetReached, self.target_reach_cb)

        self.sub_position = rospy.Subscriber('/proc_navigation/odom', Odometry, self.sub_position_cb)

        self.victory = False
        self.vision_is_unreached = False
        self.target_reached = False
        self.position_in_z = None

        self.set_target(self.param_distance_z)

    def run(self, ud):

        if self.position_in_z >= self.param_distance_max_z or self.victory:
            self.set_target(0.0)
            return 'succeeded'

        if self.position_in_z is not None and self.target_reached:
            if self.param_distance_z + self.position_in_z >= self.param_distance_max_z:
                self.set_target(self.param_distance_max_z - self.position_in_z)
            else:
                self.set_target(self.param_distance_z)

        if self.vision_is_unreached or self.target_reached:
            self.set_target(0.0)
            return 'aborted'

    def end(self):
        self.buoy_position.unregister()
        self.target_reach_sub.unregister()
