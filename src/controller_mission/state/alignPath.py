import rospy

from ..mission_state import MissionState, Parameter

from proc_control.srv import SetPositionTarget
from proc_image_processing.msg import VisionTarget


class AlignPath(MissionState):

    def __init__(self):
        MissionState.__init__(self)
        self.vision_subscriber = None
        self.set_local_target = None

        self.vision_pose_x = 0.0
        self.vision_pose_y = 0.0
        self.vision_yaw = 0.0
        self.vision_width = 0.0

        self.pose_x_is_reached = False
        self.pose_y_is_reached = False
        self.pose_yaw_is_reached = False
        self.submarine_is_align = False


    def define_parameters(self):
        self.parameters.append(Parameter('param_bounding_box', 1.0, 'bounding box'))
        self.parameters.append(Parameter('param_vision_target_width_in_meter', 0.15, 'transform pixel to meter'))
        self.parameters.append(Parameter('param_vision_target_height_in_meter', 1.2, 'transform pixel to meter'))
        self.parameters.append(Parameter('param_topic_to_listen', '/proc_image_processing/data', 'Name of topic to listen'))
        self.parameters.append(Parameter('param_nb_pixel_depth', 300, 'Minimal nb of pixel to stop'))

    def get_outcomes(self):
        return ['succeeded', 'aborted']

    def vision_subscriber_cb(self, receive):
        pixel_to_meter = (receive.width / self.param_vision_target_width_in_meter +
                          receive.height / self.param_vision_target_height_in_meter) / 2

        self.vision_pose_x = receive.x * pixel_to_meter
        self.vision_pose_y = receive.y * pixel_to_meter
        self.vision_yaw = receive.angle

        if abs(self.vision_pose_x) <= self.param_bounding_box:
            self.pose_x_is_reached = True

        if abs(self.vision_pose_y) <= self.param_bounding_box:
            self.pose_y_is_reached = True

        if abs(self.vision_yaw) <= self.param_bounding_box:
            self.pose_yaw_is_reached = True


    def align_path(self):
        stare_pose_x = self.vision_pose_x
        stare_pose_y = self.vision_pose_y
        stare_pose_yaw = self.vision_yaw

        if self.pose_x_is_reached:
            stare_pose_x = 0.0

        if self.pose_y_is_reached:
            stare_pose_y = 0.0

        if self.pose_yaw_is_reached:
            stare_pose_yaw = 0.0

        if self.pose_x_is_reached and self.pose_y_is_reached and self.pose_yaw_is_reached:
            self.submarine_is_align = True
        else:
            self.set_target(stare_pose_x, stare_pose_y, stare_pose_yaw)

    def set_target(self, position_x, position_y, position_yaw):
        try:
            self.set_local_target(position_x,
                                  position_y,
                                  0.0,
                                  0.0,
                                  0.0,
                                  position_yaw)
        except rospy.ServiceException as exc:
            rospy.loginfo('Service did not process request: ' + str(exc))

        rospy.loginfo('Set relative position y = %f' % position_x)
        rospy.loginfo('Set relative position z = %f' % position_y)
        rospy.loginfo('Set relative position yaw = %f' % position_yaw)

    def initialize(self):
        self.pose_x_is_reached = False
        self.pose_y_is_reached = False
        self.pose_yaw_is_reached = False
        self.submarine_is_align = False

        rospy.wait_for_service('/proc_control/set_local_target')
        self.set_local_target = rospy.ServiceProxy('/proc_control/set_local_target', SetPositionTarget)

        self.vision_subscriber = rospy.Subscriber(self.param_topic_to_listen, VisionTarget, self.vision_subscriber_cb)

    def run(self, ud):
        self.align_path()

        if self.submarine_is_align:
            self.set_target(0.0, 0.0, 0.0)
            return 'succeeded'

    def end(self):
        self.vision_subscriber.unregister()
