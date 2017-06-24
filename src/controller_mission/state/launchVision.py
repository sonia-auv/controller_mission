import rospy

from ..mission_state import MissionState, Parameter
from proc_image_processing.srv import execute_cmd


class LaunchVision(MissionState):

    def __init__(self):
        MissionState.__init__(self)
        self.execute_vision_cmd = None

    def define_parameters(self):
        self.parameters.append(Parameter('param_node_name', '/controller_mission/buoy', 'Topic of  result'))
        self.parameters.append(Parameter('param_filterchain_name', 'camera_feed', 'Times Out'))
        self.parameters.append(Parameter('param_media_name', 'FRONT_GIGE', 'Times Out'))
        self.parameters.append(Parameter('param_cmd', 1, 'Times Out'))

    def get_outcomes(self):
        return ['succeeded', 'aborted', 'preempted']

    def initialize(self):
        rospy.wait_for_service('/proc_image_processing/execute_cmd')
        self.execute_vision_cmd = rospy.ServiceProxy('/proc_image_processing/execute_cmd', execute_cmd)

        try:
            self.execute_vision_cmd(self.param_node_name,
                                    self.param_filterchain_name,
                                    self.param_media_name,
                                    self.param_cmd)
        except rospy.ServiceException as exc:
            rospy.loginfo('Service did not process request: ' + str(exc))

    def run(self, ud):
        return 'succeeded'

    def end(self):
        pass
