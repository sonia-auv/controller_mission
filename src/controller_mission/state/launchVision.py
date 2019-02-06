import rospy

from ..mission_state import MissionState, Parameter
from proc_image_processing.srv import execute_cmd
from provider_vision.srv import start_stop_media


class LaunchVision(MissionState):

    def __init__(self):
        MissionState.__init__(self)
        self.execute_vision_cmd = None
        self.start_stop_vision = None

    def define_parameters(self):
        self.parameters.append(Parameter('param_node_name', 'align_buoy', 'Topic of  result'))
        self.parameters.append(Parameter('param_filterchain_name', 'simple_buoy', 'Filter chain name'))
        self.parameters.append(Parameter('param_media_name', '/provider_vision/Front_GigE', 'Media name'))
        self.parameters.append(Parameter('param_start_front', 1, 'Media name'))
        self.parameters.append(Parameter('param_start_bottom', 2, 'Media name'))
        self.parameters.append(Parameter('param_cmd', 1, 'Times Out'))

    def get_outcomes(self):
        return ['succeeded', 'aborted', 'preempted']

    def initialize(self):
        rospy.wait_for_service('/proc_image_processing/execute_cmd')
        self.execute_vision_cmd = rospy.ServiceProxy('/proc_image_processing/execute_cmd', execute_cmd)

        rospy.wait_for_service('/provider_vision/start_stop_camera')
        self.start_stop_vision = rospy.ServiceProxy('/provider_vision/start_stop_camera', start_stop_media)

        try:
            if self.param_start_front == 1:

                self.start_stop_vision('Front_GigE', self.param_start_front)
                self.start_stop_vision('Bottom_GigE', self.param_start_bottom)

            else:

                self.start_stop_vision('Front_GigE', self.param_start_front)
                self.start_stop_vision('Bottom_GigE', self.param_start_bottom)


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
