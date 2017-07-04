import rospy

from ..mission_state import MissionState, Parameter
from proc_control.srv import SetBoundingBox, ResetBoundingBox


class SetControlBoundingBox(MissionState):

    def __init__(self):
        MissionState.__init__(self)
        self.set_bounding_box = None
        self.reset_bounding_box = None

    def define_parameters(self):
        self.parameters.append(Parameter('param_reset_set_bounding_box', 1, 'Set or reset bounding box'))
        self.parameters.append(Parameter('param_bounding_box_x', 0.5, 'Set x bounding box'))
        self.parameters.append(Parameter('param_bounding_box_y', 0.5, 'Set y bounding box'))
        self.parameters.append(Parameter('param_bounding_box_z', 0.5, 'Set z bounding box'))
        self.parameters.append(Parameter('param_bounding_box_yaw', 0.5, 'set yaw bounding box'))
        self.parameters.append(Parameter('param_bounding_box_roll', 0.5, 'set yaw bounding box'))
        self.parameters.append(Parameter('param_bounding_box_pitch', 0.5, 'set yaw bounding box'))

    def get_outcomes(self):
        return ['succeeded', 'aborted', 'preempted']

    def initialize(self):
        rospy.wait_for_service('/proc_control/set_bounding_box')
        self.set_bounding_box = rospy.ServiceProxy('/proc_control/set_bounding_box', SetBoundingBox)

        rospy.wait_for_service('/proc_control/reset_bounding_box')
        self.reset_bounding_box = rospy.ServiceProxy('/proc_control/reset_bounding_box', ResetBoundingBox)

        if bool(self.param_reset_set_bounding_box):
            self.reset_bounding_box()

        else:
            self.set_bounding_box(X=self.param_bounding_box_x, Y=self.param_bounding_box_y, Z=self.param_bounding_box_z,
                                  ROLL=self.param_bounding_box_roll, PITCH=self.param_bounding_box_pitch,
                                  YAW=self.param_bounding_box_yaw)

    def run(self, ud):
        return 'succeeded'

    def end(self):
        pass
