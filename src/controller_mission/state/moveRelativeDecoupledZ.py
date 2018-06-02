# import rospy

# from ..mission_state import MissionState, Parameter
# from proc_control.msg import TargetReached
# from proc_control.srv import SetPositionTarget


# class MoveRelativeZ(MissionState):

#     def __init__(self):
#         MissionState.__init__(self)
#         self.set_local_target = None
#         self.target_reach_sub = None

#         self.actual_position_x = 0.0
#         self.actual_position_y = 0.0
#         self.just_one_time = 0
#         self.target_reached = False

#     def define_parameters(self):
#         self.parameters.append(Parameter('param_distance_z', 1.0, 'Distance to travel'))

#     def get_outcomes(self):
#         return ['succeeded', 'aborted', 'preempted']

#     def target_reach_cb(self, data):
#         self.target_reached = data.target_is_reached

#     def initialize(self):
#         rospy.wait_for_service('/proc_control/set_local_target')
#         self.set_local_target = rospy.ServiceProxy('/proc_control/set_local_target', SetPositionTarget)

#         self.target_reach_sub = rospy.Subscriber('/proc_control/target_reached', TargetReached, self.target_reach_cb)

#         try:
#             self.set_local_target(0.0,
#                                   0.0,
#                                   self.param_distance_z,
#                                   0.0,
#                                   0.0,
#                                   0.0)
#         except rospy.ServiceException as exc:
#             rospy.loginfo('Service did not process request: ' + str(exc))

#         rospy.loginfo('Set relative position z = %f' % self.param_distance_z)

#     def run(self, ud):
#         if self.target_reached > 0:
#             return 'succeeded'

#     def end(self):
#         self.target_reach_sub.unregister()
