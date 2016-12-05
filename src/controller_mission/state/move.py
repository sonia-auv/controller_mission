import rospy
import smach
from proc_control.srv import SetPositionTarget
from proc_control.msg import TargetReached

_author_ = 'Francis Masse'


class Move(smach.State):
    def __init__(self, config):
        smach.State.__init__(self,
                             outcomes=['succeeded', 'position_not_reach', 'aborted'])

        target_reach_sub = rospy.Subscriber('/proc_control/target_reached', TargetReached, self.target_reach_cb)
        self.target_reached = 0
        self.server = config

    def target_reach_cb(self, data):
        self.target_reached = data.target_is_reached

    def execute(self, ud):
        rate = rospy.Rate(1)

        config = self.server.config

        rospy.wait_for_service('/proc_control/set_local_target')
        set_local_target = rospy.ServiceProxy('/proc_control/set_local_target', SetPositionTarget)

        rospy.loginfo('Executing state MOVE')

        try:
            response = set_local_target(config.set_position_x,
                                        config.set_position_y,
                                        config.set_position_z,
                                        0.0,
                                        0.0,
                                        0.0)

        except rospy.ServiceException as exc:
            rospy.loginfo('Service did not process request: ' + str(exc))

        rospy.loginfo('Set position x = %f' % config.set_position_x)
        rospy.loginfo('Set position y = %f' % config.set_position_y)
        rospy.loginfo('Set position z = %f' % config.set_position_z)

        rate.sleep()

        if self.target_reached:
            return 'succeeded'
        else:
            return 'position_not_reach'
