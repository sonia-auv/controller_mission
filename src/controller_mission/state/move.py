import rospy
import smach
from proc_control.srv import SetPositionTarget
from proc_control.msg import TargetReached

_author_ = 'Francis Masse'


class Move(smach.State):
    def __init__(self, config, direction):
        smach.State.__init__(self,
                             outcomes=['succeeded', 'position_not_reach', 'aborted'])

        target_reach_sub = rospy.Subscriber('/proc_control/target_reached', TargetReached, self.target_reach_cb)

        self.direction = direction
        self.target_reached = 0
        self.server = config

    def target_reach_cb(self, data):
        self.target_reached = data.target_is_reached

    def execute(self, ud):
        rate = rospy.Rate(1)

        config = self.server.config

        rospy.wait_for_service('/proc_control/set_global_target')
        set_local_target = rospy.ServiceProxy('/proc_control/set_global_target', SetPositionTarget)

        rospy.loginfo('Executing state MOVE')

        try:
            if self.direction == 'bow':
                response = set_local_target(config.set_position_bow,
                                            0.0,
                                            0.0,
                                            0.0,
                                            0.0,
                                            0.0)
            elif self.direction == 'port':
                response = set_local_target(0.0,
                                            config.set_position_port,
                                            0.0,
                                            0.0,
                                            0.0,
                                            0.0)
            elif self.direction == 'stern':
                response = set_local_target(config.set_position_stern,
                                            0.0,
                                            0.0,
                                            0.0,
                                            0.0,
                                            0.0)
            elif self.direction == 'starboard':
                response = set_local_target(0.0,
                                            config.set_position_starboard,
                                            0.0,
                                            0.0,
                                            0.0,
                                            0.0)

        except rospy.ServiceException as exc:
            rospy.loginfo('Service did not process request: ' + str(exc))

        rospy.loginfo('Set position x = %f' % config.set_position_bow)
        rospy.loginfo('Set position y = %f' % config.set_position_port)
        rospy.loginfo('Set position z = %f' % config.set_position_stern)
        rospy.loginfo('Set position z = %f' % config.set_position_starboard)

        rate.sleep()

        if self.target_reached:
            return 'succeeded'
        else:
            return 'position_not_reach'
