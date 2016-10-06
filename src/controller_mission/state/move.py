import rospy
import smach
from geometry_msgs.msg import Pose

_author_ = 'Francis Masse'


class Move(smach.State):
    def __init__(self, position):
        smach.State.__init__(self,
                             outcomes=['succeeded', 'aborted'],
                             input_keys=[],
                             output_keys=[])
        self.position = position

    def execute(self, ud):
        # TODO Implement Service and Subscriber
        rospy.loginfo('Executing state MOVE')
        if self.position.position.x > 0:
            rospy.loginfo('Bow distance remaining = %f' % self.position.position.x)
            return 'succeeded'
        else:
            return 'aborted'
