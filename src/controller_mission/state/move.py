import rospy
import smach

_author_ = 'Francis Masse'


class Move(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['outcome1', 'outcome2'],
                             input_keys=['direction'])

    def execute(self, userdata):
        rospy.loginfo('Executing state MOVE')
        if userdata == 1:
            return 'outcome1'
        else:
            return 'outcome2'

