import rospy
import smach

_author_ = 'Francis Masse'


class Move(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['outcome1', 'outcome2'],
                             input_keys=['direction', 'distance_in'],
                             output_keys=['distance_out'])

    def execute(self, userdata):
        rospy.loginfo('Executing state MOVE')
        if userdata.direction == 1 and userdata.distance_in > 0:
            userdata.distance_out = userdata.distance_in - 1
            rospy.loginfo('distance remaining = %f' % userdata.distance_in)
            return 'outcome1'
        else:
            return 'outcome2'

