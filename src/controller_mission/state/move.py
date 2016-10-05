import rospy
import smach

_author_ = 'Francis Masse'


class Move(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['succeeded', 'not_reach', 'aborted'],
                             input_keys=['direction', 'distance_in'],
                             output_keys=['distance_out'])

    def execute(self, userdata):
        rospy.loginfo('Executing state MOVE')
        if userdata.direction == 'Bow':
            if userdata.distance_in > 0:
                userdata.distance_out = userdata.distance_in - 1
                rospy.loginfo('Bow distance remaining = %f' % userdata.distance_in)
                return 'not_reach'
            else:
                return 'succeeded'
        if userdata.direction == 'Stern':
            if userdata.distance_in > 0:
                userdata.distance_out = userdata.distance_in - 1
                rospy.loginfo('Stern distance remaining = %f' % userdata.distance_in)
                return 'not_reach'
            else:
                return 'succeeded'
        else:
            return 'aborted'
