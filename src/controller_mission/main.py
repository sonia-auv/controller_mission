#!/usr/bin/env python

import rospy
import smach
from state.move import Move
from smach_ros import SimpleActionState

__author__ = 'Francis Masse'


def main():
    rospy.init_node('controller_mission')

    # Create a SMACH state machine
    sm = smach.StateMachine(['succeeded', 'aborted', 'preempted'])
    sm.userdata.sm_direction = 'Bow'
    sm.userdata.sm_distance = 5

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('MOVE',
                               Move(),
                               transitions={'succeeded': 'succeeded',
                                            'not_reach': 'MOVE',
                                            'aborted': 'aborted'},
                               remapping={'direction': 'sm_direction',
                                          'distance': 'sm_distance'})

    # Execute SMACH plan
    outcome = sm.execute()


# if __name__ == '__main__':
#     main()
