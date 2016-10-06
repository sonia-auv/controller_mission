#!/usr/bin/env python

import rospy
import smach
from state.move import Move
from geometry_msgs.msg import Pose


__author__ = 'Francis Masse'


def main():
    rospy.init_node('controller_mission')

    # Create a SMACH state machine
    main_sm = smach.StateMachine(['succeeded', 'aborted'])
    # The parameter of the state will be set in the GUI and will automatically
    # create the state block argument

    position_1 = Pose()
    position_1.position.x = 1

    # Open the container
    with main_sm:
        # Add states to the container
        smach.StateMachine.add('MOVE',
                               Move(position_1),
                               transitions={'succeeded': 'succeeded',
                                            'aborted': 'aborted'})

    # Execute SMACH plan
    outcome = main_sm.execute()

