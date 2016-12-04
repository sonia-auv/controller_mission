#!/usr/bin/env python

import rospy
import smach
import smach_ros
from state.move import Move
from proc_control.msg import TargetReached
from geometry_msgs.msg import Pose


__author__ = 'Francis Masse'


def target_reach_cb(data):
    if data:
        main_sm.userdata.position_reached = 1


def main():
    rospy.init_node('controller_mission')
    rospy.Subscriber('local_target_reach', TargetReached, target_reach_cb)

    # Create a SMACH state machine
    main_sm = smach.StateMachine(['succeeded', 'aborted'])

    main_sm.userdata.position_reached = 0

    # Open the container
    with main_sm:
        # Add states to the container
        smach.StateMachine.add('MOVE_FORWARD',
                               Move(),
                               transitions={'succeeded': 'MOVE_BACKWARD',
                                            'aborted': 'aborted'},
                               remapping={'position_reached': 'position_reached'})
        smach.StateMachine.add('MOVE_BACKWARD',
                               Move(),
                               transitions={'succeeded': 'succeeded',
                                            'aborted': 'aborted'},
                               remapping={'position_reached': 'position_reached'})

    sis = smach_ros.IntrospectionServer('controller_mission_server', main_sm, '/controller_mission')
    sis.start()
    # Execute SMACH plan
    outcome = main_sm.execute()

    rospy.spin()

