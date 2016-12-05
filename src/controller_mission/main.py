#!/usr/bin/env python

import rospy
import smach
import smach_ros
from state.move import Move
from dynamic_reconfigure.server import Server
from controller_mission.cfg import SquareTestConfig


__author__ = 'Francis Masse'


def square_test_cb(config, level):
    rospy.loginfo("""Reconfigure Request: {set_position_x}, {set_position_y}, {set_position_z}, \
                  """.format(**config))
    return config


def main():
    rospy.init_node('controller_mission')
    dynamic_reconf_server = Server(SquareTestConfig, square_test_cb)

    # Create a SMACH state machine
    main_sm = smach.StateMachine(['succeeded', 'aborted'])

    # Open the container
    with main_sm:

        # Add states to the container
        smach.StateMachine.add('MOVE_FORWARD',
                               Move(dynamic_reconf_server),
                               transitions={'succeeded': 'MOVE_BACKWARD',
                                            'position_not_reach': 'MOVE_FORWARD',
                                            'aborted': 'aborted'})
        smach.StateMachine.add('MOVE_BACKWARD',
                               Move(dynamic_reconf_server),
                               transitions={'succeeded': 'MOVE_FORWARD',
                                            'position_not_reach': 'MOVE_BACKWARD',
                                            'aborted': 'aborted'})

    sis = smach_ros.IntrospectionServer('controller_mission_server', main_sm, '/controller_mission')
    sis.start()

    # Execute SMACH plan
    outcome = main_sm.execute()

    rospy.spin()

