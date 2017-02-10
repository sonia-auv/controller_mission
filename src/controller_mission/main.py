#!/usr/bin/env python

import rospy
import smach
import smach_ros
from state.move import Move
from dynamic_reconfigure.server import Server
from controller_mission.cfg import SquareTestConfig


__author__ = 'Francis Masse'


def square_test_cb(config, level):
    rospy.loginfo("""Reconfigure Request: {set_position_bow}, {set_position_port}, {set_position_starboard}, \
                  {set_position_stern}""".format(**config))
    return config


def main():
    rospy.init_node('mission_executor')
    dynamic_reconf_server = Server(SquareTestConfig, square_test_cb)

    # Create a SMACH state machine
    main_sm = smach.StateMachine(['succeeded', 'aborted'])

    # Open the container
    with main_sm:

        # Add states to the container
        smach.StateMachine.add('MOVE_FORWARD',
                               Move(dynamic_reconf_server, 'bow'),
                               transitions={'succeeded': 'MOVE_PORT',
                                            'position_not_reach': 'MOVE_FORWARD',
                                            'aborted': 'aborted'})
        smach.StateMachine.add('MOVE_PORT',
                               Move(dynamic_reconf_server, 'port'),
                               transitions={'succeeded': 'MOVE_BACKWARD',
                                            'position_not_reach': 'MOVE_PORT',
                                            'aborted': 'aborted'})
        smach.StateMachine.add('MOVE_BACKWARD',
                               Move(dynamic_reconf_server, 'stern'),
                               transitions={'succeeded': 'MOVE_STARBOARD',
                                            'position_not_reach': 'MOVE_BACKWARD',
                                            'aborted': 'aborted'})
        smach.StateMachine.add('MOVE_STARBOARD',
                               Move(dynamic_reconf_server, 'starboard'),
                               transitions={'succeeded': 'succeeded',
                                            'position_not_reach': 'MOVE_STARBOARD',
                                            'aborted': 'aborted'})

    sis = smach_ros.IntrospectionServer('controller_mission_server', main_sm, '/controller_mission')
    sis.start()

    # Execute SMACH plan
    outcome = main_sm.execute()

    rospy.spin()

