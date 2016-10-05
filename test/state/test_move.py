import unittest
import smach
from controller_mission.state.move import Move

__author__ = 'Francis Masse'


class TestMainExample(unittest.TestCase):

    def test_bow(self):
        test_sm = smach.StateMachine(outcomes=['succeeded', 'aborted'])
        test_sm.userdata.direction = 'Bow'
        test_sm.userdata.distance = 3
        with test_sm:
            smach.StateMachine.add('MOVE', Move(),
                                   transitions={'succeeded': 'succeeded',
                                                'not_reach': 'MOVE'},
                                   remapping={'direction': 'direction',
                                              'distance_in': 'distance',
                                              'distance_out': 'distance'})
        outcome = test_sm.execute()
        self.assertEqual('succeeded', outcome)

    def test_stern(self):
        test_sm = smach.StateMachine(outcomes=['succeeded', 'aborted'])
        test_sm.userdata.direction = 'Stern'
        test_sm.userdata.distance = 3
        with test_sm:
            smach.StateMachine.add('MOVE', Move(),
                                   transitions={'succeeded': 'succeeded',
                                                'not_reach': 'MOVE'},
                                   remapping={'direction': 'direction',
                                              'distance_in': 'distance',
                                              'distance_out': 'distance'})
        outcome = test_sm.execute()
        self.assertEqual('succeeded', outcome)

    def test_port(self):
        move = Move()
        test_sm = smach.StateMachine(outcomes=['succeeded', 'aborted'])
        test_sm.userdata.direction = 'Port'
        test_sm.userdata.distance = 3
        with test_sm:
            smach.StateMachine.add('MOVE', Move(),
                                   transitions={'succeeded': 'succeeded',
                                                'not_reach': 'MOVE'},
                                   remapping={'direction': 'direction',
                                              'distance_in': 'distance',
                                              'distance_out': 'distance'})
        outcome = test_sm.execute()
        self.assertEqual('succeeded', outcome)

    def test_starboard(self):
        move = Move()
        test_sm = smach.StateMachine(outcomes=['succeeded', 'aborted'])
        test_sm.userdata.direction = 'Starboard'
        test_sm.userdata.distance = 3
        with test_sm:
            smach.StateMachine.add('MOVE', Move(),
                                   transitions={'succeeded': 'succeeded',
                                                'not_reach': 'MOVE'},
                                   remapping={'direction': 'direction',
                                              'distance_in': 'distance',
                                              'distance_out': 'distance'})
        outcome = test_sm.execute()
        self.assertEqual('succeeded', outcome)

    def test_up(self):
        move = Move()
        test_sm = smach.StateMachine(outcomes=['succeeded', 'aborted'])
        test_sm.userdata.direction = 'Up'
        test_sm.userdata.distance = 3
        with test_sm:
            smach.StateMachine.add('MOVE', Move(),
                                   transitions={'succeeded': 'succeeded',
                                                'not_reach': 'MOVE'},
                                   remapping={'direction': 'direction',
                                              'distance_in': 'distance',
                                              'distance_out': 'distance'})
        outcome = test_sm.execute()
        self.assertEqual('succeeded', outcome)

    def test_down(self):
        move = Move()
        test_sm = smach.StateMachine(outcomes=['succeeded', 'aborted'])
        test_sm.userdata.direction = 'Down'
        test_sm.userdata.distance = 3
        with test_sm:
            smach.StateMachine.add('MOVE', Move(),
                                   transitions={'succeeded': 'succeeded',
                                                'not_reach': 'MOVE'},
                                   remapping={'direction': 'direction',
                                              'distance_in': 'distance',
                                              'distance_out': 'distance'})
        outcome = test_sm.execute()
        self.assertEqual('succeeded', outcome)
