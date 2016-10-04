import unittest
import smach
from controller_mission.state.move import Move

__author__ = 'Francis Masse'


class TestMainExample(unittest.TestCase):

    def test_bow(self):
        move = Move()
        test_sm = smach.StateMachine(outcomes=[])
        test_sm.userdata.direction = 'Bow'
        test_sm.userdata.distance_in = 1
        result = move.execute(test_sm.userdata)
        self.assertEqual('outcome1', result)

    def test_stern(self):
        move = Move()
        test_sm = smach.StateMachine(outcomes=[])
        test_sm.userdata.direction = 'Stern'
        test_sm.userdata.distance_in = 1
        result = move.execute(test_sm.userdata)
        self.assertEqual('outcome1', result)

    def test_port(self):
        move = Move()
        test_sm = smach.StateMachine(outcomes=[])
        test_sm.userdata.direction = 'Port'
        test_sm.userdata.distance_in = 1
        result = move.execute(test_sm.userdata)
        self.assertEqual('outcome1', result)

    def test_starboard(self):
        move = Move()
        test_sm = smach.StateMachine(outcomes=[])
        test_sm.userdata.direction = 'Starboard'
        test_sm.userdata.distance_in = 1
        result = move.execute(test_sm.userdata)
        self.assertEqual('outcome1', result)

    def test_down(self):
        move = Move()
        test_sm = smach.StateMachine(outcomes=[])
        test_sm.userdata.direction = 'Down'
        test_sm.userdata.distance_in = 1
        result = move.execute(test_sm.userdata)
        self.assertEqual('outcome1', result)

    def test_up(self):
        move = Move()
        test_sm = smach.StateMachine(outcomes=[])
        test_sm.userdata.direction = 'Up'
        test_sm.userdata.distance_in = 1
        result = move.execute(test_sm.userdata)
        self.assertEqual('outcome1', result)
