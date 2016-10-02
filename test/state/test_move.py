import unittest
import smach
from controller_mission.state.move import Move

__author__ = 'Francis Masse'


class TestMainExample(unittest.TestCase):

    def test_surge(self):
        move = Move()
        test_sm = smach.StateMachine(outcomes=[])
        test_sm.userdata.direction = 1
        test_sm.userdata.distance_in = 1
        result = move.execute(test_sm.userdata)
        self.assertEqual('outcome1', result)

    def test_sway(self):
        move = Move()
        test_sm = smach.StateMachine(outcomes=[])
        test_sm.userdata.direction = 2
        test_sm.userdata.distance_in = 1
        result = move.execute(test_sm.userdata)
        self.assertEqual('outcome1', result)

