import unittest
from controller_mission.state.move import Move

__author__ = 'Francis Masse'


class TestMainExample(unittest.TestCase):

    def test_move_forward(self):
        move = Move()
        result = move.execute(1)
        self.assertEqual('outcome1', result)
