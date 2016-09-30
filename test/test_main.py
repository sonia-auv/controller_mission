import unittest
from controller_mission.main import Calculator

__author__ = 'Francis Masse'


class TestMainExample(unittest.TestCase):

    def test_main_add_method(self):
        calculator = Calculator()
        result = calculator.add(2, 2)
        self.assertEqual(4, result)

