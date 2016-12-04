import rospy
import smach
from nav_msgs.msg import Odometry
from dynamic_reconfigure.server import Server
from controller_mission.cfg import SquareTestConfig
from proc_control.srv import SetPositionTarget

_author_ = 'Francis Masse'


def square_test_cb(config, level):
    rospy.loginfo("""Reconfigure Request: {set_position_x}, {set_position_y}, {set_position_z}, \
                  {set_position_roll}, {set_position_pitch}, {set_position_yaw}, """.format(**config))
    return config


class Move(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['succeeded', 'aborted'],
                             input_keys=['position_reached'])

    def execute(self, ud):
        rate = rospy.Rate(1)
        dynamic_reconf_server = Server(SquareTestConfig, square_test_cb)

        rospy.wait_for_service('/proc_control/set_local_target')
        set_local_target = rospy.ServiceProxy('/proc_control/set_local_target', SetPositionTarget)

        rospy.loginfo('Executing state MOVE')
        while not ud.position_reached:
            try:
                response = set_local_target(dynamic_reconf_server.config.set_position_x,
                                            dynamic_reconf_server.config.set_position_y,
                                            dynamic_reconf_server.config.set_position_z,
                                            dynamic_reconf_server.config.set_position_roll,
                                            dynamic_reconf_server.config.set_position_pitch,
                                            dynamic_reconf_server.config.set_position_yaw)
            except rospy.ServiceException as exc:
                rospy.loginfo('Service did not process request: ' + str(exc))

            rospy.loginfo('Set position x = %f' % dynamic_reconf_server.config.set_position_x)
            rospy.loginfo('Set position y = %f' % dynamic_reconf_server.config.set_position_y)
            rospy.loginfo('Set position z = %f' % dynamic_reconf_server.config.set_position_z)
            rospy.loginfo('Set position roll = %f' % dynamic_reconf_server.config.set_position_roll)
            rospy.loginfo('Set position pitch = %f' % dynamic_reconf_server.config.set_position_pitch)
            rospy.loginfo('Set position yaw = %f' % dynamic_reconf_server.config.set_position_yaw)

            rate.sleep()

        if ud.position_reached:
            return 'succeeded'
        else:
            return 'aborted'
