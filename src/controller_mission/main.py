import rospy
import smach
from state.move import Move

_author_ = 'Francis Masse'


def main():
    rospy.init_node('controller_mission')

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['outcome3', 'outcome4'])
    sm.userdata.sm_direction = 0

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('MOVE', Move(),
                               transitions={'outcome1':'outcome3',
                                            'outcome2':'outcome4'},
                               remapping={'direction': 'sm_direction'})

    # Execute SMACH plan
    outcome = sm.execute()


if __name__ == '__main__':
    main()