import rospy
import smach
from state.move import Move

_author_ = 'Francis Masse'


def main():
    rospy.init_node('controller_mission')

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['outcome3', 'outcome4'])
    sm.userdata.sm_direction = 1
    sm.userdata.sm_distance = 5

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('MOVE', Move(),
                               transitions={'outcome1': 'MOVE',
                                            'outcome2': 'outcome3'},
                               remapping={'direction': 'sm_direction',
                                          'distance_in': 'sm_distance',
                                          'distance_out': 'sm_distance'})

    # Execute SMACH plan
    outcome = sm.execute()


if __name__ == '__main__':
    main()