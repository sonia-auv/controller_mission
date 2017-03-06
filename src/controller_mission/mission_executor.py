import rospy
import smach
import smach_ros
import os
import rospkg
import yaml
import threading
import time
# this from state import * is very import !!!
from state import *
from controller_mission.srv import ListMissionsResponse, ListMissions, LoadMission, LoadMissionResponse, StartMission, \
    StartMissionResponse, CurrentMission, \
    CurrentMissionResponse, ReceivedMission, ReceivedMissionResponse, StopMission, StopMissionResponse


class MissionExecutor:
    missions = []
    smach_executor_thread = None
    CONTAINER_NAME = "container_"

    def __init__(self):
        rospy.init_node('mission_executor')
        rp = rospkg.RosPack()
        # Load all missions
        rospy.loginfo('Loading missions list ...')
        self.missions_directory = os.path.join(rp.get_path('controller_mission'), 'missions')
        self.load_missions_file(self.missions_directory)
        rospy.loginfo('Done loading missions list.')

        # Service to get all available mission
        rospy.Service('mission_executor/list_missions', ListMissions, self._handle_list_missions)

        # Service to load a missions
        rospy.Service('mission_executor/load_mission', LoadMission, self._handle_load_missions)

        # Service get current missions
        rospy.Service('mission_executor/current_mission', CurrentMission, self._handle_current_mission)

        # Service to start mission
        rospy.Service('mission_executor/start_mission', StartMission, self._handle_start_mission)
        rospy.Service('mission_executor/stop_mission', StopMission, self._handle_stop_mission)

        # Receive mission content
        rospy.Service('mission_executor/download_mission', ReceivedMission, self._handle_received_mission)

        rospy.spin()

    def _handle_stop_mission(self, req):
        try:
            if self.smach_executor_thread:
                self.current_stateMachine.request_preempt()
                self.smach_executor_thread.join()
                self.smach_executor_thread = None
                rospy.loginfo('Mission stopped')
            return StopMissionResponse()
        except Exception:
            return Exception('Mission not loaded')

    def _handle_received_mission(self, req):

        with open(os.path.join(self.missions_directory, req.name), 'w') as missionfile:
            missionfile.write(req.content)
        return ReceivedMissionResponse()

    def _handle_current_mission(self, req):
        return CurrentMissionResponse(self.current_mission)

    def _handle_start_mission(self, req):
        try:
            if self.smach_executor_thread:
                self._handle_stop_mission(None)

            self.current_stateMachine = self.main_sm;
            rospy.loginfo('Starting mission : {}..'.format(self.current_mission))
            self.smach_executor_thread = threading.Thread(target=self._run_start_mission)
            self.smach_executor_thread.start()
            return StartMissionResponse()
        except Exception:
            return Exception('Mission is not loaded')

    def _run_start_mission(self):
        rospy.loginfo('Mission start in 3 ...')
        time.sleep(1)
        rospy.loginfo('Mission start in 2 ...')
        time.sleep(1)
        rospy.loginfo('Mission start in 1 ...')
        time.sleep(1)
        rospy.loginfo('Mission started')
        self.current_stateMachine.execute()
        self.smach_executor_thread = None

    def _handle_load_missions(self, req):
        mission = req.mission
        states = []
        with open(mission, 'r') as missionfile:
            states = yaml.load(missionfile)
        self.main_sm = smach.StateMachine(['succeeded', 'aborted', 'preempted'])
        self.current_mission = mission

        # Open the container
        with self.main_sm:
            state_to_ignore = []
            container_counter = 1
            # Replace single state with concurrent transitions by concurrent state
            for stateui in states:
                transitions = {}
                self._create_transition_multimap(stateui, transitions)
                self._add_concurrent_state_machine(state_to_ignore, transitions, container_counter)

            # Create all single state machine
            container_counter = 1
            for stateui in states:
                if self._is_state_ignored(state_to_ignore, stateui):
                    continue
                transitions = {}

                self._create_transition_multimap(stateui, transitions)
                # Remove concurrent state
                for k, v in transitions.items():
                    if len(v) > 1:
                        self._replace_transition_with_concurrent_transition(k, transitions, container_counter)

                self.instanciate_single_state(stateui, transitions)

                # set Root state
                if stateui.state.is_root:
                    self.main_sm.set_initial_state([stateui.state.name])

        self.sis = smach_ros.IntrospectionServer('mission_executor_server', self.main_sm, '/mission_executor')
        self.sis.start()
        return LoadMissionResponse()

    def _is_state_ignored(self, state_to_ignore, stateui):
        for ignored_state in state_to_ignore:
            if stateui.state.name == ignored_state:
                return True
        return False

    def _add_concurrent_state_machine(self, state_to_ignore, transitions, container_counter):
        for k, v in transitions.items():
            if len(v) > 1:
                all_concurrent_transition_dict = {}
                all_concurrent_transition = []

                # Get all outcome state
                for state_ui in v:
                    for tran in state_ui.state.transitions:
                        all_concurrent_transition_dict[state_ui.state.name + '_' + tran.outcome] = tran.state
                        all_concurrent_transition.append(state_ui.state.name + '_' + tran.outcome)

                sm_con = smach.Concurrence(outcomes=all_concurrent_transition,
                                           default_outcome=all_concurrent_transition[0],
                                           child_termination_cb=self.child_term_cb,
                                           outcome_cb=self.out_cb)
                # Open the container
                with sm_con:
                    for state_ui in v:
                        exec ('my_state = {}()'.format(state_ui.state._name))
                        for param in state_ui.state.parameters:
                            if isinstance(param.value, basestring):
                                exec ('my_state.{} = \'{}\''.format(param.variable_name, param.value))
                            else:
                                exec ('my_state.{} = {}'.format(param.variable_name, param.value))
                        exec ('smach.Concurrence.add(\'{}\',my_state)'.format(state_ui.state.name))

                rospy.loginfo('Add concurrent state container {}_{} with transitions = {}'.format(self.CONTAINER_NAME,
                                                                                                  container_counter,
                                                                                                  all_concurrent_transition_dict))
                smach.StateMachine.add('{}_{}'.format(self.CONTAINER_NAME, container_counter), sm_con,
                                       transitions=all_concurrent_transition_dict)

                self._replace_transition_with_concurrent_transition(k, transitions, container_counter)
                for transition_state in v:
                    state_to_ignore.append(transition_state.state.name)

    def _replace_transition_with_concurrent_transition(self, k, transitions, container_counter):
        # Remove from dic
        transitions.pop(k)
        transitions[k] = []
        transitions[k].append('{}_{}'.format(self.CONTAINER_NAME, container_counter))
        container_counter += 1

    def _create_transition_multimap(self, stateui, transitions):
        for transition_ui in stateui.transitions:
            if not transitions.has_key(transition_ui.name):
                transitions[transition_ui.name] = []
            transitions[transition_ui.name].append(transition_ui.state2)

    def instanciate_concurent_state(self, stateui, states, state_names):

        pass

    def instanciate_single_state(self, stateui, transition_dict):
        transitions = {}
        for key, val in transition_dict.items():
            if isinstance(val[0], basestring):
                transitions[key] = val[0]
            else:
                transitions[key] = val[0].state.name
        # Instanciate state and set parameter value.
        exec ('s = {}()'.format(stateui.state._name))
        for param in stateui.state.parameters:
            if isinstance(param.value, basestring):
                exec ('s.{} = \'{}\''.format(param.variable_name, param.value))
            else:
                exec ('s.{} = {}'.format(param.variable_name, param.value))

        # For debug purposes
        rospy.loginfo('Add single state {} with transitions={})'.format(stateui.state.name, transitions))
        exec (
            'smach.StateMachine.add(\'{}\',s,transitions={})'.format(stateui.state.name,
                                                                     transitions))

    def child_term_cb(self, outcome_map):
        return True

    def out_cb(self, outcome_map):
        # Return the first result
        for k, v in outcome_map.items():
            if v != 'preempted':
                return k + '_' + v

        return 'preempted'

    def load_missions_file(self, directory):
        for file in os.listdir(directory):
            file_path = os.path.join(directory, file)
            if not os.path.isfile(file_path):
                self.load_missions_file(file_path)
                continue
            self.missions.append(file_path)

    def _handle_list_missions(self, req):
        missions_list = None
        for mission in self.missions:
            if not missions_list:
                missions_list = mission
            else:
                missions_list = missions_list + ',' + mission
        return ListMissionsResponse(missions_list)


# Main function.
if __name__ == '__main__':
    try:
        ne = MissionExecutor()
    except rospy.ROSInterruptException:
        pass
