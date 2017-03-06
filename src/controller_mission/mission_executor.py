import rospy
import smach
import smach_ros
import os
import rospkg
import yaml
import threading
from state import *
from collections import defaultdict
from controller_mission.srv import ListMissionsResponse, ListMissions, LoadMission, LoadMissionResponse, StartMission, \
    StartMissionResponse, CurrentMission, \
    CurrentMissionResponse, ReceivedMission, ReceivedMissionResponse, StopMission, StopMissionResponse


class MissionExecutor:
    missions = []
    smach_executor_thread = None
    CONTAINER_NAME = "container_"

    def __init__(self):
        rospy.init_node('mission_executor')
        # TODO: Load all missions
        rp = rospkg.RosPack()
        rospy.loginfo('Loading missions ...')
        self.missions_directory = os.path.join(rp.get_path('controller_mission'), 'missions')
        self.load_missions_file(self.missions_directory)
        rospy.loginfo('Done loading missions.')
        # TODO: Service to get all available mission
        rospy.Service('mission_executor/list_missions', ListMissions, self._handle_list_missions)
        # TODO: Service to load a missions
        rospy.Service('mission_executor/load_mission', LoadMission, self._handle_load_missions)

        # TODO: Service get current missions
        rospy.Service('mission_executor/current_mission', CurrentMission, self._handle_current_mission)

        # TODO: Service to start mission
        rospy.Service('mission_executor/start_mission', StartMission, self._handle_start_mission)
        rospy.Service('mission_executor/stop_mission', StopMission, self._handle_stop_mission)

        # TODO: Receive mission content
        rospy.Service('mission_executor/download_mission', ReceivedMission, self._handle_received_mission)

        rospy.spin()

    def _handle_stop_mission(self, req):
        try:
            if self.smach_executor_thread:
                print 'request STOP'
                self.current_stateMachine.request_preempt()
                self.smach_executor_thread.join()
                self.smach_executor_thread = None
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
            self.smach_executor_thread = threading.Thread(target=self.current_stateMachine.execute)
            self.smach_executor_thread.start()
            return StartMissionResponse()
        except Exception:
            return Exception('Mission is not loaded')

    def _handle_load_missions(self, req):
        mission = req.mission
        states = []
        with open(mission, 'r') as missionfile:
            states = yaml.load(missionfile)
        self.main_sm = smach.StateMachine(['succeeded', 'aborted'])
        self.current_mission = mission
        self.container_counter = 1

        # Open the container
        with self.main_sm:
            state_to_ignore = []
            for stateui in states:
                transitions = {}

                for transition_ui in stateui.transitions:
                    if not transitions.has_key(transition_ui.name):
                        transitions[transition_ui.name] = []
                    transitions[transition_ui.name].append(transition_ui.state2)

                # TODO: Decide which state we have
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
                                exec('smach.Concurrence.add(\'{}\',my_state)'.format(state_ui.state.name))

                        print 'smach.StateMachine.add(\'{}_{}\', sm_con,transitions={})'.format(self.CONTAINER_NAME,self.container_counter,all_concurrent_transition_dict)
                        smach.StateMachine.add('{}_{}'.format(self.CONTAINER_NAME,self.container_counter), sm_con,
                                               transitions=all_concurrent_transition_dict)


                        # Remove from dic
                        transitions.pop(k)
                        transitions[k] = []
                        transitions[k].append('{}_{}'.format(self.CONTAINER_NAME,self.container_counter))
                        self.container_counter += 1
                        for transition_state in v:
                            state_to_ignore.append(transition_state.state.name)
                        print state_to_ignore


            self.container_counter =1
            for stateui in states:
                skip_state = False
                for ignored_state in state_to_ignore:
                    if stateui.state.name == ignored_state:
                        skip_state = True
                if skip_state:
                    continue
                transitions = {}

                for transition_ui in stateui.transitions:
                    if not transitions.has_key(transition_ui.name):
                        transitions[transition_ui.name] = []
                    transitions[transition_ui.name].append(transition_ui.state2)

                # TODO: Decide which state we have
                for k, v in transitions.items():
                    if len(v) > 1:
                        # Remove from dic
                        transitions.pop(k)
                        transitions[k] = []
                        transitions[k].append('{}_{}'.format(self.CONTAINER_NAME, self.container_counter))
                        self.container_counter += 1
                print 'transitions passed ', transitions
                self.instanciate_single_state(stateui,transitions)
                if stateui.state.is_root:
                    self.main_sm.set_initial_state([stateui.state.name])
        self.sis = smach_ros.IntrospectionServer('mission_executor_server', self.main_sm, '/mission_executor')
        self.sis.start()
        return LoadMissionResponse()

    def instanciate_concurent_state(self, stateui, states, state_names):

        pass

    def instanciate_single_state(self, stateui,transition_dict):
        transitions = {}
        for key, val in transition_dict.items():
            if isinstance(val[0], basestring):
                print 'set ', key, ' to ', val
                transitions[key] = val[0]
            else :
                transitions[key] = val[0].state.name
        # Instanciate state and set parameter value.
        exec ('s = {}()'.format(stateui.state._name))
        for param in stateui.state.parameters:
            if isinstance(param.value, basestring):
                exec ('s.{} = \'{}\''.format(param.variable_name, param.value))
            else:
                exec ('s.{} = {}'.format(param.variable_name, param.value))

        # For debug purposes
        print 'smach.StateMachine.add(\'{}\',{}(),transitions={})'.format(stateui.state.name,
                                                                          stateui.state._name,
                                                                          transitions)
        exec (
            'smach.StateMachine.add(\'{}\',s,transitions={})'.format(stateui.state.name,
                                                                     transitions))

    def child_term_cb(self,outcome_map):
        return True

    def out_cb(self,outcome_map):
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
            print mission
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
