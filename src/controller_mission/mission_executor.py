#! /usr/bin/env python2

import rospy
import smach
import smach_ros
import os
import rospkg
import yaml
import threading
import time
# this from state import * is very important !!!
from state import *

from std_msgs.msg import String
# from controller_mission.src.controller_mission import param_submission

from std_msgs.msg import String
from controller_mission.srv import ListMissionsResponse, ListMissions, LoadMission, LoadMissionResponse, \
    LoadMissionRequest, StartMission, \
    StartMissionResponse, CurrentMission, \
    CurrentMissionResponse, ReceivedMission, ReceivedMissionResponse, StopMission, StopMissionResponse, SendMission, \
    SendMissionResponse, ReceivedState, ReceivedStateResponse

from provider_kill_mission.msg import MissionSwitchMsg


class MissionExecutor:
    missions = []
    current_mission = None
    smach_executor_thread = None
    CONTAINER_NAME = "container"
    machine_state_has_run = False

    def __init__(self):
        rospy.init_node('mission_executor')
        rp = rospkg.RosPack()
        # Load all missions
        rospy.loginfo('Loading missions list ...')
        self.missions_directory = os.path.join(rp.get_path('controller_mission'), 'missions')
        self.controller_mission_directory = os.path.join(rp.get_path('controller_mission'))
        self.load_missions_file(self.missions_directory)
        rospy.loginfo('Done loading missions list.')

        # Service to get all available mission
        rospy.Service('mission_executor/list_missions', ListMissions, self._handle_list_missions)

        # Service to load a missions
        rospy.Service('mission_executor/load_mission', LoadMission, self._handle_load_missions)

        # Service get current missions
        self.mission_loaded_changed_publisher = rospy.Publisher('/mission_executor/mission_loaded_name', String,
                                                                queue_size=5)
        self.started_mission_name = rospy.Publisher('/mission_executor/started_mission_name', String, queue_size=5)
        self.end_mission = rospy.Publisher('/mission_executor/mission_ended', String, queue_size=5)

        self._mission_switch_sub = rospy.Subscriber('/provider_kill_mission/mission_switch_msg', MissionSwitchMsg,
                                                    self._handle_mission_switch_activated)

        # Service to start mission
        rospy.Service('mission_executor/start_mission', StartMission, self._handle_start_mission)
        rospy.Service('mission_executor/current_mission', CurrentMission, self._handle_current_mission)
        rospy.Service('mission_executor/stop_mission', StopMission, self._handle_stop_mission)

        # Download state content
        rospy.Service('mission_executor/push_state_content', ReceivedState, self._handle_state_received)
        # Download mission content
        rospy.Service('mission_executor/set_mission_content', ReceivedMission, self._handle_received_mission)
        # Receive mission content
        rospy.Service('mission_executor/get_mission_content', SendMission, self._handle_send_mission)

        rospy.spin()

    def _handle_state_received(self, req):
        with open(self.controller_mission_directory + req.name, 'w') as myfile:
            myfile.write(req.content)
        return ReceivedStateResponse()

    def _handle_current_mission(self, req):
        return CurrentMissionResponse(self.current_mission)

    def _handle_mission_switch_activated(self, msg):
        if self.current_mission:
            if msg.state:
                self._handle_start_mission(None)
            else:
                self._handle_stop_mission(None)

    def _handle_stop_mission(self, req):
        try:
            if self.smach_executor_thread and self.current_stateMachine.is_running():
                self.sis.stop()
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

        if self.current_mission:
            if self.current_mission == req.name:
                self._handle_load_missions(LoadMissionRequest(req.name))

        return ReceivedMissionResponse()

    def _handle_send_mission(self, req):
        response = SendMissionResponse()
        with open(os.path.join(self.missions_directory, req.name), 'r') as missionfile:
            response.content = missionfile.read()
        return response

    def _handle_start_mission(self, req):
        if self.smach_executor_thread:
            self._handle_stop_mission(None)
        if self.machine_state_has_run:
            self.machine_state_has_run = False
            self.main_sm = self.reload_state_machine()
        self.current_stateMachine = self.main_sm
        rospy.loginfo('Starting mission : {}..'.format(self.current_mission))
        self.started_mission_name.publish(self.current_mission)

        self.sis = smach_ros.IntrospectionServer('mission_executor_server', self.current_stateMachine,
                                                 '/mission_executor')
        self.sis.start()
        self.smach_executor_thread = threading.Thread(target=self._run_start_mission)
        self.smach_executor_thread.start()
        self.machine_state_has_run = True
        return StartMissionResponse()

    def reload_state_machine(self):
        mission = self.missions_directory + '/' + self.current_mission
        return self.create_state_machine(mission, 'main', None)

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
        self.sis.stop()
        self.end_mission.publish(self.current_mission)

    def _handle_load_missions(self, req):
        self.current_mission = req.mission
        self.mission_loaded_changed_publisher.publish(self.current_mission)
        mission = self.missions_directory + '/' + req.mission
        self.main_sm = self.create_state_machine(mission, 'main', None)

        return LoadMissionResponse()

    def create_state_machine(self, mission, sub_mission_name, global_params):
        with open(mission, 'r') as missionfile:
            mission_container = yaml.load(missionfile)
        main_sm = smach.StateMachine(['succeeded', 'aborted', 'preempted'],
                                     input_keys=['generic_data_field_1', 'generic_data_field_2', 'generic_data_field_3',
                                                 'generic_data_field_4', 'generic_data_field_5',
                                                 'generic_data_field_6'],
                                     output_keys=['generic_data_field_1', 'generic_data_field_2',
                                                  'generic_data_field_3',
                                                  'generic_data_field_4', 'generic_data_field_5',
                                                  'generic_data_field_6'])
        main_sm.userdata.generic_data_field_1 = ""
        main_sm.userdata.generic_data_field_2 = ""
        main_sm.userdata.generic_data_field_3 = ""
        main_sm.userdata.generic_data_field_4 = ""
        main_sm.userdata.generic_data_field_5 = ""
        main_sm.userdata.generic_data_field_6 = ""
        # Open the container
        with main_sm:
            state_to_ignore = []
            container_counter = 1

            for globalparam in mission_container.globalparams:
                exec ('self.{}_{} = {}'.format('main', globalparam.variable_name, globalparam.value))

            # Replace single state with concurrent transitions by concurrent state
            for stateui in mission_container.statesui:
                transitions = {}
                self._create_transition_multimap(stateui, transitions)

                for k, v in transitions.items():
                    if len(v) > 1:
                        self._add_concurrent_state_machine(k, v, state_to_ignore, transitions,
                                                           self.CONTAINER_NAME + '|' + str(container_counter),
                                                           sub_mission_name)
                        container_counter += 1

            # Create all single state machine
            if global_params:
                for global_param in global_params:
                    print '{}_{} = {}'.format(sub_mission_name.replace('|', '_'), global_param.variable_name,
                                              global_param.value)
                    exec ('self.{}_{} = {}'.format(sub_mission_name.replace('|', '_'), global_param.variable_name,
                                                   global_param.value))
            container_counter = 1
            submission_counter = 1
            for stateui in mission_container.statesui:
                if self._is_state_ignored(state_to_ignore, stateui):
                    continue
                transitions = {}

                self._create_transition_multimap(stateui, transitions)
                # Remove concurrent state
                for k, v in transitions.items():
                    if len(v) > 1:
                        self._replace_transition_with_concurrent_transition(k, transitions,
                                                                            self.CONTAINER_NAME + '|' + str(
                                                                                container_counter), sub_mission_name)
                        container_counter += 1

                if stateui.state.is_submission:
                    self.instanciate_submission_state(stateui, transitions,
                                                      sub_mission_name)
                    submission_counter += 1
                else:
                    self.instanciate_single_state(stateui, transitions, sub_mission_name, mission_container)

                # set Root state
                if stateui.state.is_root:
                    main_sm.set_initial_state([sub_mission_name + '|' + stateui.state.name])
        return main_sm

    def _is_state_ignored(self, state_to_ignore, stateui):
        for ignored_state in state_to_ignore:
            if stateui.state.name == ignored_state:
                return True
        return False

    def _add_concurrent_state_machine(self, k, v, state_to_ignore, transitions, container_name, sub_mission_name):

        all_concurrent_transition_dict = {}
        all_concurrent_transition = []

        # Get all outcome state
        for state_ui in v:
            for tran in state_ui.state.transitions:
                all_concurrent_transition_dict[
                    sub_mission_name + '|' + state_ui.state.name + '|' + tran.outcome] = sub_mission_name + '|' + tran.state
                all_concurrent_transition.append(sub_mission_name + '|' + state_ui.state.name + '|' + tran.outcome)
        default_outcome = 'succeeded'
        if len(all_concurrent_transition) > 0:
            default_outcome = all_concurrent_transition[0]
        sm_con = smach.Concurrence(outcomes=all_concurrent_transition,
                                   default_outcome=default_outcome,
                                   child_termination_cb=self.child_term_cb,
                                   outcome_cb=self.out_cb,
                                   input_keys=['generic_data_field_1', 'generic_data_field_2', 'generic_data_field_3',
                                               'generic_data_field_4', 'generic_data_field_5', 'generic_data_field_6'],
                                   output_keys=['generic_data_field_1', 'generic_data_field_2', 'generic_data_field_3',
                                                'generic_data_field_4', 'generic_data_field_5', 'generic_data_field_6'])
        # Open the container
        with sm_con:
            for state_ui in v:
                if state_ui.state.is_submission:

                    sub_state = self.create_state_machine(
                        os.path.join(self.missions_directory, state_ui.state.submission_file),
                        sub_mission_name + '|' + state_ui.state.name, state_ui.state.global_params)
                    smach.Concurrence.add(sub_mission_name + '|' + state_ui.state.name, sub_state)
                else:
                    exec ('my_state = {}()'.format(state_ui.state._name))
                    for param in state_ui.state.parameters:
                        if isinstance(param.value, basestring):
                            exec ('my_state.{} = \'{}\''.format(param.variable_name, param.value))
                        else:
                            exec ('my_state.{} = {}'.format(param.variable_name, param.value))
                    exec ('smach.Concurrence.add(\'{}\',my_state)'.format(sub_mission_name + '|' + state_ui.state.name))

        rospy.loginfo('Add concurrent state container {} with transitions = {}'.format(container_name,
                                                                                       all_concurrent_transition_dict))
        smach.StateMachine.add(sub_mission_name + '|' + container_name, sm_con,
                               transitions=all_concurrent_transition_dict,remapping={'generic_data_field_1':'generic_data_field_1','generic_data_field_2':'generic_data_field_2','generic_data_field_3':'generic_data_field_3','generic_data_field_4':'generic_data_field_4','generic_data_field_5':'generic_data_field_5','generic_data_field_6':'generic_data_field_6'})

        self._replace_transition_with_concurrent_transition(k, transitions, sub_mission_name + '|' + container_name,
                                                            sub_mission_name)
        for transition_state in v:
            state_to_ignore.append(sub_mission_name + '|' + transition_state.state.name)

    def _replace_transition_with_concurrent_transition(self, k, transitions, container_name, sub_mission_name):
        # Remove from dic
        transitions.pop(k)
        transitions[k] = []
        transitions[k].append(sub_mission_name + '|' + container_name)

    def _create_transition_multimap(self, stateui, transitions):
        for transition_ui in stateui.transitions:
            if not transitions.has_key(transition_ui.name):
                transitions[transition_ui.name] = []
            transitions[transition_ui.name].append(transition_ui.state2)

    def instanciate_single_state(self, stateui, transition_dict, sub_mission_name, mission_container):
        transitions = {}
        for key, val in transition_dict.items():
            if isinstance(val[0], basestring):
                transitions[key] = val[0]
            else:
                transitions[key] = sub_mission_name + '|' + val[0].state.name
        # print stateui.state._name
        # Instanciate state and set parameter value.
        exec ('s = {}()'.format(stateui.state._name))
        print s
        for param in stateui.state.parameters:
            value_is_param = False
            for global_param in mission_container.globalparams:
                if isinstance(param.value, basestring):
                    if param.value == global_param.variable_name:
                        value_is_param = True
                        print sub_mission_name
                        exec ('s.{} = self.{}_{}'.format(param.variable_name, sub_mission_name.replace('|', '_'),
                                                         param.value))

            if isinstance(param.value, basestring) and not value_is_param:
                exec ('s.{} = \'{}\''.format(param.variable_name, param.value))
            elif not value_is_param:
                exec ('s.{} = {}'.format(param.variable_name, param.value))

        rospy.loginfo(
            'Add single state {} with transitions={})'.format(sub_mission_name + '|' + stateui.state.name, transitions))

        exec (
            'smach.StateMachine.add(\'{}\',s,transitions={}, remapping={})'.format(
                sub_mission_name + '|' + stateui.state.name,
                transitions,
                "{'generic_data_field_1':'generic_data_field_1','generic_data_field_2':'generic_data_field_2','generic_data_field_3':'generic_data_field_3','generic_data_field_4':'generic_data_field_4','generic_data_field_5':'generic_data_field_5','generic_data_field_6':'generic_data_field_6'}"))

    def instanciate_submission_state(self, stateui, transition_dict, sub_mission_name):
        transitions = {}
        for key, val in transition_dict.items():
            if isinstance(val[0], basestring):
                transitions[key] = val[0]
            else:
                transitions[key] = sub_mission_name + '|' + val[0].state.name

        rospy.loginfo('Add submission state {} with transitions={})'.format(stateui.state.name, transitions))

        state_machine = self.create_state_machine(
            os.path.join(self.missions_directory, stateui.state.submission_file),
            sub_mission_name + '|' + stateui.state.name, stateui.state.global_params
        )
        smach.StateMachine.add(sub_mission_name + '|' + stateui.state.name, state_machine, transitions,remapping={'generic_data_field_1':'generic_data_field_1','generic_data_field_2':'generic_data_field_2','generic_data_field_3':'generic_data_field_3','generic_data_field_4':'generic_data_field_4','generic_data_field_5':'generic_data_field_5','generic_data_field_6':'generic_data_field_6'})

    def child_term_cb(self, outcome_map):
        return True

    def out_cb(self, outcome_map):
        # Return the first result
        for k, v in outcome_map.items():
            if v != 'preempted':
                return k + '|' + v

        return 'preempted'

    def load_missions_file(self, directory):
        for file in os.listdir(directory):
            file_path = os.path.join(directory, file)
            if not os.path.isfile(file_path):
                self.load_missions_file(file_path)
                continue
            self.missions.append(file_path.replace(self.missions_directory + '/', ''))

    def _handle_list_missions(self, req):
        self.missions = []
        self.load_missions_file(self.missions_directory)
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
