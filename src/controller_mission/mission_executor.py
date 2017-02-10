import rospy
import smach
import smach_ros
import os
import rospkg
import yaml
import threading
from state import *
from controller_mission.srv import ListMissionsResponse, ListMissions, LoadMission, LoadMissionResponse, StartMission, StartMissionResponse, CurrentMission, \
    CurrentMissionResponse, ReceivedMission, ReceivedMissionResponse,StopMission,StopMissionResponse


class MissionExecutor:
    missions = []
    smach_executor_thread = None

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

    def _handle_stop_mission(self,req):
        try:
            if self.smach_executor_thread:
                self.main_sm.request_preempt()
                self.smach_executor_thread.join()
                self.smach_executor_thread = None
            return StopMissionResponse()
        except Exception:
            return Exception('Mission not loaded')

    def _handle_received_mission(self,req):

        with open(os.path.join(self.missions_directory,req.name), 'w') as missionfile:
            missionfile.write(req.content)
        return ReceivedMissionResponse()

    def _handle_current_mission(self, req):
        return CurrentMissionResponse(self.current_mission)

    def _handle_start_mission(self, req):
        try:
            if self.main_sm.is_running():
                self.main_sm.request_preempt()
            self.main_sm.execute()
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
        # Open the container
        with self.main_sm:
            for stateui in states:
                print stateui.state.name
                transitions = {}

                for transition in stateui.state.transitions:
                    transitions[transition.outcome] = transition.state
                print 'smach.StateMachine.add(\'{}\',{}(),transitions={})'.format(stateui.state.name,
                                                                                  stateui.state._name, transitions)
                exec('s = {}()'.format(stateui.state._name))
                for param in stateui.state.parameters:
                    if isinstance(param.value, basestring):
                        exec ('s.{} = \'{}\''.format(param.variable_name,param.value))
                    else:
                        exec('s.{} = {}'.format(param.variable_name, param.value))
                exec (
                'smach.StateMachine.add(\'{}\',s,transitions={})'.format(stateui.state.name,
                                                                            transitions))

                # smach.StateMachine.add('MOVE_PORT',
                #                       Move(dynamic_reconf_server, 'port'),
                #                       transitions={'succeeded': 'MOVE_BACKWARD',
                #                                    'position_not_reach': 'MOVE_PORT',
                #                                    'aborted': 'aborted'})

        self.sis = smach_ros.IntrospectionServer('mission_executor_server', self.main_sm, '/mission_executor')
        self.sis.start()
        return LoadMissionResponse()

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
