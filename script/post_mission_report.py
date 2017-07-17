#!/usr/bin/env python
import os
import time
import rospkg
import rospy
from datetime import datetime

from rosgraph_msgs.msg import Log


class PostMissionReport:

    def __init__(self):
        self.first_time = None
        self.report = None
        rospy.init_node('report', anonymous=True)
        self.get_log_info = rospy.Subscriber('/rosout_agg', Log, self.log_mission_info)
        rospy.spin()
        self.report.close()

    def log_mission_info(self, log_msg):
        if log_msg.name == '/missionExecutor' or log_msg.name == '/proc_image_processing':
            self.parse_log_msg(log_msg)

    def parse_log_msg(self, log_msg):
        if log_msg.msg[:16] == 'Starting mission':
            self.create_file(log_msg.msg)

        if self.first_time is None and log_msg.msg == 'Mission started':
            self.first_time = datetime.now()
        if self.first_time is not None:
            msg = log_msg.msg
            now_time = datetime.now() - self.first_time
            now_time = str(now_time.seconds / 60) + ':' + str(now_time.seconds % 60)
            if msg[:39] == 'State machine starting in initial state':
                msg = msg[:-90]

            msg = '[' + str(now_time) + '] ' + str(msg)
            self.report.write(str(msg) + "\n")
        if log_msg.msg == 'Mission stopped':
            self.first_time = None
            self.close_file()
            return

    def create_file(self, line):
        mission_name = line.replace('Starting mission : ', '').replace('/', '-')
        rp = rospkg.RosPack()
        path = os.path.join(rp.get_path('controller_mission'), 'report')
        report_file = path + '/' + mission_name[:-6] + '-mission-report-' + time.strftime("%d-%m-%Y") + '-' + time.strftime("%H-%M-%S") + '.txt'
        self.report = open(str(report_file), 'w')

    def close_file(self):
        self.report.close()

if __name__ == '__main__':
    report = PostMissionReport()
