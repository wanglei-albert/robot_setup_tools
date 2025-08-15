#!/usr/bin/env python

from tools_ui import ToolsUI
from PyQt5.QtWidgets import QApplication
import rospy
import sys

def set_log_level():
    if rospy.has_param('log_level'):
        log_level = rospy.get_param('log_level', default='info')
        print(f'log_level is {log_level}')
    

def main():
    rospy.init_node('robot_setup_tools_node')
    set_log_level()
    
    # main task code
    app = QApplication(sys.argv)
    robot_tools = ToolsUI()
    robot_tools.show()
    app.exec()


if __name__ == '__main__':
    main()