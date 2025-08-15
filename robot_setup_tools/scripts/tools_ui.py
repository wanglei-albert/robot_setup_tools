from robot_ctl_node import  RobotCtrlNode
from aruco_read_node import ArucoReadNode

import sys
import os
import yaml
from datetime import datetime

import rospkg
from tf.transformations import euler_from_quaternion

from PyQt5.QtWidgets import QLabel, QLineEdit, QListView, QPushButton,QTextBrowser
from PyQt5.QtWidgets import QHBoxLayout, QVBoxLayout, QGridLayout, QStackedLayout, QSpacerItem
from PyQt5.QtWidgets import QApplication, QWidget, QMainWindow
from PyQt5 import QtCore, uic
from PyQt5.QtGui import QIntValidator, QDoubleValidator

class ToolsUI(QMainWindow):
    def __init__(self):
        super().__init__()

        # prepare related file source
        rospack = rospkg.RosPack()
        self.pkg_path = rospack.get_path('robot_setup_tools')
        self.ui_file = os.path.join(self.pkg_path, 'resources/robot_setup_tools.ui')
        self.config_file = os.path.join(self.pkg_path, 'config/robot_poses_config.yaml')

        # init ui resources
        self.init_ui()

        # load pose config from yaml file
        self.load_config()

        # define robot control nodes
        self.robot_ctrl_node = RobotCtrlNode()
        self.aruco_read_node = ArucoReadNode()

    def log_output(self, msg):
        time_stamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]
        self.tb_log_output.append(f'[{time_stamp}]{msg}')


    def init_ui(self):
        uic.loadUi(self.ui_file, self)
        self.setWindowFlags(QtCore.Qt.WindowCloseButtonHint|QtCore.Qt.WindowMinimizeButtonHint)
        self.setFixedSize(self.width(), self.height())
        
        self.log_output("[INFO]starting to initialize ui")
        
        self.cbx_agv_goals.activated[str].connect(self.on_cbx_agv_goals_index_changed)

        # set validator to accept valid input only
        int_validator = QIntValidator(0, 2048)
        double_validator = QDoubleValidator(100000.0, 100000.0, 5)

        self.agv_goal_x.setValidator(double_validator)
        self.agv_goal_y.setValidator(double_validator)
        self.agv_goal_yaw.setValidator(double_validator)

        self.pick_pose_x.setValidator(double_validator)
        self.pick_pose_y.setValidator(double_validator)
        self.pick_pose_z.setValidator(double_validator)
        self.pick_ori_roll.setValidator(double_validator)
        self.pick_ori_pitch.setValidator(double_validator)
        self.pick_ori_yaw.setValidator(double_validator)

        self.place_pose_x.setValidator(double_validator)
        self.place_pose_y.setValidator(double_validator)
        self.place_pose_z.setValidator(double_validator)
        self.place_ori_roll.setValidator(double_validator)
        self.place_ori_pitch.setValidator(double_validator)
        self.place_ori_yaw.setValidator(double_validator)

        # set clicked event handlers for buttons 
        self.rbtn_nav_by_coord.setChecked(True)
        self.btn_save_agv_goal.clicked.disconnect()
        self.btn_save_agv_goal.clicked.connect(self.on_btn_save_agv_goal_clicked)

        self.btn_send_agv_goal.clicked.disconnect()
        self.btn_send_agv_goal.clicked.connect(self.on_btn_send_agv_goal_clicked)

        self.btn_set_goal_via_read_aruco.clicked.disconnect()
        self.btn_set_goal_via_read_aruco.clicked.connect(self.on_btn_set_goal_via_read_aruco_clicked)
        
        self.btn_save_pick_pose.clicked.disconnect()
        self.btn_save_pick_pose.clicked.connect(self.on_btn_save_pick_pose_clicked)

        self.btn_send_pick_pose.clicked.disconnect()
        self.btn_send_pick_pose.clicked.connect(self.on_btn_send_pick_pose_clicked)

        self.btn_set_pick_pose_to_cur_pose.clicked.disconnect()
        self.btn_set_pick_pose_to_cur_pose.clicked.connect(self.on_btn_set_pick_pose_to_cur_pose_clicked)

        self.btn_save_place_pose.clicked.disconnect()
        self.btn_save_place_pose.clicked.connect(self.on_btn_save_place_pose_clicked)

        self.btn_send_place_pose.clicked.disconnect()
        self.btn_send_place_pose.clicked.connect(self.on_btn_send_place_pose_clicked)

        self.btn_set_place_pose_to_cur_pose.clicked.disconnect()
        self.btn_set_place_pose_to_cur_pose.clicked.connect(self.on_btn_set_place_pose_to_cur_pose_clicked)

        self.log_output("[INFO]finished initializing ui!")


    def load_config(self):
        self.log_output(f"[INFO]loading config from {os.path.basename(self.config_file)}")
        with open(self.config_file, encoding='utf-8') as f:
            self.robot_poses_config = yaml.safe_load(f)

        # loading agv goals config
        for it in self.robot_poses_config['agv']:
            self.cbx_agv_goals.addItem(it['name'])

        self.cbx_agv_goals.setCurrentIndex(0)
        self.on_cbx_agv_goals_index_changed(self.cbx_agv_goals.currentText())


        # load arm pick pose
        picking_pose = self.robot_poses_config['arm']['picking_pose']
        self.pick_pose_x.setText(str(picking_pose['x']))
        self.pick_pose_y.setText(str(picking_pose['y']))
        self.pick_pose_z.setText(str(picking_pose['z']))
        self.pick_ori_roll.setText(str(picking_pose['roll']))
        self.pick_ori_pitch.setText(str(picking_pose['pitch']))
        self.pick_ori_yaw.setText(str(picking_pose['yaw']))


        # load arm place pose
        placing_pose = self.robot_poses_config['arm']['placing_pose']
        self.place_pose_x.setText(str(placing_pose['x']))
        self.place_pose_y.setText(str(placing_pose['y']))
        self.place_pose_z.setText(str(placing_pose['z']))
        self.place_ori_roll.setText(str(placing_pose['roll']))
        self.place_ori_pitch.setText(str(placing_pose['pitch']))
        self.place_ori_yaw.setText(str(placing_pose['yaw']))

        self.log_output("[INFO]finished loading config")

    def on_cbx_agv_goals_index_changed(self, goal_name):
        for agv_goal in self.robot_poses_config['agv']:
            if goal_name != agv_goal['name']:
                continue

            self.agv_goal_x.setText(str(agv_goal['x']))
            self.agv_goal_y.setText(str(agv_goal['y']))
            self.agv_goal_yaw.setText(str(agv_goal['yaw']))


    def on_btn_save_agv_goal_clicked(self):
        self.log_output(f"[INFO]saving current agv goal")
        goal_name = self.cbx_agv_goals.currentText()
        for it in self.robot_poses_config['agv']:
            if goal_name == it['name']:
                it['x'] = float(self.agv_goal_x.text())
                it['y'] = float(self.agv_goal_y.text())
                it['yaw'] = float(self.agv_goal_yaw.text())
                break

        with open(self.config_file, 'w', encoding='utf-8') as f:
            yaml.dump(self.robot_poses_config, f, sort_keys=False, allow_unicode=True)

        self.log_output(f"[INFO]current agv goal saved")

    def on_btn_send_agv_goal_clicked(self):
        self.log_output(f'sending agv goal pose.')
        x = float(self.agv_goal_x.text())
        y = float(self.agv_goal_y.text())
        yaw = float(self.agv_goal_yaw.text())

        nav_by_coord = False
        if self.rbtn_nav_by_coord.isChecked():
            nav_by_coord = True

        status, result = self.robot_ctrl_node.nav_to_goal(x, y, yaw, nav_by_coord)
        status_flag = 'INFO' if status else 'ERR'
        self.log_output(f"[{status_flag}]navigating status: {status}, {result}")


    def on_btn_set_goal_via_read_aruco_clicked(self):
        self.log_output(f"[INFO]command to read aruco markers.")
        # markers = self.aruco_read_node.read_markers()
        markers = [{'id':12}]
        if len(markers) == 0:
            self.log_output("failed on reading markers")
            return
        marker_id_to_read = int(self.cbx_agv_goals.currentText())
        
        read_success = False
        for marker in markers:
            if marker['id'] == marker_id_to_read:
                rotations = euler_from_quaternion([marker['pose']['qx'], marker['pose']['qy'], marker['pose']['qz'], marker['pose']['qw']])
                self.agv_goal_x.setText(str(marker['pose']['x']))
                self.agv_goal_y.setText(str(marker['pose']['y']))
                self.agv_goal_yaw.setText(str(rotations[2]))
                read_success = True

        if not read_success:
            self.log_output(f"[ERR]failed on reading wanted markers, marker id: {marker_id_to_read}")

    def on_btn_save_pick_pose_clicked(self):
        self.log_output(f'[INFO]saving pick pose to file')
        picking_pose = self.robot_poses_config['arm']['picking_pose']
        picking_pose['x'] = float(self.pick_pose_x.text())
        picking_pose['y'] = float(self.pick_pose_y.text())
        picking_pose['z'] = float(self.pick_pose_z.text())
        picking_pose['roll'] = float(self.pick_ori_roll.text())
        picking_pose['pitch'] = float(self.pick_ori_pitch.text())
        picking_pose['yaw'] = float(self.pick_ori_yaw.text())

        with open(self.config_file, 'w', encoding='utf-8') as f:
            yaml.dump(self.robot_poses_config, f, sort_keys=False, allow_unicode=True)
        self.log_output(f'[INFO]finished saving place pose to file')

    def on_btn_send_pick_pose_clicked(self):
        self.log_output(f"[INFO]sending command to move to pick pose")
        x = float(self.pick_pose_x.text())
        y = float(self.pick_pose_y.text())
        z = float(self.pick_pose_x.text())
        roll = float(self.pick_ori_roll.text())
        pitch = float(self.pick_ori_pitch.text())
        yaw = float(self.pick_ori_yaw.text())

        status, result = self.robot_ctrl_node.move_to_pose(x, y, z, roll, pitch, yaw)
        status_flag = 'INFO' if status else 'ERR'
        self.log_output(f"[{status_flag}]{result}")


    def on_btn_set_pick_pose_to_cur_pose_clicked(self):
        self.log_output(f"[INFO]reading current arm pose to set as pick pose")
        succ_flag, result = self.robot_ctrl_node.get_cur_arm_pose()
        if succ_flag:
            self.pick_pose_x.setText(str(result['x']))
            self.pick_pose_y.setText(str(result['y']))
            self.pick_pose_z.setText(str(result['z']))

            self.pick_ori_roll.setText(str(result['rx']))
            self.pick_ori_pitch.setText(str(result['ry']))
            self.pick_ori_yaw.setText(str(result['rz']))
        else:
            self.log_output(f"[ERR]{result}")

    
    def on_btn_save_place_pose_clicked(self):
        self.log_output(f'[INFO]saving place pose to file')
        placing_pose = self.robot_poses_config['arm']['placing_pose']
        placing_pose['x'] = float(self.place_pose_x.text())
        placing_pose['y'] = float(self.place_pose_y.text())
        placing_pose['z'] = float(self.place_pose_z.text())
        placing_pose['roll'] = float(self.place_ori_roll.text())
        placing_pose['pitch'] = float(self.place_ori_pitch.text())
        placing_pose['yaw'] = float(self.place_ori_yaw.text())

        with open(self.config_file, 'w', encoding='utf-8') as f:
            yaml.dump(self.robot_poses_config, f, sort_keys=False, allow_unicode=True)
        self.log_output(f'[INFO]finished saving place pose to file')

    def on_btn_send_place_pose_clicked(self):
        self.log_output(f'[INFO]sending command to move to place pose')
        x = float(self.place_pose_x.text())
        y = float(self.place_pose_y.text())
        z = float(self.place_pose_x.text())
        roll = float(self.place_ori_roll.text())
        pitch = float(self.place_ori_pitch.text())
        yaw = float(self.place_ori_yaw.text())

        status, result =  self.robot_ctrl_node.move_to_pose(x, y, z, roll, pitch, yaw)
        status_flag = 'INFO' if status else 'ERR'
        self.log_output(f"[{status_flag}]{result}")


    def on_btn_set_place_pose_to_cur_pose_clicked(self):
        self.log_output(f'reading current arm pose to set as place pose')
        succ_flag, result = self.robot_ctrl_node.get_cur_arm_pose()
        if succ_flag:
            self.place_pose_x.setText(str(result['x']))
            self.place_pose_y.setText(str(result['y']))
            self.place_pose_z.setText(str(result['z']))

            self.place_ori_roll.setText(str(result['rx']))
            self.place_ori_pitch.setText(str(result['ry']))
            self.place_ori_yaw.setText(str(result['rz']))
        else:
            self.log_output(f"{result}")



if __name__ == '__main__':
    app = QApplication(sys.argv)
    tools_app = ToolsUI()
    tools_app.show()
    app.exec()
