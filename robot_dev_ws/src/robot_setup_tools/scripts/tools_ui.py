import sys
from PyQt5.QtWidgets import QLabel, QLineEdit, QListView, QPushButton,QTextBrowser
from PyQt5.QtWidgets import QHBoxLayout, QVBoxLayout, QGridLayout, QStackedLayout, QSpacerItem
from PyQt5.QtWidgets import QApplication, QWidget, QMainWindow
from PyQt5 import QtCore, uic
import os
import yaml
import rospkg
from robot_ctl_node import  RobotCtrlNode
from aruco_read_node import ArucoReadNode


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


    def init_ui(self):
        uic.loadUi(self.ui_file, self)
        self.setWindowFlags(QtCore.Qt.WindowCloseButtonHint|QtCore.Qt.WindowMinimizeButtonHint)
        self.setFixedSize(self.width(), self.height())
        self.cbx_agv_goals.activated[str].connect(self.on_cbx_agv_goals_index_changed)

        self.rbtn_nav_by_coord.setChecked(True)
        self.btn_save_agv_goal.clicked.disconnect()
        self.btn_save_agv_goal.clicked.connect(self.on_btn_save_agv_goal_clicked)

        self.btn_send_agv_goal.clicked.disconnect()
        self.btn_send_agv_goal.clicked.connect(self.on_btn_send_agv_goal_clicked)
        
        self.btn_save_pick_pose.clicked.disconnect()
        self.btn_save_pick_pose.clicked.connect(self.on_btn_save_pick_pose_clicked)

        self.btn_send_pick_pose.clicked.disconnect()
        self.btn_send_pick_pose.clicked.connect(self.on_btn_send_pick_pose_clicked)

        self.btn_set_pick_pose_via_read_aruco.clicked.disconnect()
        self.btn_set_pick_pose_via_read_aruco.clicked.connect(self.on_btn_set_pick_pose_via_read_aruco_clicked)

        self.btn_set_pick_pose_to_cur_pose.clicked.disconnect()
        self.btn_set_pick_pose_to_cur_pose.clicked.connect(self.on_btn_set_pick_pose_to_cur_pose_clicked)

        self.btn_save_place_pose.clicked.disconnect()
        self.btn_save_place_pose.clicked.connect(self.on_btn_save_place_pose_clicked)

        self.btn_send_place_pose.clicked.disconnect()
        self.btn_send_place_pose.clicked.connect(self.on_btn_send_place_pose_clicked)

        self.btn_set_place_pose_via_read_aruco.clicked.disconnect()
        self.btn_set_place_pose_via_read_aruco.clicked.connect(self.on_btn_set_place_pose_via_read_aruco_clicked)
        self.btn_set_place_pose_via_read_aruco.hide()

        self.btn_set_place_pose_to_cur_pose.clicked.disconnect()
        self.btn_set_place_pose_to_cur_pose.clicked.connect(self.on_btn_set_place_pose_to_cur_pose_clicked)


    def load_config(self):
        with open(self.config_file, encoding='utf-8') as f:
            self.robot_poses_config = yaml.safe_load(f)

        # loading agv goals config
        for it in self.robot_poses_config['agv']:
            print(it['name'])
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


    def on_cbx_agv_goals_index_changed(self, goal_name):
        for agv_goal in self.robot_poses_config['agv']:
            if goal_name != agv_goal['name']:
                continue

            self.agv_goal_x.setText(str(agv_goal['x']))
            self.agv_goal_y.setText(str(agv_goal['y']))
            self.agv_goal_yaw.setText(str(agv_goal['yaw']))


    def on_btn_save_agv_goal_clicked(self):
        print(f'btn_save_agv_goal clicked')
        goal_name = self.cbx_agv_goals.currentText()
        for it in self.robot_poses_config['agv']:
            if goal_name == it['name']:
                it['x'] = float(self.agv_goal_x.text())
                it['y'] = float(self.agv_goal_y.text())
                it['yaw'] = float(self.agv_goal_yaw.text())

                break

        with open(self.config_file, 'w', encoding='utf-8') as f:
            yaml.dump(self.robot_poses_config, f, sort_keys=False, allow_unicode=True)



    def on_btn_send_agv_goal_clicked(self):
        print(f'btn_send_agv_goal clicked')
        x = float(self.agv_goal_x.text())
        y = float(self.agv_goal_y.text())
        yaw = float(self.agv_goal_yaw.text())

        nav_by_coord = False
        if self.rbtn_nav_by_coord.isChecked():
            nav_by_coord = True

        self.robot_ctrl_node.nav_to_goal(x, y, yaw, nav_by_coord)


    def on_btn_save_pick_pose_clicked(self):
        print(f'btn_save_pick_pose clicked')
        picking_pose = self.robot_poses_config['arm']['picking_pose']
        picking_pose['x'] = float(self.pick_pose_x.text())
        picking_pose['y'] = float(self.pick_pose_y.text())
        picking_pose['z'] = float(self.pick_pose_z.text())
        picking_pose['roll'] = float(self.pick_ori_roll.text())
        picking_pose['pitch'] = float(self.pick_ori_pitch.text())
        picking_pose['yaw'] = float(self.pick_ori_yaw.text())

        with open(self.config_file, 'w', encoding='utf-8') as f:
            yaml.dump(self.robot_poses_config, f, sort_keys=False, allow_unicode=True)


    def on_btn_send_pick_pose_clicked(self):
        print(f'btn_send_pick_pose clicked')
        x = float(self.pick_pose_x.text())
        y = float(self.pick_pose_y.text())
        z = float(self.pick_pose_x.text())
        roll = float(self.pick_ori_roll.text())
        pitch = float(self.pick_ori_pitch.text())
        yaw = float(self.pick_ori_yaw.text())

        self.robot_ctrl_node.move_to_pick_pose(x, y, z, roll, pitch, yaw)


    def on_btn_set_pick_pose_via_read_aruco_clicked(self):
        print(f'btn_set_pick_pose_via_read_aruco clicked')

    def on_btn_set_pick_pose_to_cur_pose_clicked(self):
        print(f'btn_set_pick_pose_to_cur_pose clicked')
    
    def on_btn_save_place_pose_clicked(self):
        print(f'btn_save_place_pose clicked')
        placing_pose = self.robot_poses_config['arm']['placing_pose']
        placing_pose['x'] = float(self.place_pose_x.text())
        placing_pose['y'] = float(self.place_pose_y.text())
        placing_pose['z'] = float(self.place_pose_z.text())
        placing_pose['roll'] = float(self.place_ori_roll.text())
        placing_pose['pitch'] = float(self.place_ori_pitch.text())
        placing_pose['yaw'] = float(self.place_ori_yaw.text())

        with open(self.config_file, 'w', encoding='utf-8') as f:
            yaml.dump(self.robot_poses_config, f, sort_keys=False, allow_unicode=True)


    def on_btn_send_place_pose_clicked(self):
        print(f'btn_send_place_pose clicked')
        x = float(self.place_pose_x.text())
        y = float(self.place_pose_y.text())
        z = float(self.place_pose_x.text())
        roll = float(self.place_ori_roll.text())
        pitch = float(self.place_ori_pitch.text())
        yaw = float(self.place_ori_yaw.text())

        self.robot_ctrl_node.move_to_place_pose(x, y, z, roll, pitch, yaw)


    def on_btn_set_place_pose_via_read_aruco_clicked(self):
        print(f'btn_set_place_pose_via_read_aruco clicked')

    def on_btn_set_place_pose_to_cur_pose_clicked(self):
        print(f'btn_set_place_pose_to_cur_pose clicked')


    # def init_ui(self):

    #     # self.app = QApplication(sys.argv)
    #     self.setWindowTitle('机器人部署调试工具')
        
    #     self.main_widget = QWidget()
    #     self.setCentralWidget(self.main_widget)
    #     self.main_layout = QVBoxLayout()
    #     self.main_layout.setObjectName('g_layout')
    #     self.main_widget.setLayout(self.main_layout)


    #     self.body_layout = QGridLayout()
    #     self.main_layout.addLayout(self.body_layout)


    #     # -> agv goal group
    #     self.agv_layout = QVBoxLayout()
    #     self.body_layout.addLayout(self.agv_layout, 0, 0)
    #     self.agv_group_label = QLabel('导航目标点')
    #     self.agv_group_line_label = QLabel('----------------------------------------')
    #     self.agv_goal_edit_layout = QGridLayout()
    #     self.agv_layout.addWidget(self.agv_group_label)
    #     self.agv_layout.addWidget(self.agv_group_line_label)
    #     self.agv_layout.addLayout(self.agv_goal_edit_layout)

    #     self.agv_goal_x_label = QLabel('x')
    #     self.agv_goal_x_line_edit = QLineEdit()
    #     self.agv_goal_y_label = QLabel('y')
    #     self.agv_goal_y_line_edit = QLineEdit()
    #     self.agv_goal_yaw_label = QLabel('yaw')
    #     self.agv_goal_yaw_line_edit = QLineEdit()
        
    #     self.agv_goal_save_btn = QPushButton('重置')
    #     self.agv_goal_send_btn = QPushButton('发送')

    #     self.agv_goal_edit_layout.addWidget(self.agv_goal_x_label, 0, 1)
    #     self.agv_goal_edit_layout.addWidget(self.agv_goal_x_line_edit, 0, 2)
    #     self.agv_goal_edit_layout.addWidget(self.agv_goal_y_label, 1, 1)
    #     self.agv_goal_edit_layout.addWidget(self.agv_goal_y_line_edit, 1, 2)
    #     self.agv_goal_edit_layout.addWidget(self.agv_goal_yaw_label, 2, 1)
    #     self.agv_goal_edit_layout.addWidget(self.agv_goal_yaw_line_edit, 2, 2)
    #     self.agv_goal_edit_layout.addWidget(self.agv_goal_save_btn, 3, 3)
    #     self.agv_goal_edit_layout.addWidget(self.agv_goal_send_btn, 3, 4)
    #     # self.agv_goal_edit_layout.addWidget(QSpacerItem())


    #     # -> arm pick pose group
    #     self.arm_pick_layout = QVBoxLayout()
    #     self.body_layout.addLayout(self.arm_pick_layout, 0, 1)
    #     self.arm_pick_group_label = QLabel('抓取点位')
    #     self.arm_pick_group_line_label = QLabel('----------------------------------------')
    #     self.arm_pick_pose_edit_layout = QGridLayout()
    #     self.arm_pick_layout.addWidget(self.arm_pick_group_label)
    #     self.arm_pick_layout.addWidget(self.arm_pick_group_line_label)
    #     self.arm_pick_layout.addLayout(self.arm_pick_pose_edit_layout)

    #     self.arm_pick_pose_x_label = QLabel('x')
    #     self.arm_pick_pose_x_line_edit = QLineEdit()
    #     self.arm_pick_pose_y_label = QLabel('y')
    #     self.arm_pick_pose_y_line_edit = QLineEdit()
    #     self.arm_pick_pose_z_label = QLabel('z')
    #     self.arm_pick_pose_z_line_edit = QLineEdit()
    #     self.arm_pick_pose_roll_label = QLabel('roll')
    #     self.arm_pick_pose_roll_line_edit = QLineEdit()
    #     self.arm_pick_pose_pitch_label = QLabel('pitch')
    #     self.arm_pick_pose_pitch_line_edit = QLineEdit()
    #     self.arm_pick_pose_yaw_label = QLabel('yaw')
    #     self.arm_pick_pose_yaw_line_edit = QLineEdit()

    #     self.arm_pick_pose_save_btn = QPushButton('重置')
    #     self.arm_pick_pose_send_btn = QPushButton('发送')
    #     self.arm_pick_pose_set_to_cur_pose_btn = QPushButton('当前位置')
    #     self.arm_pick_pose_set_to_aruco_pose_btn = QPushButton('读二维码')

    #     self.arm_pick_pose_edit_layout.addWidget(self.arm_pick_pose_x_label, 0, 1)
    #     self.arm_pick_pose_edit_layout.addWidget(self.arm_pick_pose_x_line_edit, 0, 2)
    #     self.arm_pick_pose_edit_layout.addWidget(self.arm_pick_pose_y_label, 1, 1)
    #     self.arm_pick_pose_edit_layout.addWidget(self.arm_pick_pose_y_line_edit, 1, 2)
    #     self.arm_pick_pose_edit_layout.addWidget(self.arm_pick_pose_z_label, 2, 1)
    #     self.arm_pick_pose_edit_layout.addWidget(self.arm_pick_pose_z_line_edit, 2, 2)
    #     self.arm_pick_pose_edit_layout.addWidget(self.arm_pick_pose_roll_label, 3, 1)
    #     self.arm_pick_pose_edit_layout.addWidget(self.arm_pick_pose_roll_line_edit, 3, 2)
    #     self.arm_pick_pose_edit_layout.addWidget(self.arm_pick_pose_pitch_label, 4, 1)
    #     self.arm_pick_pose_edit_layout.addWidget(self.arm_pick_pose_pitch_line_edit, 4, 2)
    #     self.arm_pick_pose_edit_layout.addWidget(self.arm_pick_pose_yaw_label, 5, 1)
    #     self.arm_pick_pose_edit_layout.addWidget(self.arm_pick_pose_yaw_line_edit, 5, 2)
    #     self.arm_pick_pose_edit_layout.addWidget(self.arm_pick_pose_save_btn, 6, 3)
    #     self.arm_pick_pose_edit_layout.addWidget(self.arm_pick_pose_send_btn, 6, 4)
    #     self.arm_pick_pose_edit_layout.addWidget(self.arm_pick_pose_set_to_cur_pose_btn, 7, 3)
    #     self.arm_pick_pose_edit_layout.addWidget(self.arm_pick_pose_set_to_aruco_pose_btn, 7, 4)

    #     # -> arm place pose group
    #     self.arm_place_layout = QVBoxLayout()
    #     self.body_layout.addLayout(self.arm_place_layout, 0, 2)
    #     self.arm_place_group_label = QLabel('放置点位')
    #     self.arm_place_group_line_label = QLabel('----------------------------------------')
    #     self.arm_place_pose_edit_layout = QGridLayout()
    #     self.arm_place_layout.addWidget(self.arm_place_group_label)
    #     self.arm_place_layout.addWidget(self.arm_place_group_line_label)
    #     self.arm_place_layout.addLayout(self.arm_place_pose_edit_layout)

    #     self.arm_place_pose_x_label = QLabel('x')
    #     self.arm_place_pose_x_line_edit = QLineEdit()
    #     self.arm_place_pose_y_label = QLabel('y')
    #     self.arm_place_pose_y_line_edit = QLineEdit()
    #     self.arm_place_pose_z_label = QLabel('z')
    #     self.arm_place_pose_z_line_edit = QLineEdit()
    #     self.arm_place_pose_roll_label = QLabel('roll')
    #     self.arm_place_pose_roll_line_edit = QLineEdit()
    #     self.arm_place_pose_pitch_label = QLabel('pitch')
    #     self.arm_place_pose_pitch_line_edit = QLineEdit()
    #     self.arm_place_pose_yaw_label = QLabel('yaw')
    #     self.arm_place_pose_yaw_line_edit = QLineEdit()

    #     self.arm_place_pose_save_btn = QPushButton('重置')
    #     self.arm_place_pose_send_btn = QPushButton('发送')
    #     self.arm_place_pose_set_to_cur_pose_btn = QPushButton('当前位置')
    #     self.arm_place_pose_set_to_aruco_pose_btn = QPushButton('读二维码')

    #     self.arm_place_pose_edit_layout.addWidget(self.arm_place_pose_x_label, 0, 1)
    #     self.arm_place_pose_edit_layout.addWidget(self.arm_place_pose_x_line_edit, 0, 2)
    #     self.arm_place_pose_edit_layout.addWidget(self.arm_place_pose_y_label, 1, 1)
    #     self.arm_place_pose_edit_layout.addWidget(self.arm_place_pose_y_line_edit, 1, 2)
    #     self.arm_place_pose_edit_layout.addWidget(self.arm_place_pose_z_label, 2, 1)
    #     self.arm_place_pose_edit_layout.addWidget(self.arm_place_pose_z_line_edit, 2, 2)
    #     self.arm_place_pose_edit_layout.addWidget(self.arm_place_pose_roll_label, 3, 1)
    #     self.arm_place_pose_edit_layout.addWidget(self.arm_place_pose_roll_line_edit, 3, 2)
    #     self.arm_place_pose_edit_layout.addWidget(self.arm_place_pose_pitch_label, 4, 1)
    #     self.arm_place_pose_edit_layout.addWidget(self.arm_place_pose_pitch_line_edit, 4, 2)
    #     self.arm_place_pose_edit_layout.addWidget(self.arm_place_pose_yaw_label, 5, 1)
    #     self.arm_place_pose_edit_layout.addWidget(self.arm_place_pose_yaw_line_edit, 5, 2)
    #     self.arm_place_pose_edit_layout.addWidget(self.arm_place_pose_save_btn, 6, 3)
    #     self.arm_place_pose_edit_layout.addWidget(self.arm_place_pose_send_btn, 6, 4)
    #     self.arm_place_pose_edit_layout.addWidget(self.arm_place_pose_set_to_cur_pose_btn, 7, 3)
    #     self.arm_place_pose_edit_layout.addWidget(self.arm_place_pose_set_to_aruco_pose_btn, 7, 4)


    #     self.log_output_layout = QVBoxLayout()
    #     self.main_layout.addLayout(self.log_output_layout)


    #     self.log_group_label = QLabel('日志输出')
    #     self.log_line_lable = QLabel('--------------------')
    #     self.log_output_window = QTextBrowser()

    #     self.log_output_layout.addWidget(self.log_group_label)

    #     self.log_output_layout.addWidget(self.log_line_lable)
    #     self.log_output_layout.addWidget(self.log_output_window)

    #     self.show()
    #     # self.app.exec()

    # def on_btn_send_nav_goal_clicked(self):
    #     pass



if __name__ == '__main__':
    app = QApplication(sys.argv)
    tools_app = ToolsUI()
    tools_app.show()
    app.exec()
