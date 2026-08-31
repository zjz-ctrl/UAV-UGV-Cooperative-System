#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import math
import time
import traceback

import rospy
from PyQt5.QtCore import Qt, QTimer, pyqtSignal, QObject
from PyQt5.QtGui import QImage, QPixmap
from PyQt5.QtWidgets import (
    QApplication, QWidget, QLabel, QPushButton, QVBoxLayout,
    QHBoxLayout, QGridLayout, QGroupBox, QTextEdit, QDoubleSpinBox
)

from sensor_msgs.msg import BatteryState, Image
from nav_msgs.msg import Odometry
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode
from cv_bridge import CvBridge
import cv2

from quadrotor_msgs.msg import PositionCommand


def quaternion_to_yaw(q):
    """
    四元数转 yaw，单位：rad
    """
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class RosSignals(QObject):
    state_signal = pyqtSignal(str, bool, bool)
    battery_signal = pyqtSignal(float, float)
    odom_signal = pyqtSignal(float, float, float, float)
    image_signal = pyqtSignal(QImage)
    log_signal = pyqtSignal(str)


class LightGCS(QWidget):
    def __init__(self):
        super().__init__()

        rospy.init_node("light_gcs", anonymous=True)

        self.signals = RosSignals()
        self.bridge = CvBridge()

        # -----------------------------
        # 状态变量
        # -----------------------------
        self.current_mode = "UNKNOWN"
        self.connected = False
        self.armed = False

        self.battery_voltage = 0.0
        self.battery_percent = 0.0

        self.current_x = 0.0
        self.current_y = 0.0
        self.current_z = 0.0
        self.current_yaw = 0.0

        self.has_odom = False

        # 起飞点 / Home 点
        self.home_set = False
        self.home_x = 0.0
        self.home_y = 0.0
        self.home_z = 0.0
        self.home_yaw = 0.0

        # 控制状态
        self.control_mode = "IDLE"
        self.cmd_start_time = None
        self.cmd_start_x = 0.0
        self.cmd_start_y = 0.0
        self.cmd_start_z = 0.0
        self.cmd_start_yaw = 0.0

        self.hold_x = 0.0
        self.hold_y = 0.0
        self.hold_z = 1.0
        self.hold_yaw = 0.0

        # 8 字轨迹参数
        self.fig8_start_time = None
        self.fig8_center_x = 0.0
        self.fig8_center_y = 0.0
        self.fig8_center_z = 1.0
        self.fig8_yaw = 0.0

        # -----------------------------
        # ROS 参数
        # -----------------------------
        self.image_topic = rospy.get_param(
            "~image_topic",
            "/camera/color/image_raw"
        )

        self.cmd_topic = rospy.get_param(
            "~cmd_topic",
            "/position_cmd"
        )

        self.publish_rate = rospy.get_param("~publish_rate", 30.0)

        # -----------------------------
        # ROS Publisher / Subscriber / Service
        # -----------------------------
        self.cmd_pub = rospy.Publisher(
            self.cmd_topic,
            PositionCommand,
            queue_size=10
        )

        rospy.Subscriber("/mavros/state", State, self.state_callback, queue_size=10)
        rospy.Subscriber("/mavros/battery", BatteryState, self.battery_callback, queue_size=10)
        rospy.Subscriber("/mavros/local_position/odom", Odometry, self.odom_callback, queue_size=10)
        rospy.Subscriber(self.image_topic, Image, self.image_callback, queue_size=1)

        self.arm_client = rospy.ServiceProxy("/mavros/cmd/arming", CommandBool)
        self.mode_client = rospy.ServiceProxy("/mavros/set_mode", SetMode)

        # -----------------------------
        # Qt 界面
        # -----------------------------
        self.init_ui()
        self.connect_signals()

        # -----------------------------
        # 定时器：发布控制指令
        # -----------------------------
        self.cmd_timer = QTimer()
        self.cmd_timer.timeout.connect(self.publish_control_command)
        self.cmd_timer.start(int(1000.0 / self.publish_rate))

        # 定时器：刷新界面
        self.ui_timer = QTimer()
        self.ui_timer.timeout.connect(self.refresh_ui)
        self.ui_timer.start(200)

        self.log("轻量地面站启动完成")
        self.log("图像话题: {}".format(self.image_topic))
        self.log("控制话题: {}".format(self.cmd_topic))

    # ============================================================
    # UI
    # ============================================================

    def init_ui(self):
        self.setWindowTitle("轻量化无人机地面站 - ROS / MAVROS / PyQt")
        self.resize(1200, 760)

        main_layout = QHBoxLayout()

        # 左侧：状态与控制
        left_layout = QVBoxLayout()

        # -----------------------------
        # 状态监控区域
        # -----------------------------
        status_group = QGroupBox("无人机状态监控")
        status_layout = QGridLayout()

        self.label_connected = QLabel("连接状态: 未知")
        self.label_mode = QLabel("飞行模式: UNKNOWN")
        self.label_armed = QLabel("解锁状态: UNKNOWN")
        self.label_battery = QLabel("电池: -- V / -- %")
        self.label_position = QLabel("位置: x=0.00, y=0.00, z=0.00")
        self.label_yaw = QLabel("Yaw: 0.00 deg")
        self.label_control_mode = QLabel("地面站控制状态: IDLE")

        status_layout.addWidget(self.label_connected, 0, 0)
        status_layout.addWidget(self.label_mode, 1, 0)
        status_layout.addWidget(self.label_armed, 2, 0)
        status_layout.addWidget(self.label_battery, 3, 0)
        status_layout.addWidget(self.label_position, 4, 0)
        status_layout.addWidget(self.label_yaw, 5, 0)
        status_layout.addWidget(self.label_control_mode, 6, 0)

        status_group.setLayout(status_layout)
        left_layout.addWidget(status_group)

        # -----------------------------
        # 参数设置区域
        # -----------------------------
        param_group = QGroupBox("飞行参数")
        param_layout = QGridLayout()

        self.spin_takeoff_height = QDoubleSpinBox()
        self.spin_takeoff_height.setRange(0.3, 5.0)
        self.spin_takeoff_height.setSingleStep(0.1)
        self.spin_takeoff_height.setValue(1.0)
        self.spin_takeoff_height.setSuffix(" m")

        self.spin_takeoff_time = QDoubleSpinBox()
        self.spin_takeoff_time.setRange(2.0, 20.0)
        self.spin_takeoff_time.setSingleStep(0.5)
        self.spin_takeoff_time.setValue(6.0)
        self.spin_takeoff_time.setSuffix(" s")

        self.spin_fig8_amp_x = QDoubleSpinBox()
        self.spin_fig8_amp_x.setRange(0.1, 5.0)
        self.spin_fig8_amp_x.setSingleStep(0.1)
        self.spin_fig8_amp_x.setValue(0.8)
        self.spin_fig8_amp_x.setSuffix(" m")

        self.spin_fig8_amp_y = QDoubleSpinBox()
        self.spin_fig8_amp_y.setRange(0.1, 5.0)
        self.spin_fig8_amp_y.setSingleStep(0.1)
        self.spin_fig8_amp_y.setValue(0.8)
        self.spin_fig8_amp_y.setSuffix(" m")

        self.spin_fig8_period = QDoubleSpinBox()
        self.spin_fig8_period.setRange(5.0, 60.0)
        self.spin_fig8_period.setSingleStep(1.0)
        self.spin_fig8_period.setValue(16.0)
        self.spin_fig8_period.setSuffix(" s")

        param_layout.addWidget(QLabel("起飞高度"), 0, 0)
        param_layout.addWidget(self.spin_takeoff_height, 0, 1)
        param_layout.addWidget(QLabel("起飞时间"), 1, 0)
        param_layout.addWidget(self.spin_takeoff_time, 1, 1)
        param_layout.addWidget(QLabel("8字 X 幅度"), 2, 0)
        param_layout.addWidget(self.spin_fig8_amp_x, 2, 1)
        param_layout.addWidget(QLabel("8字 Y 幅度"), 3, 0)
        param_layout.addWidget(self.spin_fig8_amp_y, 3, 1)
        param_layout.addWidget(QLabel("8字周期"), 4, 0)
        param_layout.addWidget(self.spin_fig8_period, 4, 1)

        param_group.setLayout(param_layout)
        left_layout.addWidget(param_group)

        # -----------------------------
        # 控制按钮区域
        # -----------------------------
        control_group = QGroupBox("飞行控制")
        control_layout = QGridLayout()

        self.btn_set_offboard = QPushButton("切换 OFFBOARD")
        self.btn_arm = QPushButton("解锁")
        self.btn_disarm = QPushButton("上锁")
        self.btn_takeoff = QPushButton("一键起飞")
        self.btn_hold = QPushButton("当前位置悬停")
        self.btn_land = QPushButton("一键降落")
        self.btn_fig8 = QPushButton("执行 8 字轨迹")
        self.btn_stop = QPushButton("紧急停止/悬停")

        self.btn_set_offboard.clicked.connect(self.set_offboard)
        self.btn_arm.clicked.connect(self.arm)
        self.btn_disarm.clicked.connect(self.disarm)
        self.btn_takeoff.clicked.connect(self.start_takeoff)
        self.btn_hold.clicked.connect(self.start_hold)
        self.btn_land.clicked.connect(self.start_land)
        self.btn_fig8.clicked.connect(self.start_figure8)
        self.btn_stop.clicked.connect(self.emergency_stop)

        control_layout.addWidget(self.btn_set_offboard, 0, 0)
        control_layout.addWidget(self.btn_arm, 0, 1)
        control_layout.addWidget(self.btn_disarm, 1, 0)
        control_layout.addWidget(self.btn_takeoff, 1, 1)
        control_layout.addWidget(self.btn_hold, 2, 0)
        control_layout.addWidget(self.btn_land, 2, 1)
        control_layout.addWidget(self.btn_fig8, 3, 0)
        control_layout.addWidget(self.btn_stop, 3, 1)

        control_group.setLayout(control_layout)
        left_layout.addWidget(control_group)

        # -----------------------------
        # 日志区域
        # -----------------------------
        log_group = QGroupBox("运行日志")
        log_layout = QVBoxLayout()
        self.text_log = QTextEdit()
        self.text_log.setReadOnly(True)
        log_layout.addWidget(self.text_log)
        log_group.setLayout(log_layout)
        left_layout.addWidget(log_group)

        # 右侧：视频显示
        right_layout = QVBoxLayout()
        video_group = QGroupBox("D435i 图像显示")
        video_layout = QVBoxLayout()

        self.image_label = QLabel("等待图像...")
        self.image_label.setAlignment(Qt.AlignCenter)
        self.image_label.setMinimumSize(640, 480)
        self.image_label.setStyleSheet("background-color: black; color: white;")

        video_layout.addWidget(self.image_label)
        video_group.setLayout(video_layout)
        right_layout.addWidget(video_group)

        main_layout.addLayout(left_layout, 4)
        main_layout.addLayout(right_layout, 6)

        self.setLayout(main_layout)

    def connect_signals(self):
        self.signals.state_signal.connect(self.update_state_ui)
        self.signals.battery_signal.connect(self.update_battery_ui)
        self.signals.odom_signal.connect(self.update_odom_ui)
        self.signals.image_signal.connect(self.update_image_ui)
        self.signals.log_signal.connect(self.append_log)

    # ============================================================
    # ROS Callbacks
    # ============================================================

    def state_callback(self, msg):
        self.current_mode = msg.mode
        self.connected = msg.connected
        self.armed = msg.armed
        self.signals.state_signal.emit(msg.mode, msg.connected, msg.armed)

    def battery_callback(self, msg):
        self.battery_voltage = msg.voltage

        if msg.percentage >= 0:
            self.battery_percent = msg.percentage * 100.0
        else:
            self.battery_percent = 0.0

        self.signals.battery_signal.emit(self.battery_voltage, self.battery_percent)

    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        self.current_z = msg.pose.pose.position.z
        self.current_yaw = quaternion_to_yaw(msg.pose.pose.orientation)

        self.has_odom = True

        if not self.home_set:
            self.home_x = self.current_x
            self.home_y = self.current_y
            self.home_z = self.current_z
            self.home_yaw = self.current_yaw
            self.home_set = True
            self.signals.log_signal.emit(
                "记录 Home 点: x={:.2f}, y={:.2f}, z={:.2f}".format(
                    self.home_x, self.home_y, self.home_z
                )
            )

        self.signals.odom_signal.emit(
            self.current_x,
            self.current_y,
            self.current_z,
            self.current_yaw
        )

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            cv_image = cv2.resize(cv_image, (640, 480))
            rgb_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)

            h, w, ch = rgb_image.shape
            bytes_per_line = ch * w

            qt_image = QImage(
                rgb_image.data,
                w,
                h,
                bytes_per_line,
                QImage.Format_RGB888
            ).copy()

            self.signals.image_signal.emit(qt_image)

        except Exception as e:
            self.signals.log_signal.emit("图像显示错误: {}".format(str(e)))

    # ============================================================
    # UI Updates
    # ============================================================

    def update_state_ui(self, mode, connected, armed):
        if connected:
            self.label_connected.setText("连接状态: 已连接")
            self.label_connected.setStyleSheet("color: green;")
        else:
            self.label_connected.setText("连接状态: 未连接")
            self.label_connected.setStyleSheet("color: red;")

        self.label_mode.setText("飞行模式: {}".format(mode))

        if armed:
            self.label_armed.setText("解锁状态: 已解锁")
            self.label_armed.setStyleSheet("color: green;")
        else:
            self.label_armed.setText("解锁状态: 未解锁")
            self.label_armed.setStyleSheet("color: red;")

    def update_battery_ui(self, voltage, percent):
        self.label_battery.setText(
            "电池: {:.2f} V / {:.1f} %".format(voltage, percent)
        )

        if percent > 30.0:
            self.label_battery.setStyleSheet("color: green;")
        elif percent > 15.0:
            self.label_battery.setStyleSheet("color: orange;")
        else:
            self.label_battery.setStyleSheet("color: red;")

    def update_odom_ui(self, x, y, z, yaw):
        self.label_position.setText(
            "位置: x={:.2f}, y={:.2f}, z={:.2f}".format(x, y, z)
        )
        self.label_yaw.setText(
            "Yaw: {:.2f} deg".format(math.degrees(yaw))
        )

    def update_image_ui(self, qt_image):
        self.image_label.setPixmap(QPixmap.fromImage(qt_image))

    def refresh_ui(self):
        self.label_control_mode.setText(
            "地面站控制状态: {}".format(self.control_mode)
        )

    def append_log(self, text):
        now = time.strftime("%H:%M:%S")
        self.text_log.append("[{}] {}".format(now, text))

    def log(self, text):
        self.signals.log_signal.emit(text)

    # ============================================================
    # MAVROS Services
    # ============================================================

    def set_offboard(self):
        try:
            rospy.wait_for_service("/mavros/set_mode", timeout=2.0)
            resp = self.mode_client(custom_mode="OFFBOARD")
            if resp.mode_sent:
                self.log("已请求切换到 OFFBOARD")
            else:
                self.log("OFFBOARD 模式切换请求失败")
        except Exception as e:
            self.log("切换 OFFBOARD 失败: {}".format(str(e)))

    def arm(self):
        try:
            rospy.wait_for_service("/mavros/cmd/arming", timeout=2.0)
            resp = self.arm_client(True)
            if resp.success:
                self.log("解锁成功")
            else:
                self.log("解锁请求失败")
        except Exception as e:
            self.log("解锁失败: {}".format(str(e)))

    def disarm(self):
        try:
            rospy.wait_for_service("/mavros/cmd/arming", timeout=2.0)
            resp = self.arm_client(False)
            if resp.success:
                self.log("上锁成功")
            else:
                self.log("上锁请求失败")
        except Exception as e:
            self.log("上锁失败: {}".format(str(e)))

    def set_land_mode(self):
        try:
            rospy.wait_for_service("/mavros/set_mode", timeout=2.0)
            resp = self.mode_client(custom_mode="AUTO.LAND")
            if resp.mode_sent:
                self.log("已请求 AUTO.LAND")
            else:
                self.log("AUTO.LAND 请求失败，改用 Offboard 缓慢下降")
        except Exception as e:
            self.log("AUTO.LAND 服务调用失败，改用 Offboard 缓慢下降: {}".format(str(e)))

    # ============================================================
    # 控制按钮逻辑
    # ============================================================

    def start_takeoff(self):
        if not self.has_odom:
            self.log("没有收到 odom，不能起飞")
            return

        target_z = self.spin_takeoff_height.value()

        self.cmd_start_time = rospy.Time.now().to_sec()
        self.cmd_start_x = self.current_x
        self.cmd_start_y = self.current_y
        self.cmd_start_z = self.current_z
        self.cmd_start_yaw = self.current_yaw

        self.hold_x = self.current_x
        self.hold_y = self.current_y
        self.hold_z = target_z
        self.hold_yaw = self.current_yaw

        self.control_mode = "TAKEOFF"

        self.log(
            "开始一键起飞: 当前 z={:.2f}, 目标 z={:.2f}".format(
                self.current_z, target_z
            )
        )

        # 先发布一小段 setpoint，再切 OFFBOARD 和解锁
        QTimer.singleShot(1000, self.set_offboard)
        QTimer.singleShot(1500, self.arm)

    def start_hold(self):
        if not self.has_odom:
            self.log("没有收到 odom，不能悬停")
            return

        self.hold_x = self.current_x
        self.hold_y = self.current_y
        self.hold_z = self.current_z
        self.hold_yaw = self.current_yaw

        self.control_mode = "HOLD"

        self.log(
            "当前位置悬停: x={:.2f}, y={:.2f}, z={:.2f}, yaw={:.2f}deg".format(
                self.hold_x,
                self.hold_y,
                self.hold_z,
                math.degrees(self.hold_yaw)
            )
        )

    def start_land(self):
        if not self.has_odom:
            self.log("没有收到 odom，不能降落")
            return

        self.cmd_start_time = rospy.Time.now().to_sec()
        self.cmd_start_x = self.current_x
        self.cmd_start_y = self.current_y
        self.cmd_start_z = self.current_z
        self.cmd_start_yaw = self.current_yaw

        self.control_mode = "LAND"

        self.log("开始一键降落")
        self.set_land_mode()

    def start_figure8(self):
        if not self.has_odom:
            self.log("没有收到 odom，不能执行 8 字轨迹")
            return

        self.fig8_center_x = self.current_x
        self.fig8_center_y = self.current_y
        self.fig8_center_z = self.current_z
        self.fig8_yaw = self.current_yaw
        self.fig8_start_time = rospy.Time.now().to_sec()

        self.control_mode = "FIGURE8"

        self.log(
            "开始执行 8 字轨迹，中心点: x={:.2f}, y={:.2f}, z={:.2f}".format(
                self.fig8_center_x,
                self.fig8_center_y,
                self.fig8_center_z
            )
        )

    def emergency_stop(self):
        if not self.has_odom:
            self.log("没有收到 odom，无法紧急悬停")
            return

        self.hold_x = self.current_x
        self.hold_y = self.current_y
        self.hold_z = self.current_z
        self.hold_yaw = self.current_yaw

        self.control_mode = "HOLD"

        self.log("紧急停止：切换为当前位置悬停")

    # ============================================================
    # PositionCommand 发布逻辑
    # ============================================================

    def publish_control_command(self):
        if rospy.is_shutdown():
            return

        if not self.has_odom:
            return

        now = rospy.Time.now().to_sec()

        if self.control_mode == "IDLE":
            return

        elif self.control_mode == "TAKEOFF":
            self.publish_takeoff_cmd(now)

        elif self.control_mode == "HOLD":
            self.publish_position_cmd(
                self.hold_x,
                self.hold_y,
                self.hold_z,
                self.hold_yaw
            )

        elif self.control_mode == "LAND":
            self.publish_land_cmd(now)

        elif self.control_mode == "FIGURE8":
            self.publish_figure8_cmd(now)

    def publish_takeoff_cmd(self, now):
        takeoff_time = self.spin_takeoff_time.value()
        target_z = self.spin_takeoff_height.value()

        t = now - self.cmd_start_time
        ratio = max(0.0, min(1.0, t / takeoff_time))

        # smoothstep 平滑起飞
        s = ratio * ratio * (3.0 - 2.0 * ratio)

        x = self.cmd_start_x
        y = self.cmd_start_y
        z = self.cmd_start_z + s * (target_z - self.cmd_start_z)
        yaw = self.cmd_start_yaw

        self.publish_position_cmd(x, y, z, yaw)

        if ratio >= 1.0:
            self.hold_x = x
            self.hold_y = y
            self.hold_z = target_z
            self.hold_yaw = yaw
            self.control_mode = "HOLD"
            self.log("起飞完成，进入悬停")

    def publish_land_cmd(self, now):
        """
        如果 AUTO.LAND 没有成功，这里仍然持续发布一个缓慢下降的 Offboard 指令。
        """
        descend_speed = 0.25
        t = now - self.cmd_start_time

        target_z = max(0.10, self.cmd_start_z - descend_speed * t)

        self.publish_position_cmd(
            self.cmd_start_x,
            self.cmd_start_y,
            target_z,
            self.cmd_start_yaw
        )

        if target_z <= 0.12:
            self.log("已下降到接近地面，建议确认安全后上锁")
            self.control_mode = "IDLE"

    def publish_figure8_cmd(self, now):
        if self.fig8_start_time is None:
            return

        t = now - self.fig8_start_time

        A = self.spin_fig8_amp_x.value()
        B = self.spin_fig8_amp_y.value()
        T = self.spin_fig8_period.value()

        w = 2.0 * math.pi / T

        # 标准横向 8 字轨迹
        x = self.fig8_center_x + A * math.sin(w * t)
        y = self.fig8_center_y + B * math.sin(w * t) * math.cos(w * t)
        z = self.fig8_center_z
        yaw = self.fig8_yaw

        self.publish_position_cmd(x, y, z, yaw)

    def publish_position_cmd(self, x, y, z, yaw):
        cmd = PositionCommand()
        cmd.header.stamp = rospy.Time.now()
        cmd.header.frame_id = "world"

        cmd.position.x = x
        cmd.position.y = y
        cmd.position.z = z

        cmd.velocity.x = 0.0
        cmd.velocity.y = 0.0
        cmd.velocity.z = 0.0

        cmd.acceleration.x = 0.0
        cmd.acceleration.y = 0.0
        cmd.acceleration.z = 0.0

        cmd.yaw = yaw
        cmd.yaw_dot = 0.0

        cmd.trajectory_flag = PositionCommand.TRAJECTORY_STATUS_READY
        cmd.trajectory_id = 1

        self.cmd_pub.publish(cmd)

    # ============================================================
    # 关闭窗口
    # ============================================================

    def closeEvent(self, event):
        self.control_mode = "IDLE"
        self.log("关闭地面站")
        event.accept()


def main():
    app = QApplication(sys.argv)

    try:
        win = LightGCS()
        win.show()
        sys.exit(app.exec_())

    except Exception:
        print(traceback.format_exc())


if __name__ == "__main__":
    main()
