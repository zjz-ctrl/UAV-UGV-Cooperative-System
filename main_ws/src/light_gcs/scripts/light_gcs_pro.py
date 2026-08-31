#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
SDU 轻量化无人机地面站

当前版本核心逻辑：
1. GAZEBO 模式：
   - 地面站可发布中位虚拟摇杆，辅助切换 OFFBOARD；
   - 一键起飞可自动先切 OFFBOARD，再启动起飞脚本；
   - 解锁 / 上锁 / 急停悬停等按钮可直接执行。

2. REAL 模式：
   - OFFBOARD、解锁、上锁、急停等安全关键操作不由地面站执行；
   - 点击相应按钮时仅在日志中提示“请使用遥控器”；
   - 一键起飞要求：
       * MAVROS 已连接；
       * 定位 / Odom 正常；
       * CXR 控制器正常；
       * raw setpoint 新鲜；
       * 当前已由遥控器切换至 OFFBOARD；
       * 当前已由遥控器完成解锁；
     满足后才允许启动 auto_takeoff_poscmd.py；
   - REAL 模式下起飞脚本强制 _auto_arm:=false。

3. 任务管理：
   - 8 字轨迹由地面站内部持续发布 /position_cmd；
   - “停止任务”可停止外部起飞脚本、停止 8 字轨迹及其他内部任务，并切换为当前位置 HOLD 悬停。
"""

import sys
import math
import time
import os
import signal
import subprocess
import traceback
import shlex
import threading

import rospy
import rosgraph
import cv2
import numpy as np

from PyQt5.QtCore import Qt, QTimer, pyqtSignal, QObject, QRectF, QSize
from PyQt5.QtGui import (
    QImage, QPixmap, QPainter, QColor, QPen, QFont, QPainterPath, QPolygonF
)
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QLabel, QPushButton, QFrame,
    QHBoxLayout, QVBoxLayout, QGridLayout, QTextEdit, QSizePolicy,
    QDoubleSpinBox, QComboBox, QLineEdit, QScrollArea, QToolButton, QStyle,
    QDialog, QStackedWidget
)

from sensor_msgs.msg import BatteryState, Image
from nav_msgs.msg import Odometry
from mavros_msgs.msg import State, RCIn, PositionTarget, ManualControl
from mavros_msgs.srv import CommandBool, SetMode
from cv_bridge import CvBridge
from quadrotor_msgs.msg import PositionCommand


def quaternion_to_euler(q):
    x, y, z, w = q.x, q.y, q.z, q.w

    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


class RosSignals(QObject):
    state_signal = pyqtSignal(str, bool, bool, bool)
    battery_signal = pyqtSignal(float, float)
    odom_signal = pyqtSignal(float, float, float, float, float, float)
    video1_signal = pyqtSignal(QImage)
    video2_signal = pyqtSignal(QImage)
    log_signal = pyqtSignal(str)
    remote_status_signal = pyqtSignal(bool, str)


class StatusCard(QFrame):
    def __init__(self, title, value="--"):
        super().__init__()
        self.setObjectName("StatusCard")

        self.title_label = QLabel(title)
        self.title_label.setObjectName("CardTitle")
        # 固定标题浅灰框高度，避免状态卡变高后标题背景被纵向拉伸
        self.title_label.setFixedHeight(28)
        self.title_label.setAlignment(Qt.AlignLeft | Qt.AlignVCenter)
        self.title_label.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)

        self.value_label = QLabel(value)
        self.value_label.setObjectName("CardValue")
        self.value_label.setWordWrap(True)

        layout = QVBoxLayout()
        layout.setContentsMargins(12, 10, 12, 10)
        layout.setSpacing(5)
        layout.addWidget(self.title_label)
        layout.addWidget(self.value_label)
        self.setLayout(layout)

    def set_value(self, text):
        self.value_label.setText(text)


class AttitudeCompassWidget(QWidget):
    """
    QGC 风格双仪表：
    - 左侧人工地平仪，保留黄色滚转指针、黄色机体基准线；
    - 右侧航向罗盘；
    - Roll / Pitch 数值放到姿态球外侧下方，不再压在圆内；
    - 两个圆表间距缩小，整体更紧凑。
    """
    def __init__(self):
        super().__init__()
        self.roll_deg = 0.0
        self.pitch_deg = 0.0
        self.yaw_deg = 0.0
        self.setMinimumSize(340, 118)
        self.setMaximumHeight(124)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)

    def set_attitude(self, roll_deg, pitch_deg, yaw_deg):
        self.roll_deg = roll_deg
        self.pitch_deg = pitch_deg
        self.yaw_deg = yaw_deg
        self.update()

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)

        w, h = self.width(), self.height()

        # 底部专门预留文字区，避免 R/P 进入姿态球内部
        footer_h = 18
        dial_h = max(70, h - footer_h)

        # 两个圆表更靠近一些
        dial_gap = 6
        r = max(40, min((w - dial_gap - 18) / 4.0, dial_h / 2.0 - 5))

        total_w = 4 * r + dial_gap
        start_x = max(6, (w - total_w) / 2.0)

        cx1 = start_x + r
        cx2 = cx1 + 2 * r + dial_gap
        cy = 5 + r

        self._draw_horizon(painter, cx1, cy, r)
        self._draw_compass(painter, cx2, cy, r)

        # R / P 单独放在姿态球下方
        painter.setPen(QColor(45, 55, 72))
        painter.setFont(QFont("Arial", 7, QFont.Bold))
        footer_rect = QRectF(cx1 - r, cy + r + 1, 2 * r, footer_h - 2)
        painter.drawText(
            footer_rect,
            Qt.AlignCenter,
            "R {:+.0f}°    P {:+.0f}°".format(self.roll_deg, self.pitch_deg)
        )

    def _draw_horizon(self, painter, cx, cy, r):
        # 外圈柔和底影
        painter.setPen(QPen(QColor(208, 218, 230), 3))
        painter.setBrush(QColor(245, 248, 252))
        painter.drawEllipse(QRectF(cx - r - 1, cy - r - 1, 2 * r + 2, 2 * r + 2))

        clip_path = QPainterPath()
        clip_path.addEllipse(QRectF(cx - r, cy - r, 2 * r, 2 * r))

        painter.save()
        painter.setClipPath(clip_path)
        painter.translate(cx, cy)
        painter.rotate(-self.roll_deg)

        pitch_scale = r / 35.0
        pitch_offset = max(-r * 0.8, min(r * 0.8, self.pitch_deg * pitch_scale))

        # 天空与地面
        painter.setPen(Qt.NoPen)
        painter.setBrush(QColor(70, 150, 230))
        painter.drawRect(QRectF(-2 * r, -3 * r + pitch_offset, 4 * r, 3 * r))
        painter.setBrush(QColor(108, 170, 86))
        painter.drawRect(QRectF(-2 * r, pitch_offset, 4 * r, 3 * r))

        # 地平线
        painter.setPen(QPen(QColor(248, 250, 252), 2.4))
        painter.drawLine(int(-2 * r), int(pitch_offset), int(2 * r), int(pitch_offset))

        # 俯仰刻度
        painter.setFont(QFont("Arial", 6, QFont.Bold))
        for deg in [-20, -10, 10, 20]:
            yy = pitch_offset - deg * pitch_scale
            if -r * 0.82 < yy < r * 0.82:
                length = 14 if abs(deg) == 10 else 18
                painter.setPen(QPen(QColor(255, 255, 255), 1.2))
                painter.drawLine(-length, int(yy), length, int(yy))
                painter.drawText(int(length + 3), int(yy + 2), str(abs(deg)))

        painter.restore()

        # 姿态球外圈
        painter.setPen(QPen(QColor(59, 69, 83), 2.2))
        painter.setBrush(Qt.NoBrush)
        painter.drawEllipse(QRectF(cx - r, cy - r, 2 * r, 2 * r))

        # 外圈滚转刻度
        painter.save()
        painter.translate(cx, cy)
        for angle in [-60, -45, -30, -20, -10, 0, 10, 20, 30, 45, 60]:
            painter.save()
            painter.rotate(angle)
            outer = r - 2
            inner = r - (10 if abs(angle) in [0, 30, 60] else 6)
            painter.setPen(QPen(QColor(228, 235, 242), 1.4))
            painter.drawLine(0, int(-outer), 0, int(-inner))
            painter.restore()
        painter.restore()

        # 恢复原来更醒目的黄色顶部滚转指针
        painter.setPen(Qt.NoPen)
        painter.setBrush(QColor(255, 205, 0))
        pointer = QPainterPath()
        pointer.moveTo(cx, cy - r + 4)
        pointer.lineTo(cx - 6, cy - r + 16)
        pointer.lineTo(cx + 6, cy - r + 16)
        pointer.closeSubpath()
        painter.drawPath(pointer)

        # 黄色机体基准线
        painter.setPen(QPen(QColor(250, 204, 21), 2.9))
        painter.drawLine(int(cx - 24), int(cy), int(cx - 7), int(cy))
        painter.drawLine(int(cx + 7), int(cy), int(cx + 24), int(cy))
        painter.drawLine(int(cx), int(cy - 8), int(cx), int(cy + 8))

    def _draw_compass(self, painter, cx, cy, r):
        painter.setPen(QPen(QColor(208, 218, 230), 3))
        painter.setBrush(QColor(245, 248, 252))
        painter.drawEllipse(QRectF(cx - r - 1, cy - r - 1, 2 * r + 2, 2 * r + 2))

        painter.setPen(QPen(QColor(61, 68, 81), 2.2))
        painter.setBrush(QColor(37, 42, 50))
        painter.drawEllipse(QRectF(cx - r, cy - r, 2 * r, 2 * r))

        painter.save()
        painter.translate(cx, cy)
        painter.rotate(-self.yaw_deg)

        # 罗盘刻度
        for angle in range(0, 360, 15):
            painter.save()
            painter.rotate(angle)
            outer = r - 4
            inner = r - (10 if angle % 45 == 0 else 7)
            painter.setPen(QPen(QColor(210, 220, 232), 1.0 if angle % 45 else 1.4))
            painter.drawLine(0, int(-outer), 0, int(-inner))
            painter.restore()

        # N/E/S/W
        painter.setPen(QColor(242, 246, 250))
        painter.setFont(QFont("Arial", 7, QFont.Bold))
        for label, angle in [("N", 0), ("E", 90), ("S", 180), ("W", 270)]:
            rad = math.radians(angle - 90)
            tx = math.cos(rad) * (r - 18)
            ty = math.sin(rad) * (r - 18)
            painter.drawText(QRectF(tx - 8, ty - 7, 16, 14), Qt.AlignCenter, label)

        painter.restore()

        # 航向固定指针
        painter.setPen(Qt.NoPen)
        painter.setBrush(QColor(96, 165, 250))
        tri = QPainterPath()
        tri.moveTo(cx, cy - r + 4)
        tri.lineTo(cx - 5, cy - r + 14)
        tri.lineTo(cx + 5, cy - r + 14)
        tri.closeSubpath()
        painter.drawPath(tri)

        painter.setPen(QPen(QColor(148, 163, 184), 1))
        painter.drawLine(int(cx), int(cy - r + 14), int(cx), int(cy - r + 22))

        heading = int((self.yaw_deg % 360 + 360) % 360)
        box = QRectF(cx - 18, cy - 10, 36, 20)
        painter.setPen(QPen(QColor(122, 138, 158), 1.0))
        painter.setBrush(QColor(24, 29, 37))
        painter.drawRoundedRect(box, 4, 4)
        painter.setPen(QColor(241, 245, 249))
        painter.setFont(QFont("Arial", 7, QFont.Bold))
        painter.drawText(box, Qt.AlignCenter, f"{heading:03d}°")


class VideoFrame(QFrame):
    def __init__(self, title="视频窗口"):
        super().__init__()
        self.setObjectName("VideoFrame")
        self.last_qimg = None

        self.title_label = QLabel(title)
        self.title_label.setObjectName("VideoTitle")

        self.image_label = QLabel("等待图像...")
        self.image_label.setObjectName("ImageArea")
        self.image_label.setAlignment(Qt.AlignCenter)
        self.image_label.setMinimumSize(260, 180)
        self.image_label.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

        layout = QVBoxLayout()
        layout.setContentsMargins(10, 10, 10, 10)
        layout.setSpacing(8)
        # 不显示“主视频流 / 副视频流”标题行，直接展示图像区域
        layout.addWidget(self.image_label)
        self.setLayout(layout)

    def set_title(self, text):
        self.title_label.setText(text)

    def set_qimage(self, qimg):
        self.last_qimg = qimg

        if qimg is None:
            return

        pix = QPixmap.fromImage(qimg)
        scaled = pix.scaled(
            self.image_label.width(),
            self.image_label.height(),
            Qt.KeepAspectRatio,
            Qt.SmoothTransformation
        )
        self.image_label.setPixmap(scaled)

    def resizeEvent(self, event):
        super().resizeEvent(event)

        if self.last_qimg is not None:
            self.set_qimage(self.last_qimg)


class VideoWorkspace(QFrame):
    """主视频 + 右下角画中画副视频。"""
    def __init__(self):
        super().__init__()
        self.setObjectName("VideoWorkspace")
        self.setMinimumHeight(350)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

        self.main_video = VideoFrame("主视频流")
        self.main_video.setObjectName("MainVideoFrame")
        self.main_video.setParent(self)

        self.sub_video = VideoFrame("副视频流")
        self.sub_video.setObjectName("PipVideoFrame")
        self.sub_video.setParent(self)
        self.sub_video.setFixedSize(250, 176)
        # 副视频不再显示背后的白色容器框，直接呈现图像区域
        self.sub_video.layout().setContentsMargins(0, 0, 0, 0)
        self.sub_video.layout().setSpacing(0)
        self.sub_video.raise_()

    def resizeEvent(self, event):
        super().resizeEvent(event)
        margin = 0
        self.main_video.setGeometry(
            margin,
            margin,
            max(0, self.width() - 2 * margin),
            max(0, self.height() - 2 * margin)
        )

        pip_margin = 14
        pip_w = min(280, max(220, int(self.width() * 0.30)))
        pip_h = min(195, max(154, int(pip_w * 0.69)))
        self.sub_video.setFixedSize(pip_w, pip_h)
        x = max(pip_margin, self.width() - pip_w - pip_margin)
        y = max(pip_margin, self.height() - pip_h - pip_margin)
        self.sub_video.move(x, y)
        self.sub_video.raise_()


class LightGCSPro(QMainWindow):
    def __init__(self):
        super().__init__()

        rospy.init_node("light_gcs_pro", anonymous=True)

        self.signals = RosSignals()
        self.bridge = CvBridge()
        self.is_closing = False
        self.cleanup_done = False
        self.shutdown_requested = False
        # 右侧“控制器 / 脚本”区域的进程管理
        self.cxr_process = None
        self.cxr_command = ["roslaunch", "cxr_egoctrl_v1", "cxr_egoctrl_v1.launch"]
        self.custom_task_process = None
        self.custom_task_command = None
        self.remote_takeoff_requested = False

        # =========================
        # 参数
        # =========================
        self.run_mode = rospy.get_param("~run_mode", "GAZEBO")
        if self.run_mode not in ["GAZEBO", "REAL"]:
            self.run_mode = "GAZEBO"

        self.logo_path = rospy.get_param("~logo_path", "")

        self.video_topic_1 = rospy.get_param(
            "~video_topic_1",
            "/iris_D435i/realsense/depth_camera/color/image_raw"
        )

        self.video_topic_2 = rospy.get_param(
            "~video_topic_2",
            "/iris_D435i/realsense/depth_camera/depth/image_raw"
        )

        self.cmd_topic = rospy.get_param("~cmd_topic", "/position_cmd")
        self.publish_rate = rospy.get_param("~publish_rate", 30.0)

        self.takeoff_height = rospy.get_param("~takeoff_height", 1.0)
        self.takeoff_time = rospy.get_param("~takeoff_time", 6.0)
        self.takeoff_pkg = rospy.get_param("~takeoff_pkg", "cxr_egoctrl_v1")
        self.takeoff_script = rospy.get_param("~takeoff_script", "auto_takeoff_poscmd.py")
        self.takeoff_auto_arm = rospy.get_param("~takeoff_auto_arm", True)

        self.takeoff_target_yaw = rospy.get_param("~takeoff_target_yaw", 0.0)
        self.takeoff_final_hold_time = rospy.get_param("~takeoff_final_hold_time", 3.0)

        self.fig8_amp_x = rospy.get_param("~fig8_amp_x", 0.6)
        self.fig8_amp_y = rospy.get_param("~fig8_amp_y", 0.6)
        self.fig8_period = rospy.get_param("~fig8_period", 18.0)

        self.manual_keepalive_rate = rospy.get_param("~manual_keepalive_rate", 20.0)

        # =========================
        # REAL 模式：机载电脑 SSH / tmux 远程管理配置
        # 说明：
        # 1. REAL 模式建议先在机载电脑独立启动 roscore；
        # 2. 笔记本启动地面站前，ROS_MASTER_URI 应指向机载电脑；
        # 3. 以下命令支持通过 launch 参数覆盖，避免把个人路径写死。
        # =========================
        self.remote_host_param = rospy.get_param("~remote_host", "192.168.1.110")
        self.remote_user_param = rospy.get_param("~remote_user", "omni")
        self.remote_ssh_port_param = int(rospy.get_param("~remote_ssh_port", 22))
        self.remote_session_prefix = rospy.get_param("~remote_session_prefix", "sdu_gcs")
        self.remote_setup_command = rospy.get_param(
            "~remote_setup_command",
            "source /opt/ros/noetic/setup.bash && source ~/Fast-Drone-250/devel/setup.bash"
        )

        self.remote_module_commands = {
            "mavros": rospy.get_param("~remote_mavros_cmd", "roslaunch mavros px4.launch"),
            "camera": rospy.get_param("~remote_camera_cmd", "roslaunch realsense2_camera rs_camera.launch"),
            "vins": rospy.get_param("~remote_vins_cmd", "roslaunch vins_estimator vins_rviz.launch"),
            "cxr": rospy.get_param("~remote_cxr_cmd", "roslaunch cxr_egoctrl_v1 cxr_egoctrl_v1.launch"),
        }

        # =========================
        # 飞控状态
        # =========================
        self.connected = False
        self.armed = False
        self.manual_input = False
        self.current_mode = "UNKNOWN"

        self.battery_voltage = 0.0
        self.battery_percent = 0.0

        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.has_odom = False

        # =========================
        # cxr raw setpoint 状态
        # =========================
        self.raw_setpoint_last_wall_time = None
        self.raw_setpoint_received = False

        # =========================
        # 虚拟摇杆状态
        # =========================
        self.manual_keepalive_enabled = False

        # =========================
        # OFFBOARD 切换状态机
        # =========================
        self.offboard_switch_active = False
        self.offboard_auto_takeoff_after = False
        self.offboard_switch_start_wall = None
        self.offboard_last_request_wall = 0.0
        self.offboard_request_count = 0
        self.offboard_max_request_count = 12
        self.offboard_warmup_sec = 1.0
        self.offboard_request_interval_sec = 0.6

        # =========================
        # 任务状态
        # =========================
        self.control_mode = "IDLE"
        self.takeoff_process = None

        self.hold_x = 0.0
        self.hold_y = 0.0
        self.hold_z = 1.0
        self.hold_yaw = 0.0

        self.land_start_time = None
        self.land_start_z = 0.0

        self.fig8_start_time = None
        self.fig8_center_x = 0.0
        self.fig8_center_y = 0.0
        self.fig8_center_z = 1.0
        self.fig8_yaw = 0.0

        self.home_set = False
        self.home_x = 0.0
        self.home_y = 0.0
        self.home_z = 0.0

        self.video1_qimg = None
        self.video2_qimg = None
        self.main_video_index = 1
        self.pip_visible = True

        # =========================
        # ROS 通信
        # =========================
        self.cmd_pub = rospy.Publisher(
            self.cmd_topic,
            PositionCommand,
            queue_size=10
        )

        self.manual_control_pub = rospy.Publisher(
            "/mavros/manual_control/send",
            ManualControl,
            queue_size=10
        )

        self.state_sub = rospy.Subscriber(
            "/mavros/state",
            State,
            self.state_callback,
            queue_size=10
        )

        self.battery_sub = rospy.Subscriber(
            "/mavros/battery",
            BatteryState,
            self.battery_callback,
            queue_size=10
        )

        self.odom_sub = rospy.Subscriber(
            "/mavros/local_position/odom",
            Odometry,
            self.odom_callback,
            queue_size=10
        )

        self.raw_setpoint_sub = rospy.Subscriber(
            "/mavros/setpoint_raw/local",
            PositionTarget,
            self.raw_setpoint_callback,
            queue_size=10
        )

        self.rc_sub = rospy.Subscriber(
            "/mavros/rc/in",
            RCIn,
            self.rc_callback,
            queue_size=10
        )

        self.video1_sub = rospy.Subscriber(
            self.video_topic_1,
            Image,
            self.video_callback,
            callback_args=1,
            queue_size=1
        )

        self.video2_sub = rospy.Subscriber(
            self.video_topic_2,
            Image,
            self.video_callback,
            callback_args=2,
            queue_size=1
        )

        self.arm_client = rospy.ServiceProxy("/mavros/cmd/arming", CommandBool)
        self.mode_client = rospy.ServiceProxy("/mavros/set_mode", SetMode)

        # =========================
        # UI
        # =========================
        self.init_ui()
        self.connect_signals()
        self.apply_styles()

        self.ui_timer = QTimer()
        self.ui_timer.timeout.connect(self.refresh_ui)
        self.ui_timer.start(150)

        self.cmd_timer = QTimer()
        self.cmd_timer.timeout.connect(self.publish_control_command)
        self.cmd_timer.start(int(1000.0 / self.publish_rate))

        self.manual_timer = QTimer()
        self.manual_timer.timeout.connect(self.publish_virtual_manual_control)
        self.manual_timer.start(int(1000.0 / self.manual_keepalive_rate))

        self.offboard_timer = QTimer()
        self.offboard_timer.timeout.connect(self.process_offboard_switch)

        self.log("轻量化地面站启动完成")
        self.log("运行模式: {}".format(self.run_mode))
        self.log("视频流1: {}".format(self.video_topic_1))
        self.log("视频流2: {}".format(self.video_topic_2))
        self.log("任务控制话题: {}".format(self.cmd_topic))
        self.log("起飞目标 yaw: {:.3f} rad".format(self.takeoff_target_yaw))
        self.log("起飞后自动保持时间: {:.1f} s".format(self.takeoff_final_hold_time))
        self.log("GAZEBO模式：地面站可自动发布中位虚拟摇杆，并切换 OFFBOARD")
        self.log("REAL模式：OFFBOARD、解锁、上锁、急停等安全关键动作由遥控器完成")
        self.log("停止任务按钮：可停止起飞脚本、8字轨迹及其他内部任务，并转为当前位置悬停")

    # ============================================================
    # UI
    # ============================================================
    def init_ui(self):
        self.setWindowTitle("SDU 轻量化无人机地面站")
        self.resize(1480, 900)
        self.setMinimumSize(1240, 760)

        central = QWidget()
        self.setCentralWidget(central)

        root_layout = QVBoxLayout()
        root_layout.setContentsMargins(10, 10, 10, 10)
        root_layout.setSpacing(8)
        central.setLayout(root_layout)

        # ============================================================
        # 顶部栏：Logo / 运行模式 / 链路状态 / 快捷按钮
        # ============================================================
        top_bar = QFrame()
        top_bar.setObjectName("TopBar")
        top_layout = QHBoxLayout()
        top_layout.setContentsMargins(14, 7, 14, 7)
        top_layout.setSpacing(10)
        top_bar.setLayout(top_layout)

        self.logo_label = QLabel()
        self.logo_label.setFixedSize(180, 44)
        self.logo_label.setObjectName("LogoLabel")
        self.logo_label.setAlignment(Qt.AlignLeft | Qt.AlignVCenter)
        self.logo_label.setStyleSheet("background: transparent; border: none;")

        if self.logo_path:
            pix = QPixmap(self.logo_path)
            if not pix.isNull():
                self.logo_label.setPixmap(
                    pix.scaled(176, 42, Qt.KeepAspectRatio, Qt.SmoothTransformation)
                )
            else:
                self.logo_label.setText("SDU UAV GCS")
        else:
            self.logo_label.setText("SDU UAV GCS")

        self.mode_panel = QFrame()
        self.mode_panel.setObjectName("ModePanel")
        mode_panel_layout = QHBoxLayout()
        mode_panel_layout.setContentsMargins(8, 4, 8, 4)
        mode_panel_layout.setSpacing(8)
        self.mode_panel.setLayout(mode_panel_layout)

        self.mode_title_label = QLabel("运行模式")
        self.mode_title_label.setObjectName("ModeTitleLabel")
        self.mode_title_label.setFixedWidth(68)
        self.mode_title_label.setAlignment(Qt.AlignCenter)

        self.mode_combo = QComboBox()
        self.mode_combo.setObjectName("ModeCombo")
        self.mode_combo.addItems(["GAZEBO", "REAL"])
        self.mode_combo.setCurrentText(self.run_mode)
        self.mode_combo.currentTextChanged.connect(self.change_run_mode)
        self.mode_combo.setFixedWidth(110)

        mode_panel_layout.addWidget(self.mode_title_label)
        mode_panel_layout.addWidget(self.mode_combo)

        self.top_status_label = QLabel("MAVROS 未连接")
        self.top_status_label.setObjectName("TopStatusLabel")
        self.top_status_label.setMinimumWidth(210)
        self.top_status_label.setAlignment(Qt.AlignCenter)

        self.btn_switch_stream = QPushButton("切换视频")
        self.btn_switch_stream.setObjectName("TopActionButton")
        self.btn_switch_stream.clicked.connect(self.switch_video)

        self.btn_toggle_pip = QPushButton("副视频")
        self.btn_toggle_pip.setObjectName("TopActionButton")
        self.btn_toggle_pip.clicked.connect(self.toggle_pip)

        self.btn_toggle_log_window = QPushButton("运行日志")
        self.btn_toggle_log_window.setObjectName("TopActionButton")
        self.btn_toggle_log_window.clicked.connect(self.toggle_log_dialog)

        self.btn_clear_log = QPushButton("清空日志")
        self.btn_clear_log.setObjectName("TopActionButton")
        self.btn_clear_log.clicked.connect(lambda: self.log_box.clear())

        top_layout.addWidget(self.logo_label)
        top_layout.addWidget(self.mode_panel)
        top_layout.addWidget(self.top_status_label)
        top_layout.addStretch()
        top_layout.addWidget(self.btn_switch_stream)
        top_layout.addWidget(self.btn_toggle_pip)
        top_layout.addWidget(self.btn_toggle_log_window)
        top_layout.addWidget(self.btn_clear_log)
        root_layout.addWidget(top_bar)

        # ============================================================
        # 主体滚动区：状态栏 + 视频 / 控制器脚本面板
        # 底部操作栏放在滚动区外，确保始终可见。
        # ============================================================
        body_scroll = QScrollArea()
        body_scroll.setObjectName("BodyScrollArea")
        body_scroll.setWidgetResizable(True)
        body_scroll.setFrameShape(QFrame.NoFrame)
        body_scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        body_scroll.setVerticalScrollBarPolicy(Qt.ScrollBarAsNeeded)
        body_scroll.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

        body_widget = QWidget()
        body_widget.setObjectName("BodyScrollContent")
        body_layout = QVBoxLayout()
        body_layout.setContentsMargins(0, 0, 0, 0)
        body_layout.setSpacing(8)
        body_widget.setLayout(body_layout)
        body_scroll.setWidget(body_widget)
        root_layout.addWidget(body_scroll, 1)

        # ============================================================
        # 状态卡片区
        # ============================================================
        status_frame = QFrame()
        status_frame.setObjectName("StatusFrame")
        status_layout = QHBoxLayout()
        status_layout.setContentsMargins(8, 8, 8, 8)
        status_layout.setSpacing(8)
        status_frame.setLayout(status_layout)

        self.card_battery = StatusCard("电池", "-- V\n-- %")
        self.card_state = StatusCard("飞控", "连接: --\n模式: --\n解锁: --")
        self.card_position = StatusCard("位置", "x=0.00\ny=0.00\nz=0.00\nyaw=0.0°")
        self.card_ready = StatusCard("起飞条件", "等待检查")
        self.card_control = StatusCard("控制状态", "IDLE")

        self.attitude_card = QFrame()
        self.attitude_card.setObjectName("StatusCard")
        attitude_layout = QVBoxLayout()
        attitude_layout.setContentsMargins(10, 6, 10, 5)
        attitude_layout.setSpacing(4)
        self.attitude_title = QLabel("姿态 / 航向")
        self.attitude_title.setObjectName("CardTitle")
        self.attitude_title.setFixedHeight(28)
        self.attitude_title.setAlignment(Qt.AlignLeft | Qt.AlignVCenter)
        self.attitude_title.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        self.attitude_widget = AttitudeCompassWidget()
        attitude_layout.addWidget(self.attitude_title)
        attitude_layout.addWidget(self.attitude_widget, alignment=Qt.AlignCenter)
        self.attitude_card.setLayout(attitude_layout)

        # 统一状态栏内各浅灰卡片高度，与“姿态 / 航向”卡片保持一致
        status_card_height = 164
        for card in [
            self.card_battery,
            self.card_state,
            self.attitude_card,
            self.card_position,
            self.card_ready,
            self.card_control,
        ]:
            card.setFixedHeight(status_card_height)

        status_layout.addWidget(self.card_battery, 0)
        status_layout.addWidget(self.card_state, 1)
        status_layout.addWidget(self.attitude_card, 1)
        status_layout.addWidget(self.card_position, 1)
        status_layout.addWidget(self.card_ready, 1)
        status_layout.addWidget(self.card_control, 1)
        body_layout.addWidget(status_frame)

        # ============================================================
        # 中部工作区：左侧视频工作台 + 右侧控制器脚本面板
        # ============================================================
        middle_frame = QFrame()
        middle_frame.setObjectName("MiddleFrame")
        middle_layout = QHBoxLayout()
        middle_layout.setContentsMargins(0, 0, 0, 0)
        middle_layout.setSpacing(10)
        middle_frame.setLayout(middle_layout)

        video_panel = QFrame()
        video_panel.setObjectName("VideoPanel")
        video_panel_layout = QVBoxLayout()
        video_panel_layout.setContentsMargins(10, 10, 10, 10)
        video_panel_layout.setSpacing(4)
        video_panel.setLayout(video_panel_layout)

        self.video_workspace = VideoWorkspace()
        self.main_video = self.video_workspace.main_video
        self.sub_video = self.video_workspace.sub_video
        video_panel_layout.addWidget(self.video_workspace, 1)
        middle_layout.addWidget(video_panel, 6)

        right_panel = QFrame()
        right_panel.setObjectName("RightPanel")
        right_layout = QVBoxLayout()
        right_layout.setContentsMargins(10, 10, 10, 10)
        right_layout.setSpacing(8)
        right_panel.setLayout(right_layout)

        # ---------- CXR 控制器卡片 ----------
        cxr_card = QFrame()
        cxr_card.setObjectName("PanelCard")
        cxr_layout = QVBoxLayout()
        cxr_layout.setContentsMargins(10, 7, 10, 7)
        cxr_layout.setSpacing(5)
        cxr_card.setLayout(cxr_layout)

        cxr_title = QLabel("CXR 控制器")
        cxr_title.setObjectName("MiniSectionTitle")
        cxr_layout.addWidget(cxr_title)

        cxr_btn_row = QHBoxLayout()
        cxr_btn_row.setSpacing(8)
        self.btn_start_cxr_inline = QPushButton("启动 CXR")
        self.btn_stop_cxr_inline = QPushButton("停止 CXR")
        self.btn_start_cxr_inline.setObjectName("PanelPrimaryButton")
        self.btn_stop_cxr_inline.setObjectName("PanelSecondaryButton")
        self.btn_start_cxr_inline.clicked.connect(self.start_cxr_controller)
        self.btn_stop_cxr_inline.clicked.connect(self.stop_cxr_controller)
        cxr_btn_row.addWidget(self.btn_start_cxr_inline)
        cxr_btn_row.addWidget(self.btn_stop_cxr_inline)
        cxr_layout.addLayout(cxr_btn_row)
        right_layout.addWidget(cxr_card)

        # ---------- 脚本 / Launch 卡片 ----------
        task_card = QFrame()
        task_card.setObjectName("PanelCard")
        task_layout = QVBoxLayout()
        task_layout.setContentsMargins(10, 7, 10, 7)
        task_layout.setSpacing(5)
        task_card.setLayout(task_layout)

        task_title = QLabel("指定脚本")
        task_title.setObjectName("MiniSectionTitle")
        task_layout.addWidget(task_title)

        package_row = QHBoxLayout()
        package_row.setSpacing(8)
        package_label = QLabel("功能包")
        package_label.setObjectName("FormLabel")
        package_label.setFixedWidth(54)
        package_label.setAlignment(Qt.AlignCenter)
        self.task_package_input = QLineEdit()
        self.task_package_input.setText("cxr_egoctrl_v1")
        self.task_package_input.setPlaceholderText("默认 cxr_egoctrl_v1")
        package_row.addWidget(package_label)
        package_row.addWidget(self.task_package_input, 1)
        task_layout.addLayout(package_row)

        name_row = QHBoxLayout()
        name_row.setSpacing(8)
        name_label = QLabel("名称")
        name_label.setObjectName("FormLabel")
        name_label.setFixedWidth(54)
        name_label.setAlignment(Qt.AlignCenter)
        self.task_name_input = QLineEdit()
        self.task_name_input.setPlaceholderText("输入 .py / .launch，或完整命令")
        self.task_name_input.returnPressed.connect(self.start_custom_task)
        name_row.addWidget(name_label)
        name_row.addWidget(self.task_name_input, 1)
        task_layout.addLayout(name_row)

        self.custom_task_status_label = QLabel("任务状态：未启动")
        self.custom_task_status_label.setObjectName("InlineStatus")
        self.custom_task_status_label.setWordWrap(True)
        task_layout.addWidget(self.custom_task_status_label)

        task_btn_row = QHBoxLayout()
        task_btn_row.setSpacing(8)
        self.btn_run_custom_task = QPushButton("运行")
        self.btn_stop_custom_task = QPushButton("停止")
        self.btn_clear_custom_task = QPushButton("清空")
        self.btn_run_custom_task.setObjectName("PanelPrimaryButton")
        self.btn_stop_custom_task.setObjectName("PanelWarningButton")
        self.btn_clear_custom_task.setObjectName("PanelSecondaryButton")
        self.btn_run_custom_task.clicked.connect(self.start_custom_task)
        self.btn_stop_custom_task.clicked.connect(self.stop_custom_task)
        self.btn_clear_custom_task.clicked.connect(self.clear_custom_task_inputs)
        task_btn_row.addWidget(self.btn_run_custom_task)
        task_btn_row.addWidget(self.btn_stop_custom_task)
        task_btn_row.addWidget(self.btn_clear_custom_task)
        task_layout.addLayout(task_btn_row)
        right_layout.addWidget(task_card)

        # ---------- 目标点前往卡片 ----------
        goto_card = QFrame()
        goto_card.setObjectName("PanelCard")
        goto_layout = QVBoxLayout()
        goto_layout.setContentsMargins(10, 7, 10, 7)
        goto_layout.setSpacing(6)
        goto_card.setLayout(goto_layout)

        goto_title = QLabel("目标点控制")
        goto_title.setObjectName("MiniSectionTitle")
        goto_layout.addWidget(goto_title)

        goto_grid = QGridLayout()
        goto_grid.setHorizontalSpacing(8)
        goto_grid.setVerticalSpacing(6)

        self.target_x_spin = self.make_target_spin(-100.0, 100.0, 0.0)
        self.target_y_spin = self.make_target_spin(-100.0, 100.0, 0.0)
        self.target_z_spin = self.make_target_spin(0.0, 100.0, self.takeoff_height)
        self.target_yaw_spin = self.make_target_spin(-180.0, 180.0, 0.0)

        target_items = [
            ("X / m", self.target_x_spin, 0, 0),
            ("Y / m", self.target_y_spin, 0, 2),
            ("Z / m", self.target_z_spin, 1, 0),
            ("Yaw / °", self.target_yaw_spin, 1, 2),
        ]

        for label_text, spin, row, col in target_items:
            label = QLabel(label_text)
            label.setObjectName("FormLabel")
            label.setAlignment(Qt.AlignCenter)
            label.setFixedWidth(58)
            goto_grid.addWidget(label, row, col)
            goto_grid.addWidget(spin, row, col + 1)

        goto_layout.addLayout(goto_grid)

        self.btn_goto_target = QPushButton("前往目标点")
        self.btn_goto_target.setObjectName("PanelPrimaryButton")
        self.btn_goto_target.clicked.connect(self.goto_target_point)
        goto_layout.addWidget(self.btn_goto_target)

        goto_hint = QLabel("目标点采用本地坐标系；点击后持续向 /position_cmd 发布目标保持指令。")
        goto_hint.setObjectName("InlineStatus")
        goto_hint.setWordWrap(True)
        goto_layout.addWidget(goto_hint)

        right_layout.addWidget(goto_card, 1)

        # ---------- 独立运行日志窗口，默认隐藏 ----------
        self.log_dialog = QDialog(self)
        self.log_dialog.setObjectName("LogDialog")
        self.log_dialog.setWindowTitle("运行日志")
        self.log_dialog.resize(760, 420)

        log_dialog_layout = QVBoxLayout()
        log_dialog_layout.setContentsMargins(12, 12, 12, 12)
        log_dialog_layout.setSpacing(8)
        self.log_dialog.setLayout(log_dialog_layout)

        log_header = QLabel("地面站运行日志")
        log_header.setObjectName("SectionTitle")
        log_dialog_layout.addWidget(log_header)

        self.log_box = QTextEdit()
        self.log_box.setReadOnly(True)
        self.log_box.setMinimumSize(720, 330)
        log_dialog_layout.addWidget(self.log_box, 1)

        middle_layout.addWidget(right_panel, 5)

        # ============================================================
        # REAL 页面：机载电脑远程管理 + 实机任务控制
        # 与 GAZEBO 页面使用独立界面，避免本地仿真控制与实机远程管理互相干扰。
        # ============================================================
        real_middle_frame = QFrame()
        real_middle_frame.setObjectName("MiddleFrame")
        real_middle_layout = QHBoxLayout()
        real_middle_layout.setContentsMargins(0, 0, 0, 0)
        real_middle_layout.setSpacing(10)
        real_middle_frame.setLayout(real_middle_layout)

        real_video_panel = QFrame()
        real_video_panel.setObjectName("VideoPanel")
        real_video_panel_layout = QVBoxLayout()
        real_video_panel_layout.setContentsMargins(10, 10, 10, 10)
        real_video_panel_layout.setSpacing(4)
        real_video_panel.setLayout(real_video_panel_layout)

        self.real_video_workspace = VideoWorkspace()
        self.real_main_video = self.real_video_workspace.main_video
        self.real_sub_video = self.real_video_workspace.sub_video
        real_video_panel_layout.addWidget(self.real_video_workspace, 1)
        real_middle_layout.addWidget(real_video_panel, 6)

        real_right_panel = QFrame()
        real_right_panel.setObjectName("RightPanel")
        real_right_layout = QVBoxLayout()
        real_right_layout.setContentsMargins(10, 10, 10, 10)
        real_right_layout.setSpacing(8)
        real_right_panel.setLayout(real_right_layout)

        # ---------- 机载连接卡片 ----------
        remote_conn_card = QFrame()
        remote_conn_card.setObjectName("PanelCard")
        remote_conn_layout = QVBoxLayout()
        remote_conn_layout.setContentsMargins(10, 7, 10, 7)
        remote_conn_layout.setSpacing(6)
        remote_conn_card.setLayout(remote_conn_layout)

        remote_conn_title = QLabel("机载电脑连接")
        remote_conn_title.setObjectName("MiniSectionTitle")
        remote_conn_layout.addWidget(remote_conn_title)

        remote_conn_grid = QGridLayout()
        remote_conn_grid.setHorizontalSpacing(8)
        remote_conn_grid.setVerticalSpacing(6)

        host_label = QLabel("IP")
        host_label.setObjectName("FormLabel")
        host_label.setFixedWidth(54)
        host_label.setAlignment(Qt.AlignCenter)
        self.remote_host_input = QLineEdit(self.remote_host_param)
        self.remote_host_input.setPlaceholderText("机载电脑 IP")

        user_label = QLabel("用户")
        user_label.setObjectName("FormLabel")
        user_label.setFixedWidth(54)
        user_label.setAlignment(Qt.AlignCenter)
        self.remote_user_input = QLineEdit(self.remote_user_param)
        self.remote_user_input.setPlaceholderText("SSH 用户名")

        port_label = QLabel("端口")
        port_label.setObjectName("FormLabel")
        port_label.setFixedWidth(54)
        port_label.setAlignment(Qt.AlignCenter)
        self.remote_port_input = QLineEdit(str(self.remote_ssh_port_param))
        self.remote_port_input.setPlaceholderText("22")

        remote_conn_grid.addWidget(host_label, 0, 0)
        remote_conn_grid.addWidget(self.remote_host_input, 0, 1)
        remote_conn_grid.addWidget(user_label, 0, 2)
        remote_conn_grid.addWidget(self.remote_user_input, 0, 3)
        remote_conn_grid.addWidget(port_label, 1, 0)
        remote_conn_grid.addWidget(self.remote_port_input, 1, 1)
        remote_conn_layout.addLayout(remote_conn_grid)

        self.remote_status_label = QLabel(
            "SSH：未测试 | 笔记本 ROS_MASTER_URI={}".format(os.environ.get("ROS_MASTER_URI", "未设置"))
        )
        self.remote_status_label.setObjectName("InlineStatus")
        self.remote_status_label.setWordWrap(True)
        remote_conn_layout.addWidget(self.remote_status_label)

        remote_conn_btn_row = QHBoxLayout()
        remote_conn_btn_row.setSpacing(8)
        self.btn_test_remote_ssh = QPushButton("测试 SSH")
        self.btn_refresh_remote_sessions = QPushButton("查看远程会话")
        self.btn_test_remote_ssh.setObjectName("PanelPrimaryButton")
        self.btn_refresh_remote_sessions.setObjectName("PanelSecondaryButton")
        self.btn_test_remote_ssh.clicked.connect(self.test_remote_ssh)
        self.btn_refresh_remote_sessions.clicked.connect(self.refresh_remote_sessions)
        remote_conn_btn_row.addWidget(self.btn_test_remote_ssh)
        remote_conn_btn_row.addWidget(self.btn_refresh_remote_sessions)
        remote_conn_layout.addLayout(remote_conn_btn_row)
        real_right_layout.addWidget(remote_conn_card)

        # ---------- 远程模块管理卡片 ----------
        remote_modules_card = QFrame()
        remote_modules_card.setObjectName("PanelCard")
        remote_modules_layout = QVBoxLayout()
        remote_modules_layout.setContentsMargins(10, 7, 10, 7)
        remote_modules_layout.setSpacing(6)
        remote_modules_card.setLayout(remote_modules_layout)

        remote_modules_title = QLabel("机载模块管理")
        remote_modules_title.setObjectName("MiniSectionTitle")
        remote_modules_layout.addWidget(remote_modules_title)

        remote_modules_grid = QGridLayout()
        remote_modules_grid.setHorizontalSpacing(8)
        remote_modules_grid.setVerticalSpacing(6)

        remote_module_rows = [
            ("MAVROS", "mavros"),
            ("D435i 相机", "camera"),
            ("VINS", "vins"),
            ("CXR 控制器", "cxr"),
        ]

        for row, (module_name, module_key) in enumerate(remote_module_rows):
            label = QLabel(module_name)
            label.setObjectName("FormLabel")
            label.setAlignment(Qt.AlignCenter)
            start_btn = QPushButton("启动")
            stop_btn = QPushButton("停止")
            start_btn.setObjectName("PanelPrimaryButton")
            stop_btn.setObjectName("PanelSecondaryButton")
            start_btn.clicked.connect(
                lambda checked=False, key=module_key, name=module_name:
                self.start_remote_module(key, name)
            )
            stop_btn.clicked.connect(
                lambda checked=False, key=module_key, name=module_name:
                self.stop_remote_module(key, name)
            )
            remote_modules_grid.addWidget(label, row, 0)
            remote_modules_grid.addWidget(start_btn, row, 1)
            remote_modules_grid.addWidget(stop_btn, row, 2)

        remote_modules_layout.addLayout(remote_modules_grid)
        real_right_layout.addWidget(remote_modules_card)

        # ---------- REAL 目标点控制卡片 ----------
        real_goto_card = QFrame()
        real_goto_card.setObjectName("PanelCard")
        real_goto_layout = QVBoxLayout()
        real_goto_layout.setContentsMargins(10, 7, 10, 7)
        real_goto_layout.setSpacing(6)
        real_goto_card.setLayout(real_goto_layout)

        real_goto_title = QLabel("实机目标点控制")
        real_goto_title.setObjectName("MiniSectionTitle")
        real_goto_layout.addWidget(real_goto_title)

        real_goto_grid = QGridLayout()
        real_goto_grid.setHorizontalSpacing(8)
        real_goto_grid.setVerticalSpacing(6)

        self.real_target_x_spin = self.make_target_spin(-100.0, 100.0, 0.0)
        self.real_target_y_spin = self.make_target_spin(-100.0, 100.0, 0.0)
        self.real_target_z_spin = self.make_target_spin(0.0, 100.0, self.takeoff_height)
        self.real_target_yaw_spin = self.make_target_spin(-180.0, 180.0, 0.0)

        real_target_items = [
            ("X / m", self.real_target_x_spin, 0, 0),
            ("Y / m", self.real_target_y_spin, 0, 2),
            ("Z / m", self.real_target_z_spin, 1, 0),
            ("Yaw / °", self.real_target_yaw_spin, 1, 2),
        ]

        for label_text, spin, row, col in real_target_items:
            label = QLabel(label_text)
            label.setObjectName("FormLabel")
            label.setAlignment(Qt.AlignCenter)
            label.setFixedWidth(58)
            real_goto_grid.addWidget(label, row, col)
            real_goto_grid.addWidget(spin, row, col + 1)

        real_goto_layout.addLayout(real_goto_grid)

        self.btn_real_goto_target = QPushButton("前往目标点")
        self.btn_real_goto_target.setObjectName("PanelPrimaryButton")
        self.btn_real_goto_target.clicked.connect(self.goto_real_target_point)
        real_goto_layout.addWidget(self.btn_real_goto_target)

        real_goto_hint = QLabel("实机目标点由笔记本地面站发布到 /position_cmd；请确保 CXR、OFFBOARD 与解锁状态已就绪。")
        real_goto_hint.setObjectName("InlineStatus")
        real_goto_hint.setWordWrap(True)
        real_goto_layout.addWidget(real_goto_hint)
        real_right_layout.addWidget(real_goto_card)

        # ---------- REAL 自定义远程命令 ----------
        remote_custom_card = QFrame()
        remote_custom_card.setObjectName("PanelCard")
        remote_custom_layout = QVBoxLayout()
        remote_custom_layout.setContentsMargins(10, 7, 10, 7)
        remote_custom_layout.setSpacing(6)
        remote_custom_card.setLayout(remote_custom_layout)

        remote_custom_title = QLabel("远程自定义脚本")
        remote_custom_title.setObjectName("MiniSectionTitle")
        remote_custom_layout.addWidget(remote_custom_title)

        custom_session_row = QHBoxLayout()
        custom_session_row.setSpacing(8)
        custom_session_label = QLabel("会话")
        custom_session_label.setObjectName("FormLabel")
        custom_session_label.setFixedWidth(54)
        custom_session_label.setAlignment(Qt.AlignCenter)
        self.remote_custom_session_input = QLineEdit("custom")
        self.remote_custom_session_input.setPlaceholderText("tmux 会话后缀")
        custom_session_row.addWidget(custom_session_label)
        custom_session_row.addWidget(self.remote_custom_session_input, 1)
        remote_custom_layout.addLayout(custom_session_row)

        custom_cmd_row = QHBoxLayout()
        custom_cmd_row.setSpacing(8)
        custom_cmd_label = QLabel("命令")
        custom_cmd_label.setObjectName("FormLabel")
        custom_cmd_label.setFixedWidth(54)
        custom_cmd_label.setAlignment(Qt.AlignCenter)
        self.remote_custom_cmd_input = QLineEdit()
        self.remote_custom_cmd_input.setPlaceholderText("例如：roslaunch xxx xxx.launch")
        custom_cmd_row.addWidget(custom_cmd_label)
        custom_cmd_row.addWidget(self.remote_custom_cmd_input, 1)
        remote_custom_layout.addLayout(custom_cmd_row)

        remote_custom_btn_row = QHBoxLayout()
        remote_custom_btn_row.setSpacing(8)
        self.btn_start_remote_custom = QPushButton("启动远程脚本")
        self.btn_stop_remote_custom = QPushButton("停止远程脚本")
        self.btn_start_remote_custom.setObjectName("PanelPrimaryButton")
        self.btn_stop_remote_custom.setObjectName("PanelSecondaryButton")
        self.btn_start_remote_custom.clicked.connect(self.start_remote_custom_task)
        self.btn_stop_remote_custom.clicked.connect(self.stop_remote_custom_task)
        remote_custom_btn_row.addWidget(self.btn_start_remote_custom)
        remote_custom_btn_row.addWidget(self.btn_stop_remote_custom)
        remote_custom_layout.addLayout(remote_custom_btn_row)
        real_right_layout.addWidget(remote_custom_card)

        real_safety_hint = QLabel(
            "REAL 安全原则：OFFBOARD 切换、解锁/上锁、紧急接管请使用遥控器；机载 roscore 请先独立启动。"
        )
        real_safety_hint.setObjectName("InlineStatus")
        real_safety_hint.setWordWrap(True)
        real_right_layout.addWidget(real_safety_hint)
        real_right_layout.addStretch(1)

        real_middle_layout.addWidget(real_right_panel, 6)

        self.mode_body_stack = QStackedWidget()
        self.mode_body_stack.setObjectName("ModeBodyStack")
        self.mode_body_stack.addWidget(middle_frame)
        self.mode_body_stack.addWidget(real_middle_frame)
        body_layout.addWidget(self.mode_body_stack, 1)

        # ============================================================
        # 底部固定操作栏：分组 / 语义配色
        # ============================================================
        bottom_frame = QFrame()
        bottom_frame.setObjectName("BottomBar")
        bottom_frame.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        bottom_layout = QHBoxLayout()
        bottom_layout.setContentsMargins(8, 4, 8, 4)
        bottom_layout.setSpacing(5)
        bottom_frame.setLayout(bottom_layout)

        self.btn_offboard = self.make_action_btn("OFFBOARD", "mode", "切换 OFFBOARD / 板载模式", QStyle.SP_BrowserReload)
        self.btn_takeoff = self.make_action_btn("起飞", "primary", "一键起飞", QStyle.SP_ArrowUp)
        self.btn_stop_task = self.make_action_btn("停止", "warning", "停止当前任务并保持", QStyle.SP_MediaStop)
        self.btn_hold = self.make_action_btn("悬停", "normal", "当前位置悬停", QStyle.SP_MediaPause)
        self.btn_land = self.make_action_btn("降落", "normal", "降落", QStyle.SP_ArrowDown)
        self.btn_return = self.make_action_btn("返航", "normal", "返回起点或预设返航点", QStyle.SP_DialogResetButton)
        self.btn_traj = self.make_action_btn("轨迹", "accent", "执行 8 字轨迹", QStyle.SP_FileDialogDetailedView)
        self.btn_stop = self.make_action_btn("急停", "danger", "急停悬停；实机请优先遥控器接管", QStyle.SP_MessageBoxCritical)
        self.btn_arm = self.make_action_btn("解锁", "neutral", "解锁；实机请使用遥控器", QStyle.SP_DialogApplyButton)
        self.btn_disarm = self.make_action_btn("上锁", "neutral", "上锁；实机请使用遥控器", QStyle.SP_DialogCancelButton)

        self.btn_offboard.clicked.connect(lambda: self.start_offboard_switch(auto_takeoff_after=False))
        self.btn_takeoff.clicked.connect(self.start_takeoff)
        self.btn_stop_task.clicked.connect(self.stop_current_task)
        self.btn_hold.clicked.connect(self.start_hold)
        self.btn_land.clicked.connect(self.start_land)
        self.btn_return.clicked.connect(self.start_return_home)
        self.btn_traj.clicked.connect(self.start_figure8)
        self.btn_stop.clicked.connect(self.emergency_stop)
        self.btn_arm.clicked.connect(self.arm)
        self.btn_disarm.clicked.connect(self.disarm)

        bottom_layout.addStretch()
        for widget in [
            self.btn_offboard,
            self.make_vertical_separator(),
            self.btn_takeoff,
            self.btn_hold,
            self.btn_land,
            self.btn_return,
            self.make_vertical_separator(),
            self.btn_traj,
            self.btn_stop_task,
            self.make_vertical_separator(),
            self.btn_stop,
            self.btn_arm,
            self.btn_disarm,
        ]:
            bottom_layout.addWidget(widget)
        bottom_layout.addStretch()

        # REAL 模式专属底部操作栏：不提供 OFFBOARD / 解锁 / 上锁 / 急停软件按钮，
        # 避免和遥控器安全职责混淆。
        real_bottom_frame = QFrame()
        real_bottom_frame.setObjectName("BottomBar")
        real_bottom_frame.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        real_bottom_layout = QHBoxLayout()
        real_bottom_layout.setContentsMargins(8, 4, 8, 4)
        real_bottom_layout.setSpacing(5)
        real_bottom_frame.setLayout(real_bottom_layout)

        self.btn_real_takeoff = self.make_action_btn("起飞", "primary", "确认遥控器已切 OFFBOARD 且已解锁后，远程启动起飞脚本", QStyle.SP_ArrowUp)
        self.btn_real_stop_task = self.make_action_btn("停止", "warning", "停止当前任务并保持；同时请求停止远程起飞脚本", QStyle.SP_MediaStop)
        self.btn_real_hold = self.make_action_btn("悬停", "normal", "当前位置悬停", QStyle.SP_MediaPause)
        self.btn_real_land = self.make_action_btn("降落", "normal", "REAL 模式仅提示安全降落流程", QStyle.SP_ArrowDown)
        self.btn_real_return = self.make_action_btn("返航", "normal", "返回起点或预设返航点", QStyle.SP_DialogResetButton)
        self.btn_real_traj = self.make_action_btn("轨迹", "accent", "执行 8 字轨迹", QStyle.SP_FileDialogDetailedView)

        self.btn_real_takeoff.clicked.connect(self.start_takeoff)
        self.btn_real_stop_task.clicked.connect(self.stop_current_task)
        self.btn_real_hold.clicked.connect(self.start_hold)
        self.btn_real_land.clicked.connect(self.start_land)
        self.btn_real_return.clicked.connect(self.start_return_home)
        self.btn_real_traj.clicked.connect(self.start_figure8)

        real_bottom_layout.addStretch()
        for widget in [
            self.btn_real_takeoff,
            self.btn_real_hold,
            self.btn_real_land,
            self.btn_real_return,
            self.make_vertical_separator(),
            self.btn_real_traj,
            self.btn_real_stop_task,
        ]:
            real_bottom_layout.addWidget(widget)
        real_bottom_layout.addStretch()

        self.mode_bottom_stack = QStackedWidget()
        self.mode_bottom_stack.setObjectName("ModeBottomStack")
        self.mode_bottom_stack.addWidget(bottom_frame)
        self.mode_bottom_stack.addWidget(real_bottom_frame)
        root_layout.addWidget(self.mode_bottom_stack)

        self.update_mode_views()

    def make_action_btn(self, text, role="normal", tooltip="", standard_icon=None):
        btn = QToolButton()
        btn.setText(text)
        btn.setToolButtonStyle(Qt.ToolButtonTextUnderIcon)
        btn.setToolTip(tooltip)
        btn.setFixedSize(68, 54)
        btn.setIconSize(QSize(20, 20))
        if standard_icon is not None:
            btn.setIcon(self.style().standardIcon(standard_icon))
        role_map = {
            "mode": "ModeActionButton",
            "primary": "PrimaryActionButton",
            "warning": "WarningActionButton",
            "danger": "DangerActionButton",
            "accent": "AccentActionButton",
            "neutral": "NeutralActionButton",
            "normal": "ActionButton",
        }
        btn.setObjectName(role_map.get(role, "ActionButton"))
        return btn

    def make_target_spin(self, minimum, maximum, value):
        spin = QDoubleSpinBox()
        spin.setDecimals(2)
        spin.setRange(minimum, maximum)
        spin.setSingleStep(0.10)
        spin.setValue(value)
        spin.setMinimumHeight(30)
        return spin

    def make_vertical_separator(self):
        line = QFrame()
        line.setObjectName("VerticalSeparator")
        line.setFixedWidth(1)
        line.setMinimumHeight(38)
        return line

    def apply_styles(self):
        self.setStyleSheet("""
            QMainWindow, QWidget {
                background-color: #f4f7fb;
                color: #1f2937;
                font-size: 14px;
                font-family: "Microsoft YaHei", "Noto Sans CJK SC", "Arial";
            }

            #BodyScrollArea, #BodyScrollContent, #ModeBodyStack, #ModeBottomStack {
                background-color: transparent;
                border: none;
            }

            #TopBar, #StatusFrame, #VideoPanel, #RightPanel, #BottomBar {
                background-color: #ffffff;
                border: 1px solid #dbe4ee;
                border-radius: 16px;
            }

            #MiddleFrame {
                background-color: transparent;
                border: none;
            }

            #LogoLabel {
                color: #1d2733;
                font-size: 22px;
                font-weight: bold;
                background: transparent;
                border: none;
            }

            #ModePanel {
                background-color: #eef4fb;
                border: 1px solid #d7e2ef;
                border-radius: 13px;
            }

            #ModeTitleLabel {
                color: #344054;
                font-weight: bold;
                background-color: transparent;
                border: none;
            }

            #ModeCombo {
                min-height: 30px;
                background-color: #ffffff;
                border: 1px solid #cbd5e1;
                border-radius: 9px;
                padding: 2px 8px;
                font-weight: bold;
            }

            #TopStatusLabel {
                color: #991b1b;
                background-color: #fee2e2;
                border-radius: 11px;
                padding: 8px 14px;
                font-weight: bold;
            }

            #TopActionButton, #PanelPrimaryButton, #PanelSecondaryButton, #PanelWarningButton {
                min-height: 34px;
                border: none;
                border-radius: 10px;
                padding: 6px 12px;
                font-weight: bold;
            }

            #TopActionButton, #PanelPrimaryButton {
                color: #ffffff;
                background-color: #2563eb;
            }

            #TopActionButton:hover, #PanelPrimaryButton:hover {
                background-color: #1d4ed8;
            }

            #PanelSecondaryButton {
                color: #1d4ed8;
                background-color: #eaf2ff;
                border: 1px solid #cfe0ff;
            }

            #PanelSecondaryButton:hover {
                background-color: #dbeafe;
            }

            #PanelWarningButton {
                color: #ffffff;
                background-color: #f59e0b;
            }

            #PanelWarningButton:hover {
                background-color: #d97706;
            }

            #StatusCard {
                background-color: #fbfdff;
                border: 1px solid #dfe7f0;
                border-radius: 14px;
            }

            #CardTitle {
                color: #667085;
                font-size: 13px;
                font-weight: bold;
                background-color: #f3f6fa;
                border-radius: 8px;
                padding: 4px 6px;
            }

            #CardValue {
                color: #172033;
                font-size: 14px;
                font-weight: 600;
            }

            #SectionTitle {
                color: #172033;
                font-size: 15px;
                font-weight: bold;
                background-color: #f3f6fb;
                border-radius: 9px;
                padding: 5px 8px;
            }

            #MiniSectionTitle {
                color: #172033;
                font-size: 14px;
                font-weight: bold;
                background-color: transparent;
            }

            #VideoWorkspace {
                background-color: transparent;
                border: none;
            }

            #MainVideoFrame, #PipVideoFrame, #VideoFrame {
                background-color: #ffffff;
                border: 1px solid #dbe4ee;
                border-radius: 14px;
            }

            #PipVideoFrame {
                background-color: transparent;
                border: none;
            }

            #PipVideoFrame #ImageArea {
                border: 2px solid #c7d7ec;
                border-radius: 11px;
                background-color: #eaf0f6;
            }

            #VideoTitle {
                font-size: 14px;
                font-weight: bold;
                color: #1f2937;
                background-color: #f5f8fc;
                border-radius: 8px;
                padding: 4px 6px;
            }

            #ImageArea {
                background-color: #eaf0f6;
                border: 1px solid #d8e2ec;
                border-radius: 11px;
                color: #667085;
                font-weight: 600;
            }

            #PanelCard {
                background-color: #fbfdff;
                border: 1px solid #dfe7f0;
                border-radius: 14px;
            }

            #InlineStatus {
                color: #475467;
                background-color: #eef2f6;
                border-radius: 9px;
                padding: 6px 8px;
                font-size: 13px;
            }

            #CommandHint, #MicroHint {
                color: #667085;
                background-color: #f5f8fc;
                border: 1px solid #e1e8f0;
                border-radius: 9px;
                padding: 6px 8px;
                font-size: 12px;
            }

            #FormLabel {
                color: #344054;
                font-weight: bold;
                background-color: #f5f8fc;
                border-radius: 8px;
                padding: 4px 2px;
            }

            QLineEdit, QComboBox, QDoubleSpinBox {
                background-color: #ffffff;
                border: 1px solid #cfd8e3;
                border-radius: 9px;
                padding: 5px 8px;
                min-height: 28px;
            }

            #LogDialog {
                background-color: #f4f7fb;
            }

            QTextEdit {
                background-color: #ffffff;
                border: 1px solid #d8e2ec;
                border-radius: 11px;
                color: #1f2937;
                padding: 6px;
            }

            #BottomBar {
                background-color: #ffffff;
                border: 1px solid #dbe4ee;
                border-radius: 16px;
            }

            #ActionButton, #ModeActionButton, #PrimaryActionButton, #WarningActionButton,
            #DangerActionButton, #AccentActionButton, #NeutralActionButton {
                border-radius: 15px;
                font-weight: bold;
                font-size: 12px;
                color: #ffffff;
                background-color: #2563eb;
                border: 1px solid #2563eb;
                padding: 4px;
            }

            #VerticalSeparator {
                background-color: #dbe4ee;
                border: none;
                margin-left: 4px;
                margin-right: 4px;
            }

            QToolTip {
                color: #ffffff;
                background-color: #1f2937;
                border: 1px solid #111827;
                border-radius: 6px;
                padding: 5px 8px;
                font-size: 12px;
            }

            QScrollBar:vertical {
                width: 9px;
                background: transparent;
                margin: 4px 2px 4px 2px;
            }
            QScrollBar::handle:vertical {
                background: #cbd5e1;
                border-radius: 4px;
                min-height: 24px;
            }
            QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical {
                height: 0px;
            }
        """)

    def connect_signals(self):
        self.signals.state_signal.connect(self.update_state_ui)
        self.signals.battery_signal.connect(self.update_battery_ui)
        self.signals.odom_signal.connect(self.update_odom_ui)
        self.signals.video1_signal.connect(self.update_video1)
        self.signals.video2_signal.connect(self.update_video2)
        self.signals.log_signal.connect(self.append_log)
        self.signals.remote_status_signal.connect(self.update_remote_status_ui)

    def state_callback(self, msg):
        if self.is_closing:
            return

        self.current_mode = msg.mode
        self.connected = msg.connected
        self.armed = msg.armed
        self.manual_input = msg.manual_input

        self.signals.state_signal.emit(
            msg.mode,
            msg.connected,
            msg.armed,
            msg.manual_input
        )

    def battery_callback(self, msg):
        if self.is_closing:
            return

        self.battery_voltage = msg.voltage
        self.battery_percent = msg.percentage * 100.0 if msg.percentage >= 0.0 else 0.0
        self.signals.battery_signal.emit(self.battery_voltage, self.battery_percent)

    def odom_callback(self, msg):
        if self.is_closing:
            return

        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.z = msg.pose.pose.position.z

        self.roll, self.pitch, self.yaw = quaternion_to_euler(msg.pose.pose.orientation)
        self.has_odom = True

        if not self.home_set:
            self.home_x = self.x
            self.home_y = self.y
            self.home_z = self.z
            self.home_set = True

        self.signals.odom_signal.emit(
            self.x,
            self.y,
            self.z,
            math.degrees(self.roll),
            math.degrees(self.pitch),
            math.degrees(self.yaw)
        )

    def raw_setpoint_callback(self, msg):
        self.raw_setpoint_last_wall_time = time.monotonic()
        self.raw_setpoint_received = True

    def rc_callback(self, msg):
        pass

    def video_callback(self, msg, idx):
        if self.is_closing:
            return

        try:
            qimg = self.rosimg_to_qimage(msg)

            if idx == 1:
                self.signals.video1_signal.emit(qimg)
            else:
                self.signals.video2_signal.emit(qimg)

        except Exception as e:
            self.log("视频流{} 显示错误: {}".format(idx, str(e)))

    def rosimg_to_qimage(self, msg):
        enc = msg.encoding.lower()

        if "bgr" in enc or "rgb" in enc:
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            rgb = cv2.cvtColor(cv_img, cv2.COLOR_BGR2RGB)
            h, w, ch = rgb.shape
            return QImage(rgb.data, w, h, ch * w, QImage.Format_RGB888).copy()

        cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")

        if cv_img.dtype != np.uint8:
            cv_img = np.nan_to_num(cv_img, nan=0.0, posinf=0.0, neginf=0.0)
            cv_img = cv2.normalize(cv_img, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)

        if len(cv_img.shape) == 2:
            rgb = cv2.cvtColor(cv_img, cv2.COLOR_GRAY2RGB)
        else:
            rgb = cv2.cvtColor(cv_img, cv2.COLOR_BGR2RGB)

        h, w, ch = rgb.shape
        return QImage(rgb.data, w, h, ch * w, QImage.Format_RGB888).copy()

    def update_state_ui(self, mode, connected, armed, manual_input):
        conn_text = "已连接" if connected else "未连接"
        arm_text = "已解锁" if armed else "未解锁"
        manual_text = "有效" if manual_input else "无"

        self.card_state.set_value(
            "连接: {}\n模式: {}\n解锁: {}\n手动输入: {}".format(
                conn_text,
                mode,
                arm_text,
                manual_text
            )
        )

        if connected:
            self.top_status_label.setText(
                "MAVROS 已连接 | {} | {}".format(mode, arm_text)
            )
            self.top_status_label.setStyleSheet(
                "color:#166534; background-color:#dcfce7; border-radius:10px; "
                "padding:8px 16px; font-weight:bold;"
            )
        else:
            self.top_status_label.setText("MAVROS 未连接")
            self.top_status_label.setStyleSheet(
                "color:#991b1b; background-color:#fee2e2; border-radius:10px; "
                "padding:8px 16px; font-weight:bold;"
            )

    def update_battery_ui(self, voltage, percent):
        self.card_battery.set_value("{:.2f} V\n{:.1f} %".format(voltage, percent))

    def update_odom_ui(self, x, y, z, roll_deg, pitch_deg, yaw_deg):
        self.card_position.set_value(
            "x={:.2f}\ny={:.2f}\nz={:.2f}\nyaw={:.1f}°".format(
                x, y, z, yaw_deg
            )
        )

        self.attitude_widget.set_attitude(roll_deg, pitch_deg, yaw_deg)

    def update_video1(self, qimg):
        self.video1_qimg = qimg
        self.refresh_video_display()

    def update_video2(self, qimg):
        self.video2_qimg = qimg
        self.refresh_video_display()

    def refresh_video_display(self):
        if self.main_video_index == 1:
            main_img = self.video1_qimg
            sub_img = self.video2_qimg
            main_name = "RGB / 视频流1"
            sub_name = "Depth / 视频流2"
        else:
            main_img = self.video2_qimg
            sub_img = self.video1_qimg
            main_name = "Depth / 视频流2"
            sub_name = "RGB / 视频流1"

        video_pairs = [
            (self.main_video, self.sub_video),
            (getattr(self, "real_main_video", None), getattr(self, "real_sub_video", None)),
        ]

        for main_widget, sub_widget in video_pairs:
            if main_widget is None or sub_widget is None:
                continue

            main_widget.set_title("主视频流 - {}".format(main_name))
            sub_widget.set_title("副视频流 - {}".format(sub_name))

            if main_img is not None:
                main_widget.set_qimage(main_img)

            if sub_img is not None:
                sub_widget.set_qimage(sub_img)

    def refresh_ui(self):
        manual_status = "ON" if self.manual_keepalive_enabled else "OFF"
        setpoint_status = "正常" if self.is_raw_setpoint_fresh() else "无/过期"

        self.card_control.set_value(
            "运行模式: {}\n任务: {}\n虚拟摇杆: {}\nraw setpoint: {}".format(
                self.run_mode,
                self.control_mode,
                manual_status,
                setpoint_status
            )
        )

        ok, reason = self.check_takeoff_ready()
        if ok:
            self.card_ready.set_value("已满足\n可一键起飞")
        else:
            self.card_ready.set_value(reason)

        self.refresh_inline_manager_status()

    def append_log(self, text):
        if self.is_closing:
            return

        now = time.strftime("%H:%M:%S")
        self.log_box.append("[{}] {}".format(now, text))

    def log(self, text):
        if not self.is_closing:
            self.signals.log_signal.emit(text)

    def update_mode_views(self):
        mode_index = 0 if self.run_mode == "GAZEBO" else 1
        if hasattr(self, "mode_body_stack"):
            self.mode_body_stack.setCurrentIndex(mode_index)
        if hasattr(self, "mode_bottom_stack"):
            self.mode_bottom_stack.setCurrentIndex(mode_index)

    def change_run_mode(self, text):
        self.run_mode = text

        if self.run_mode != "GAZEBO":
            self.manual_keepalive_enabled = False

        self.update_mode_views()
        self.refresh_video_display()
        self.log("运行模式切换为: {}".format(self.run_mode))

        if self.run_mode == "GAZEBO":
            self.log("GAZEBO模式：本地仿真控制界面已启用；可自动辅助切换 OFFBOARD。")
        else:
            self.log("REAL模式：机载远程管理界面已启用；OFFBOARD、解锁、上锁、急停请使用遥控器。")

    def get_topic_subscribers(self, topic_name):
        try:
            master = rosgraph.Master(rospy.get_name())
            pubs, subs, srvs = master.getSystemState()

            for topic, nodes in subs:
                if topic == topic_name:
                    return nodes

            return []

        except Exception:
            return []

    def check_position_cmd_controller_ready(self):
        subscribers = self.get_topic_subscribers(self.cmd_topic)

        if len(subscribers) == 0:
            return False, "未检测到 /position_cmd 订阅者\n请先启动 cxr 控制器"

        return True, "OK"

    def is_raw_setpoint_fresh(self):
        if not self.raw_setpoint_received:
            return False

        if self.raw_setpoint_last_wall_time is None:
            return False

        return (time.monotonic() - self.raw_setpoint_last_wall_time) < 0.6

    def check_offboard_switch_ready(self):
        if not self.connected:
            return False, "MAVROS 未连接"

        if not self.has_odom:
            return False, "未收到本地里程计"

        ok, reason = self.check_position_cmd_controller_ready()
        if not ok:
            return False, reason

        if not self.is_raw_setpoint_fresh():
            return False, "未检测到新鲜 raw setpoint\n请确认 cxr 正常运行"

        return True, "OK"

    def check_takeoff_ready(self):
        if not self.connected:
            return False, "MAVROS 未连接"

        if not self.has_odom:
            return False, "未收到本地里程计"

        ok, reason = self.check_position_cmd_controller_ready()
        if not ok:
            return False, reason

        if not self.is_raw_setpoint_fresh():
            return False, "raw setpoint 不新鲜"

        if self.current_mode != "OFFBOARD":
            return False, "当前不是 OFFBOARD"

        if self.run_mode == "REAL" and not self.armed:
            return False, "实机未解锁\n请先使用遥控器解锁"

        return True, "起飞条件满足"

    def publish_virtual_manual_control(self):
        if self.run_mode != "GAZEBO":
            return

        if not self.manual_keepalive_enabled:
            return

        msg = ManualControl()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = ""

        msg.x = 0.0
        msg.y = 0.0
        msg.z = 500.0
        msg.r = 0.0
        msg.buttons = 0

        self.manual_control_pub.publish(msg)

    def start_offboard_switch(self, auto_takeoff_after=False):
        if self.run_mode == "REAL":
            self.log("实机模式：OFFBOARD / 板载模式必须由遥控器切换，地面站不会主动发送模式切换请求。")
            self.log("请先确认控制器已启动，再使用遥控器拨杆切换到 OFFBOARD。")
            return

        if self.current_mode == "OFFBOARD":
            self.log("当前已经是 OFFBOARD")
            if auto_takeoff_after:
                self.launch_takeoff_script()
            return

        ok, reason = self.check_offboard_switch_ready()

        if not ok:
            self.log("不能切换 OFFBOARD：{}".format(reason.replace("\n", "；")))
            return

        self.offboard_switch_active = True
        self.offboard_auto_takeoff_after = auto_takeoff_after
        self.offboard_switch_start_wall = time.monotonic()
        self.offboard_last_request_wall = 0.0
        self.offboard_request_count = 0

        self.manual_keepalive_enabled = True
        self.log("GAZEBO模式：已启用中位虚拟摇杆保活")
        self.log("准备切换 OFFBOARD：先预热手动输入与 setpoint")

        if not self.offboard_timer.isActive():
            self.offboard_timer.start(100)

    def process_offboard_switch(self):
        if not self.offboard_switch_active:
            self.offboard_timer.stop()
            return

        if self.current_mode == "OFFBOARD":
            self.offboard_switch_active = False
            self.offboard_timer.stop()
            self.log("OFFBOARD 切换成功")

            if self.offboard_auto_takeoff_after:
                self.log("OFFBOARD 已就绪，继续执行一键起飞")
                self.launch_takeoff_script()

            return

        if self.offboard_switch_start_wall is None:
            return

        elapsed = time.monotonic() - self.offboard_switch_start_wall

        if elapsed < self.offboard_warmup_sec:
            return

        if self.offboard_request_count >= self.offboard_max_request_count:
            self.offboard_switch_active = False
            self.offboard_timer.stop()
            self.log("OFFBOARD 多次切换失败，请检查 cxr、PX4 参数及 raw setpoint")
            return

        now = time.monotonic()
        if now - self.offboard_last_request_wall < self.offboard_request_interval_sec:
            return

        self.offboard_last_request_wall = now
        self.offboard_request_count += 1

        try:
            rospy.wait_for_service("/mavros/set_mode", timeout=1.0)
            resp = self.mode_client(base_mode=0, custom_mode="OFFBOARD")

            self.log(
                "发送 OFFBOARD 请求 {}/{}，mode_sent={}".format(
                    self.offboard_request_count,
                    self.offboard_max_request_count,
                    resp.mode_sent
                )
            )

        except Exception as e:
            self.log("OFFBOARD 请求异常：{}".format(str(e)))

    def make_separator(self):
        line = QFrame()
        line.setFrameShape(QFrame.HLine)
        line.setObjectName("SoftSeparator")
        return line

    def is_cxr_process_running(self):
        return self.cxr_process is not None and self.cxr_process.poll() is None

    def is_custom_task_running(self):
        return self.custom_task_process is not None and self.custom_task_process.poll() is None

    def refresh_inline_manager_status(self):
        if self.is_custom_task_running():
            self.custom_task_status_label.setText("任务状态：运行中")
            self.custom_task_status_label.setStyleSheet(
                "color:#166534; background-color:#dcfce7; border-radius:8px; padding:6px 8px;"
            )
        elif self.custom_task_process is not None:
            exit_code = self.custom_task_process.poll()
            self.custom_task_status_label.setText("任务状态：已结束，退出码={}".format(exit_code))
            self.custom_task_status_label.setStyleSheet(
                "color:#475467; background-color:#eef2f6; border-radius:8px; padding:6px 8px;"
            )
        else:
            self.custom_task_status_label.setText("任务状态：未启动")
            self.custom_task_status_label.setStyleSheet(
                "color:#475467; background-color:#eef2f6; border-radius:8px; padding:6px 8px;"
            )

    def update_remote_status_ui(self, ok, text):
        if not hasattr(self, "remote_status_label"):
            return

        if ok:
            self.remote_status_label.setStyleSheet(
                "color:#166534; background-color:#dcfce7; border-radius:8px; padding:6px 8px;"
            )
        else:
            self.remote_status_label.setStyleSheet(
                "color:#991b1b; background-color:#fee2e2; border-radius:8px; padding:6px 8px;"
            )

        self.remote_status_label.setText(text)

    def get_remote_connection_info(self):
        host = self.remote_host_input.text().strip() if hasattr(self, "remote_host_input") else self.remote_host_param
        user = self.remote_user_input.text().strip() if hasattr(self, "remote_user_input") else self.remote_user_param
        port_text = self.remote_port_input.text().strip() if hasattr(self, "remote_port_input") else str(self.remote_ssh_port_param)

        if not host:
            raise ValueError("机载电脑 IP 不能为空")
        if not user:
            raise ValueError("SSH 用户名不能为空")

        try:
            port = int(port_text)
        except ValueError:
            raise ValueError("SSH 端口必须是整数")

        if port <= 0 or port > 65535:
            raise ValueError("SSH 端口范围错误")

        return host, user, port

    def run_ssh_async(self, purpose, remote_command, update_status=False):
        def worker():
            try:
                host, user, port = self.get_remote_connection_info()
            except Exception as e:
                self.log("{}失败：{}".format(purpose, str(e)))
                if update_status:
                    self.signals.remote_status_signal.emit(False, "SSH：配置错误 | {}".format(str(e)))
                return

            ssh_cmd = [
                "ssh",
                "-o", "BatchMode=yes",
                "-o", "ConnectTimeout=5",
                "-p", str(port),
                "{}@{}".format(user, host),
                remote_command
            ]

            try:
                result = subprocess.run(
                    ssh_cmd,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE,
                    text=True,
                    timeout=35
                )

                stdout = (result.stdout or "").strip()
                stderr = (result.stderr or "").strip()
                success = result.returncode == 0

                if success:
                    detail = stdout if stdout else "执行成功"
                    self.log("{}：{}".format(purpose, detail))
                    if update_status:
                        self.signals.remote_status_signal.emit(
                            True,
                            "SSH：已连接 | {}@{}:{} | {}".format(user, host, port, detail)
                        )
                else:
                    detail = stderr if stderr else stdout if stdout else "返回码 {}".format(result.returncode)
                    self.log("{}失败：{}".format(purpose, detail))
                    if update_status:
                        self.signals.remote_status_signal.emit(
                            False,
                            "SSH：连接/执行失败 | {}".format(detail)
                        )

            except subprocess.TimeoutExpired:
                self.log("{}失败：SSH 命令超时".format(purpose))
                if update_status:
                    self.signals.remote_status_signal.emit(False, "SSH：连接超时")
            except Exception as e:
                self.log("{}失败：{}".format(purpose, str(e)))
                if update_status:
                    self.signals.remote_status_signal.emit(False, "SSH：{}".format(str(e)))

        threading.Thread(target=worker, daemon=True).start()

    def test_remote_ssh(self):
        self.run_ssh_async(
            "SSH 连通性测试",
            "echo SSH_OK && hostname",
            update_status=True
        )

    def refresh_remote_sessions(self):
        self.run_ssh_async(
            "远程 tmux 会话",
            "tmux list-sessions -F '#S' 2>/dev/null || echo NO_TMUX_SESSIONS",
            update_status=False
        )

    def build_remote_session_name(self, suffix):
        clean = "".join(ch for ch in str(suffix) if ch.isalnum() or ch in ("_", "-"))
        if not clean:
            clean = "custom"
        return "{}_{}".format(self.remote_session_prefix, clean)

    def build_remote_launch_command(self, module_command):
        command = str(module_command or "").strip()
        if not command:
            raise ValueError("远程启动命令未配置")

        setup_cmd = str(self.remote_setup_command or "").strip()
        if setup_cmd:
            return "{} && {}".format(setup_cmd, command)
        return command

    def start_remote_session(self, suffix, display_name, module_command):
        try:
            session_name = self.build_remote_session_name(suffix)
            full_command = self.build_remote_launch_command(module_command)
        except Exception as e:
            self.log("启动{}失败：{}".format(display_name, str(e)))
            return

        inner_command = "bash -lc {}".format(shlex.quote(full_command))
        remote_command = (
            "if tmux has-session -t {session} 2>/dev/null; then "
            "echo '[SESSION_EXISTS] {session}'; "
            "else tmux new-session -d -s {session} {inner} && "
            "echo '[STARTED] {session}'; fi"
        ).format(
            session=shlex.quote(session_name),
            inner=shlex.quote(inner_command)
        )

        self.run_ssh_async("启动{}".format(display_name), remote_command)

    def stop_remote_session(self, suffix, display_name):
        session_name = self.build_remote_session_name(suffix)
        remote_command = (
            "if tmux has-session -t {session} 2>/dev/null; then "
            "tmux send-keys -t {session} C-c; "
            "sleep 1; "
            "tmux kill-session -t {session} 2>/dev/null || true; "
            "echo '[STOPPED] {session}'; "
            "else echo '[NOT_RUNNING] {session}'; fi"
        ).format(session=shlex.quote(session_name))

        self.run_ssh_async("停止{}".format(display_name), remote_command)

    def start_remote_module(self, module_key, display_name):
        command = self.remote_module_commands.get(module_key, "")
        self.start_remote_session(module_key, display_name, command)

    def stop_remote_module(self, module_key, display_name):
        self.stop_remote_session(module_key, display_name)

    def start_remote_custom_task(self):
        if not hasattr(self, "remote_custom_cmd_input"):
            return
        suffix = self.remote_custom_session_input.text().strip() or "custom"
        command = self.remote_custom_cmd_input.text().strip()
        if not command:
            self.log("启动远程自定义脚本失败：请输入远程命令")
            return
        self.start_remote_session(suffix, "远程自定义脚本", command)

    def stop_remote_custom_task(self):
        if not hasattr(self, "remote_custom_session_input"):
            return
        suffix = self.remote_custom_session_input.text().strip() or "custom"
        self.stop_remote_session(suffix, "远程自定义脚本")

    def start_cxr_controller(self):
        if self.is_cxr_process_running():
            self.log("CXR 控制器已由地面站启动并运行中，不重复启动。")
            return

        try:
            self.cxr_process = subprocess.Popen(
                self.cxr_command,
                preexec_fn=os.setsid
            )
            self.log("已启动 CXR 控制器：{}".format(" ".join(self.cxr_command)))
        except Exception as e:
            self.log("启动 CXR 控制器失败：{}".format(str(e)))

        self.refresh_inline_manager_status()

    def stop_cxr_controller(self):
        if not self.is_cxr_process_running():
            self.log("当前没有由地面站启动、且可停止的 CXR 控制器进程。")
            self.log("如 CXR 在外部终端启动，请在对应终端停止。")
            self.refresh_inline_manager_status()
            return

        self.terminate_process_group(
            self.cxr_process,
            " CXR 控制器进程",
            log_result=True
        )
        self.cxr_process = None
        self.refresh_inline_manager_status()

    def build_custom_task_command(self):
        raw_package = self.task_package_input.text().strip() or "cxr_egoctrl_v1"
        raw_name = self.task_name_input.text().strip()

        if not raw_name:
            raise ValueError("请先输入 .py / .launch 名称，或完整命令。")

        command_prefixes = ("rosrun ", "roslaunch ", "python ", "python3 ", "bash ", "sh ")
        if raw_name.startswith(command_prefixes):
            cmd = shlex.split(raw_name)
        elif raw_name.endswith(".launch"):
            if raw_name.startswith("/") or raw_name.startswith("./"):
                cmd = ["roslaunch", raw_name]
            else:
                cmd = ["roslaunch", raw_package, raw_name]
        elif raw_name.endswith(".py"):
            if raw_name.startswith("/") or raw_name.startswith("./"):
                cmd = ["python3", raw_name]
            else:
                cmd = ["rosrun", raw_package, raw_name]
        else:
            # 默认按 rosrun 处理，兼容无扩展名的可执行节点
            cmd = ["rosrun", raw_package, raw_name]

        return cmd

    def start_custom_task(self):
        if self.is_custom_task_running():
            self.log("已有脚本 / launch 正在运行，请先停止后再启动新的任务。")
            return

        try:
            cmd = self.build_custom_task_command()
            self.custom_task_process = subprocess.Popen(
                cmd,
                preexec_fn=os.setsid
            )
            self.custom_task_command = cmd
            self.log("已启动任务：{}".format(" ".join(cmd)))
        except Exception as e:
            self.log("启动任务失败：{}".format(str(e)))

        self.refresh_inline_manager_status()

    def stop_custom_task(self):
        if not self.is_custom_task_running():
            self.log("当前没有由右侧面板启动、且可停止的任务进程。")
            self.refresh_inline_manager_status()
            return

        self.terminate_process_group(
            self.custom_task_process,
            "右侧面板任务进程",
            log_result=True
        )
        self.custom_task_process = None
        self.refresh_inline_manager_status()

    def clear_custom_task_inputs(self):
        self.task_name_input.clear()
        self.task_name_input.setFocus()

    def switch_video(self):
        self.main_video_index = 2 if self.main_video_index == 1 else 1
        self.refresh_video_display()
        self.log("已切换主副视频流")

    def toggle_pip(self):
        self.pip_visible = not self.pip_visible
        self.sub_video.setVisible(self.pip_visible)
        if hasattr(self, "real_sub_video"):
            self.real_sub_video.setVisible(self.pip_visible)
        self.log("副视频{}".format("显示" if self.pip_visible else "隐藏"))

    def toggle_log_dialog(self):
        if self.log_dialog.isVisible():
            self.log_dialog.hide()
            self.btn_toggle_log_window.setText("运行日志")
        else:
            self.log_dialog.show()
            self.log_dialog.raise_()
            self.log_dialog.activateWindow()
            self.btn_toggle_log_window.setText("隐藏日志")

    def arm(self):
        if self.run_mode == "REAL":
            self.log("实机模式：请使用遥控器完成解锁，地面站不代替飞手执行物理安全操作。")
            return

        try:
            rospy.wait_for_service("/mavros/cmd/arming", timeout=2.0)
            resp = self.arm_client(True)

            if resp.success:
                self.log("解锁请求成功")
            else:
                self.log("解锁请求失败")

        except Exception as e:
            self.log("解锁失败: {}".format(str(e)))

    def disarm(self):
        if self.run_mode == "REAL":
            self.log("实机模式：请使用遥控器完成上锁，地面站不执行上锁请求。")
            return

        try:
            rospy.wait_for_service("/mavros/cmd/arming", timeout=2.0)
            resp = self.arm_client(False)

            if resp.success:
                self.log("上锁请求成功")
            else:
                self.log("上锁请求失败")

        except Exception as e:
            self.log("上锁失败: {}".format(str(e)))

    def set_land_mode(self):
        if self.run_mode == "REAL":
            self.log("实机模式：降落模式切换请优先通过遥控器或预先验证过的返航/降落流程完成。")
            self.log("当前地面站不会在 REAL 模式下主动请求 AUTO.LAND。")
            return

        try:
            rospy.wait_for_service("/mavros/set_mode", timeout=2.0)
            resp = self.mode_client(base_mode=0, custom_mode="AUTO.LAND")

            if resp.mode_sent:
                self.log("已请求 AUTO.LAND")
            else:
                self.log("AUTO.LAND 请求失败")

        except Exception as e:
            self.log("切换 AUTO.LAND 失败: {}".format(str(e)))

    def terminate_process_group(self, proc, process_name, log_result=True, timeout=1.5):
        """
        可靠停止由地面站自身启动的子进程组。
        - 先 SIGTERM；
        - 超时未退出再 SIGKILL；
        - 仅清理由本地 Popen 保存的进程，不误杀外部手动启动的节点。
        """
        if proc is None:
            return False

        if proc.poll() is not None:
            return False

        try:
            pgid = os.getpgid(proc.pid)
            os.killpg(pgid, signal.SIGTERM)

            try:
                proc.wait(timeout=timeout)
            except subprocess.TimeoutExpired:
                os.killpg(pgid, signal.SIGKILL)
                proc.wait(timeout=timeout)

            if log_result and not self.is_closing:
                self.log("已停止{}".format(process_name))
            return True

        except ProcessLookupError:
            return False
        except Exception as e:
            if log_result and not self.is_closing:
                self.log("停止{}失败: {}".format(process_name, str(e)))
            return False

    def stop_takeoff_script(self, log_result=True):
        stopped = self.terminate_process_group(
            self.takeoff_process,
            "自动起飞脚本",
            log_result=log_result
        )
        self.takeoff_process = None
        return stopped

    def stop_current_task(self):
        """
        停止当前自动任务：
        1. 停止外部自动起飞脚本；
        2. 停止地面站内部任务，如 8 字轨迹、返航、降落等；
        3. 若有里程计，则切换为当前位置 HOLD 悬停。
        """
        script_was_running = self.stop_takeoff_script(log_result=True)

        if self.run_mode == "REAL":
            self.stop_remote_session("takeoff", "远程自动起飞脚本")
            self.remote_takeoff_requested = False

        old_mode = self.control_mode

        if self.has_odom:
            self.hold_x = self.x
            self.hold_y = self.y
            self.hold_z = self.z
            self.hold_yaw = self.yaw
            self.control_mode = "HOLD"

            if old_mode == "FIGURE8":
                self.log(
                    "已停止 8 字轨迹，切换为当前位置悬停：x={:.2f}, y={:.2f}, z={:.2f}".format(
                        self.hold_x,
                        self.hold_y,
                        self.hold_z
                    )
                )
            elif old_mode not in ["IDLE", "HOLD"]:
                self.log(
                    "已停止当前任务 {}，切换为当前位置悬停：x={:.2f}, y={:.2f}, z={:.2f}".format(
                        old_mode,
                        self.hold_x,
                        self.hold_y,
                        self.hold_z
                    )
                )
            elif script_was_running:
                self.log(
                    "自动起飞脚本已停止，切换为当前位置悬停：x={:.2f}, y={:.2f}, z={:.2f}".format(
                        self.hold_x,
                        self.hold_y,
                        self.hold_z
                    )
                )
            else:
                self.log("当前没有正在运行的轨迹任务，保持当前位置悬停")
        else:
            self.control_mode = "IDLE"
            self.log("已停止任务，但未收到里程计，无法自动切换当前位置悬停")

    def start_takeoff(self):
        if self.run_mode == "REAL":
            if self.current_mode != "OFFBOARD":
                self.log("实机模式：请先使用遥控器切换到 OFFBOARD / 板载模式。")
                return

            if not self.armed:
                self.log("实机模式：请先使用遥控器完成解锁。")
                return

            ok, reason = self.check_takeoff_ready()
            if not ok:
                self.log("实机模式：不能起飞：{}".format(reason.replace("\n", "；")))
                return

            self.launch_takeoff_script()
            return

        # GAZEBO 模式
        if self.current_mode == "OFFBOARD":
            ok, reason = self.check_takeoff_ready()
            if not ok:
                self.log("不能起飞：{}".format(reason.replace("\n", "；")))
                return

            self.launch_takeoff_script()
            return

        self.log("当前不是 OFFBOARD，GAZEBO模式将自动先切 OFFBOARD，再执行一键起飞")
        self.start_offboard_switch(auto_takeoff_after=True)

    def launch_takeoff_script(self):
        ok, reason = self.check_takeoff_ready()

        if not ok:
            self.log("不能启动起飞脚本：{}".format(reason.replace("\n", "；")))
            return

        target_z = self.takeoff_height
        takeoff_time = self.takeoff_time

        if self.run_mode == "GAZEBO":
            if self.takeoff_process is not None and self.takeoff_process.poll() is None:
                self.log("自动起飞脚本已经在运行，不重复启动")
                return

        if self.run_mode == "REAL":
            auto_arm_text = "false"
        else:
            auto_arm_text = "true" if self.takeoff_auto_arm else "false"

        cmd = [
            "rosrun",
            self.takeoff_pkg,
            self.takeoff_script,
            "_target_z:={:.2f}".format(target_z),
            "_takeoff_time:={:.2f}".format(takeoff_time),
            "_auto_arm:={}".format(auto_arm_text),
            "_target_yaw:={:.4f}".format(self.takeoff_target_yaw),
            "_final_hold_time:={:.2f}".format(self.takeoff_final_hold_time)
        ]

        if self.run_mode == "REAL":
            remote_cmd = " ".join(shlex.quote(part) for part in cmd)
            self.log("REAL模式：确认 OFFBOARD 与解锁后，准备通过 SSH/tmux 在机载电脑启动起飞脚本。")
            self.start_remote_session("takeoff", "远程自动起飞脚本", remote_cmd)
            self.remote_takeoff_requested = True
            return

        try:
            self.log("起飞条件满足：MAVROS正常、cxr正常、当前模式=OFFBOARD")
            self.log("启动自动起飞脚本：")
            self.log(" ".join(cmd))

            self.takeoff_process = subprocess.Popen(
                cmd,
                preexec_fn=os.setsid
            )

            self.log(
                "一键起飞已触发：目标高度 {:.2f} m，起飞时间 {:.1f} s，目标 yaw {:.3f} rad，最终保持 {:.1f} s".format(
                    target_z,
                    takeoff_time,
                    self.takeoff_target_yaw,
                    self.takeoff_final_hold_time
                )
            )

        except Exception as e:
            self.log("启动自动起飞脚本失败: {}".format(str(e)))

    def check_task_ready(self):
        if not self.connected:
            return False, "MAVROS 未连接"

        if not self.has_odom:
            return False, "没有收到里程计"

        ok, reason = self.check_position_cmd_controller_ready()
        if not ok:
            return False, reason.replace("\n", "；")

        if not self.is_raw_setpoint_fresh():
            return False, "raw setpoint 不新鲜"

        if self.current_mode != "OFFBOARD":
            return False, "当前不是 OFFBOARD"

        if self.run_mode == "REAL" and not self.armed:
            return False, "实机未解锁，请先使用遥控器解锁"

        return True, "OK"

    def goto_target_point_values(self, x_spin, y_spin, z_spin, yaw_spin, label_prefix="目标点"):
        ok, reason = self.check_task_ready()
        if not ok:
            self.log("不能前往{}：{}".format(label_prefix, reason))
            return

        self.stop_takeoff_script()

        target_x = x_spin.value()
        target_y = y_spin.value()
        target_z = z_spin.value()
        target_yaw_deg = yaw_spin.value()

        self.hold_x = target_x
        self.hold_y = target_y
        self.hold_z = target_z
        self.hold_yaw = math.radians(target_yaw_deg)
        self.control_mode = "HOLD"

        self.log(
            "前往{}：x={:.2f}, y={:.2f}, z={:.2f}, yaw={:.1f}°".format(
                label_prefix,
                target_x,
                target_y,
                target_z,
                target_yaw_deg
            )
        )

    def goto_target_point(self):
        self.goto_target_point_values(
            self.target_x_spin,
            self.target_y_spin,
            self.target_z_spin,
            self.target_yaw_spin,
            label_prefix="仿真目标点"
        )

    def goto_real_target_point(self):
        self.goto_target_point_values(
            self.real_target_x_spin,
            self.real_target_y_spin,
            self.real_target_z_spin,
            self.real_target_yaw_spin,
            label_prefix="实机目标点"
        )

    def start_hold(self):
        ok, reason = self.check_task_ready()
        if not ok:
            self.log("不能悬停：{}".format(reason))
            return

        self.stop_takeoff_script()

        self.hold_x = self.x
        self.hold_y = self.y
        self.hold_z = self.z
        self.hold_yaw = self.yaw
        self.control_mode = "HOLD"

        self.log(
            "当前位置悬停: x={:.2f}, y={:.2f}, z={:.2f}".format(
                self.hold_x,
                self.hold_y,
                self.hold_z
            )
        )

    def start_land(self):
        if self.run_mode == "REAL":
            self.log("实机模式：降落请优先使用遥控器或已验证的独立降落流程。")
            self.log("当前地面站的‘降落’按钮在 REAL 模式下仅提示，不主动切换 AUTO.LAND。")
            return

        if not self.has_odom:
            self.log("没有收到 odom，不能降落")
            return

        self.stop_takeoff_script()

        self.land_start_time = rospy.Time.now().to_sec()
        self.land_start_z = self.z

        self.hold_x = self.x
        self.hold_y = self.y
        self.hold_yaw = self.yaw

        self.control_mode = "LAND"

        self.log("开始降落：请求 AUTO.LAND")
        self.set_land_mode()

    def emergency_stop(self):
        if self.run_mode == "REAL":
            self.log("实机模式：紧急情况请立即使用遥控器接管或切换安全模式。")
            self.log("如仅需停止当前自动任务，请点击‘停止任务’。")
            return

        ok, reason = self.check_task_ready()
        if not ok:
            self.log("不能急停悬停：{}".format(reason))
            return

        self.stop_takeoff_script()

        self.hold_x = self.x
        self.hold_y = self.y
        self.hold_z = self.z
        self.hold_yaw = self.yaw
        self.control_mode = "HOLD"

        self.log("急停悬停：已记录当前位置并持续发布保持指令")

    def start_return_home(self):
        ok, reason = self.check_task_ready()
        if not ok:
            self.log("不能返航：{}".format(reason))
            return

        self.stop_takeoff_script()

        self.hold_x = self.home_x if self.home_set else 0.0
        self.hold_y = self.home_y if self.home_set else 0.0
        self.hold_z = max(self.z, self.takeoff_height)
        self.hold_yaw = self.yaw

        self.control_mode = "HOLD"

        self.log(
            "返航悬停目标: x={:.2f}, y={:.2f}, z={:.2f}".format(
                self.hold_x,
                self.hold_y,
                self.hold_z
            )
        )

    def start_figure8(self):
        ok, reason = self.check_task_ready()
        if not ok:
            self.log("不能执行8字轨迹：{}".format(reason))
            return

        self.stop_takeoff_script()

        self.fig8_center_x = self.x
        self.fig8_center_y = self.y
        self.fig8_center_z = self.z
        self.fig8_yaw = self.yaw
        self.fig8_start_time = rospy.Time.now().to_sec()

        self.control_mode = "FIGURE8"

        self.log(
            "开始执行8字轨迹：中心=({:.2f},{:.2f},{:.2f})，周期={:.1f}s".format(
                self.fig8_center_x,
                self.fig8_center_y,
                self.fig8_center_z,
                self.fig8_period
            )
        )

    def publish_control_command(self):
        if self.is_closing or rospy.is_shutdown() or not self.has_odom:
            return

        now = rospy.Time.now().to_sec()

        if self.control_mode == "IDLE":
            return

        if self.control_mode == "HOLD":
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

    def publish_land_cmd(self, now):
        if self.land_start_time is None:
            return

        descend_speed = 0.25
        t = now - self.land_start_time
        z_cmd = max(0.10, self.land_start_z - descend_speed * t)

        self.publish_position_cmd(
            self.hold_x,
            self.hold_y,
            z_cmd,
            self.hold_yaw
        )

        if z_cmd <= 0.12:
            self.control_mode = "IDLE"
            self.log("已下降到接近地面，建议确认安全后上锁")

    def publish_figure8_cmd(self, now):
        if self.fig8_start_time is None:
            return

        t = now - self.fig8_start_time
        w = 2.0 * math.pi / self.fig8_period

        x = self.fig8_center_x + self.fig8_amp_x * math.sin(w * t)
        y = self.fig8_center_y + self.fig8_amp_y * math.sin(w * t) * math.cos(w * t)
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

    def cleanup_resources(self, reason="程序退出"):
        """
        统一清理入口：
        1. 停止地面站自身任务；
        2. 停止由地面站启动的起飞脚本、自定义脚本和 CXR；
        3. 停止 Qt 定时器；
        4. 注销 ROS 订阅；
        5. 触发 rospy shutdown。
        """
        if self.cleanup_done:
            return

        self.cleanup_done = True
        self.is_closing = True
        self.shutdown_requested = True
        self.control_mode = "IDLE"
        self.manual_keepalive_enabled = False
        self.offboard_switch_active = False

        # 停止由地面站启动的外部进程
        try:
            self.stop_takeoff_script(log_result=False)
        except Exception:
            pass

        try:
            self.terminate_process_group(
                self.custom_task_process,
                "右侧面板任务进程",
                log_result=False
            )
            self.custom_task_process = None
        except Exception:
            pass

        try:
            self.terminate_process_group(
                self.cxr_process,
                " CXR 控制器进程",
                log_result=False
            )
            self.cxr_process = None
        except Exception:
            pass

        try:
            if hasattr(self, "log_dialog") and self.log_dialog is not None:
                self.log_dialog.hide()
        except Exception:
            pass

        # 停止定时器，避免退出过程中继续发布或刷新 UI
        for timer_name in ["cmd_timer", "ui_timer", "manual_timer", "offboard_timer"]:
            try:
                timer = getattr(self, timer_name, None)
                if timer is not None:
                    timer.stop()
            except Exception:
                pass

        # 注销 ROS 订阅
        for sub_name in [
            "state_sub",
            "battery_sub",
            "odom_sub",
            "raw_setpoint_sub",
            "rc_sub",
            "video1_sub",
            "video2_sub",
        ]:
            try:
                sub = getattr(self, sub_name, None)
                if sub is not None:
                    sub.unregister()
            except Exception:
                pass

        try:
            if not rospy.is_shutdown():
                rospy.signal_shutdown(reason)
        except Exception:
            pass

    def request_shutdown(self, reason="外部退出请求"):
        """
        供 Ctrl+C / ROS shutdown 回调安全调用。
        使用 Qt 单次定时器把 close() 切回 GUI 事件循环，确保触发 closeEvent。
        """
        if self.cleanup_done or self.shutdown_requested:
            return

        self.shutdown_requested = True
        try:
            QTimer.singleShot(0, self.close)
        except Exception:
            try:
                self.close()
            except Exception:
                pass

    def closeEvent(self, event):
        self.cleanup_resources("light_gcs_pro closed")
        event.accept()


def main():
    app = QApplication(sys.argv)

    try:
        win = LightGCSPro()

        # 让 Python 能在 Qt 事件循环中及时处理 SIGINT。
        signal_pump_timer = QTimer()
        signal_pump_timer.timeout.connect(lambda: None)
        signal_pump_timer.start(200)

        def handle_sigint(signum, frame):
            win.request_shutdown("Ctrl+C")
            try:
                app.quit()
            except Exception:
                pass

        signal.signal(signal.SIGINT, handle_sigint)
        signal.signal(signal.SIGTERM, handle_sigint)

        # 若 ROS 已进入 shutdown，也让 Qt 窗口跟着退出。
        try:
            rospy.on_shutdown(lambda: win.request_shutdown("ROS shutdown"))
        except Exception:
            pass

        # 无论是窗口右上角关闭，还是 app.quit，都走统一清理。
        app.aboutToQuit.connect(lambda: win.cleanup_resources("Qt aboutToQuit"))

        win.show()
        ret = app.exec_()

        win.cleanup_resources("Qt event loop exited")
        sys.exit(ret)

    except Exception:
        print(traceback.format_exc())


if __name__ == "__main__":
    main()