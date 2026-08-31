#!/usr/bin/env python3
import math
import rospy
from quadrotor_msgs.msg import PositionCommand

def pub_point(pub, rate, x, y, z, yaw=0.0, yaw_dot=0.0):
    msg = PositionCommand()
    msg.position.x = x
    msg.position.y = y
    msg.position.z = z
    msg.yaw = yaw
    msg.yaw_dot = yaw_dot
    pub.publish(msg)
    rate.sleep()

def move_line(pub, rate, x0, y0, z0, x1, y1, z1, v=0.3, yaw=0.0):
    """
    以速度 v (m/s) 从 (x0,y0,z0) 平滑走到 (x1,y1,z1)
    """
    dx, dy, dz = (x1 - x0), (y1 - y0), (z1 - z0)
    dist = math.sqrt(dx*dx + dy*dy + dz*dz)
    if dist < 1e-6:
        return

    T = dist / max(v, 1e-3)
    dt = 1.0 / rate.sleep_dur.to_sec()  # 不用这个
    # 用 rate 的频率算步数更稳
    hz = rate.sleep_dur.to_sec()
    # hz 是周期(秒)，步数 = T / 周期
    steps = max(1, int(T / hz))

    for i in range(steps + 1):
        s = float(i) / float(steps)
        x = x0 + s * dx
        y = y0 + s * dy
        z = z0 + s * dz
        pub_point(pub, rate, x, y, z, yaw=yaw, yaw_dot=0.0)

def main():
    rospy.init_node("circle_position_cmd")
    pub = rospy.Publisher("/position_cmd", PositionCommand, queue_size=1)

    # 发布频率（建议 20~50Hz）
    rate_hz = rospy.get_param("~rate", 30)
    rate = rospy.Rate(rate_hz)

    # 圆参数
    cx = rospy.get_param("~cx", 0.0)
    cy = rospy.get_param("~cy", 0.0)
    cz = rospy.get_param("~cz", 1.0)
    R  = rospy.get_param("~R", 1.0)

    # 速度参数
    v_line   = rospy.get_param("~v_line", 0.4)   # 先走到(1,0,1)的速度
    v_circle = rospy.get_param("~v_circle", 0.5) # 画圆切向速度(越大越快)
    omega = v_circle / max(R, 1e-3)              # 角速度 rad/s

    yaw = rospy.get_param("~yaw", 0.0)

    # 起点和“圆的起点”
    start_x, start_y, start_z = (cx, cy, cz)
    entry_x, entry_y, entry_z = (cx + R, cy, cz)  # (1,0,1) 对应 cx=0,R=1

    # 等待 publisher 建立（避免一开始丢消息）
    rospy.sleep(0.5)

    # 1) 先从 (0,0,1) 平滑到 (1,0,1)
    move_line(pub, rate, start_x, start_y, start_z, entry_x, entry_y, entry_z, v=v_line, yaw=yaw)

    # 2) 从 (1,0,1) 开始匀速画圆
    t0 = rospy.Time.now().to_sec()
    while not rospy.is_shutdown():
        t = rospy.Time.now().to_sec() - t0
        theta = omega * t  # 匀速角度
        x = cx + R * math.cos(theta)
        y = cy + R * math.sin(theta)
        z = cz
        pub_point(pub, rate, x, y, z, yaw=yaw, yaw_dot=0.0)

if __name__ == "__main__":
    main()
