#!/usr/bin/env python3
"""多高度自动校准：
通过 /position_cmd 控制无人机依次飞到 3m、8m（标签正上方），
每段自动采集数据，合并最小化误差搜索 cam_rpy。

前提：cxr_egoctrl 控制器运行中，且已切 OFFBOARD。
坐标系：position_cmd 为 ENU（z 向上为正，与控制器的 setpoint_raw/local 一致）。
"""
import rospy, sys
import numpy as np
import tf.transformations as tr
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
try:
    from quadrotor_msgs.msg import PositionCommand
except ImportError:
    import os
    import rospkg
    _ws = rospkg.RosPack().get_path('quadrotor_msgs').split('/src/')[0]
    sys.path.insert(0, os.path.join(_ws, 'devel', 'lib', 'python3', 'dist-packages'))
    from quadrotor_msgs.msg import PositionCommand

rospy.init_node('multi_calib_auto', anonymous=True)

def om(m):
    p, q = m.pose.pose.position, m.pose.pose.orientation
    M = tr.quaternion_matrix([q.x,q.y,q.z,q.w]); M[:3,3] = [p.x,p.y,p.z]
    return M

data = {'u': None, 'g': None, 't': None}
def cu(m): data['u'] = m
def cg(m): data['g'] = m
def ct(m): data['t'] = m
rospy.Subscriber('/uav0/mavros/local_position/odom', Odometry, cu)
rospy.Subscriber('/ugv1/mavros/local_position/odom', Odometry, cg)
rospy.Subscriber('/tag_detector/tag_pose', PoseStamped, ct)

cmd_pub = rospy.Publisher('/position_cmd', PositionCommand, queue_size=5)

def send_goal(x, y, z):
    msg = PositionCommand()
    msg.header.stamp = rospy.Time.now()
    msg.position.x = x; msg.position.y = y; msg.position.z = z
    msg.velocity.x = 0; msg.velocity.y = 0; msg.velocity.z = 0
    msg.yaw = 0.0
    msg.trajectory_id = 0
    msg.trajectory_flag = 0
    for _ in range(5):
        cmd_pub.publish(msg)
        rospy.sleep(0.02)

def go_and_hold(x, y, z, tol=0.35, timeout=60):
    """发目标并持续发布，直到无人机到达并稳定。返回是否到达。"""
    t0 = rospy.Time.now()
    arrived_at = None
    while (rospy.Time.now()-t0).to_sec() < timeout:
        send_goal(x, y, z)
        if data['u'] is not None:
            p = data['u'].pose.pose.position
            err = abs(p.z - z)
            if err < tol:
                if arrived_at is None:
                    arrived_at = rospy.Time.now()
                elif (rospy.Time.now()-arrived_at).to_sec() > 4:
                    return True
            else:
                arrived_at = None
        rospy.sleep(0.05)
    return False

def collect(n, timeout):
    got = 0
    t0 = rospy.Time.now()
    while got < n and (rospy.Time.now()-t0).to_sec() < timeout:
        if data['u'] is not None and data['g'] is not None and data['t'] is not None:
            samples.append((om(data['u']), om(data['g']), data['t']))
            data['t'] = None
            got += 1
        rospy.sleep(0.2)
    return got

samples = []
GX, GY = 1.5, 0.0   # 无人车位置（标签正上方）

print('=== 等待数据源（5 秒）...', flush=True)
rospy.sleep(5)
if data['u'] is None:
    print('没有 odom 数据，检查仿真/控制器！'); exit(1)

print('=== 第 1 段：飞到 (1.5, 0, 3m) ===', flush=True)
ok = go_and_hold(GX, GY, 3.0)
if not ok:
    print('警告：3m 未稳定到达，继续尝试采集'); 
n = collect(25, 20)
print('  3m 采到 %d 组' % n, flush=True)

print('=== 第 2 段：飞到 (1.5, 0, 8m) ===', flush=True)
ok = go_and_hold(GX, GY, 8.0)
if not ok:
    print('警告：8m 未稳定到达，继续尝试采集')
n = collect(25, 30)
print('  8m 采到 %d 组' % n, flush=True)

print('=== 第 3 段：回到 (1.5, 0, 4m) ===', flush=True)
go_and_hold(GX, GY, 4.0)

if len(samples) < 20:
    print('数据不足（%d 组）！' % len(samples)); exit(1)

EXP = np.array([1.5, 0.0, 0.0])
T_tag_bG = tr.translation_matrix([0,0,0.30])

def eval_rpy(rpy):
    R = tr.euler_matrix(rpy[0], rpy[1], rpy[2])[:3,:3]
    T_bU_cam = np.eye(4); T_bU_cam[:3,:3] = R; T_bU_cam[:3,3] = [0,0,-0.17]
    errs = []
    for Tu, Tg, tmsg in samples:
        p, q = tmsg.pose.position, tmsg.pose.orientation
        Tct = tr.quaternion_matrix([q.x,q.y,q.z,q.w]); Tct[:3,3]=[p.x,p.y,p.z]
        out = (Tu @ T_bU_cam @ Tct @ np.linalg.inv(T_tag_bG) @ np.linalg.inv(Tg))[:3,3]
        errs.append(np.linalg.norm(out - EXP))
    return np.mean(errs)

rpy0 = [np.pi, 0, -np.pi/2]
best = rpy0; best_err = eval_rpy(rpy0)
print('\n理论值 [180,0,-90] 平均误差: %.3f m' % best_err, flush=True)
for dyaw in np.arange(-6, 6, 0.2):
    for dpitch in np.arange(-4, 4, 0.2):
        for droll in np.arange(-4, 4, 0.2):
            rpy = [np.pi+np.radians(droll), np.radians(dpitch), -np.pi/2+np.radians(dyaw)]
            e = eval_rpy(rpy)
            if e < best_err:
                best_err = e; best = rpy

print('最优 cam_rpy（度）: [%.2f, %.2f, %.2f]  平均误差 %.3f m' %
      (np.degrees(best[0]), np.degrees(best[1]), np.degrees(best[2]), best_err))

R = tr.euler_matrix(best[0], best[1], best[2])[:3,:3]
T_bU_cam = np.eye(4); T_bU_cam[:3,:3] = R; T_bU_cam[:3,3] = [0,0,-0.17]
outs = []
for Tu, Tg, tmsg in samples:
    p, q = tmsg.pose.position, tmsg.pose.orientation
    Tct = tr.quaternion_matrix([q.x,q.y,q.z,q.w]); Tct[:3,3]=[p.x,p.y,p.z]
    outs.append(((Tu @ T_bU_cam @ Tct @ np.linalg.inv(T_tag_bG) @ np.linalg.inv(Tg))[:3,3],
                 Tu[2,3]))
outs = np.array(outs)
lo = outs[outs[:,1] < 5]; hi = outs[outs[:,1] >= 5]
if len(lo): print('低空段输出均值:', lo[:,0].mean(0).round(2), ' 散布:', (lo[:,0].max(0)-lo[:,0].min(0)).round(2))
if len(hi): print('高空段输出均值:', hi[:,0].mean(0).round(2), ' 散布:', (hi[:,0].max(0)-hi[:,0].min(0)).round(2))
print('写入 sim.yaml 的 cam_rpy（弧度）: [%.8f, %.8f, %.8f]' % tuple(best))