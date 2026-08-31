#!/usr/bin/env python3
"""精校准：采集多组实时数据，网格搜索 cam_rpy 使输出最接近期望 (1.5,0,0)。

用法：无人机悬停（任意高度 3~8m），运行本脚本 15 秒采集数据。
输出最优 cam_rpy（在 yaw 附近微调）。
"""
import rospy
import numpy as np
import tf.transformations as tr
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped

rospy.init_node('fine_calib', anonymous=True)

N = 60  # 采样 60 组（每组间隔 ~0.25s）
samples = []

def om(m):
    p, q = m.pose.pose.position, m.pose.pose.orientation
    M = tr.quaternion_matrix([q.x,q.y,q.z,q.w]); M[:3,3] = [p.x,p.y,p.z]
    return M

rospy.Subscriber('/uav0/mavros/local_position/odom', Odometry, lambda m: None, queue_size=1)
rospy.Subscriber('/ugv1/mavros/local_position/odom', Odometry, lambda m: None, queue_size=1)

import threading
data = {'u': None, 'g': None, 't': None}
def cu(m): data['u'] = m
def cg(m): data['g'] = m
def ct(m): data['t'] = m
rospy.Subscriber('/uav0/mavros/local_position/odom', Odometry, cu)
rospy.Subscriber('/ugv1/mavros/local_position/odom', Odometry, cg)
rospy.Subscriber('/tag_detector/tag_pose', PoseStamped, ct)

print('采集数据中（15 秒，请让无人机悬停不动）...', flush=True)
t0 = rospy.Time.now()
while len(samples) < N and (rospy.Time.now() - t0).to_sec() < 20:
    if data['u'] is not None and data['g'] is not None and data['t'] is not None:
        samples.append((om(data['u']), om(data['g']), data['t']))
        data['t'] = None
    rospy.sleep(0.25)
print('采集到 %d 组' % len(samples), flush=True)

EXP = np.array([1.5, 0.0, 0.0])
T_tag_bG = tr.translation_matrix([0,0,0.30])

def chain(T_mU_bU, T_cam_tag, T_mG_bG, R_cam, t_cam):
    T_bU_cam = np.eye(4); T_bU_cam[:3,:3] = R_cam; T_bU_cam[:3,3] = t_cam
    T_mU_mG = T_mU_bU @ T_bU_cam @ T_cam_tag @ np.linalg.inv(T_tag_bG) @ np.linalg.inv(T_mG_bG)
    return T_mU_mG[:3,3]

# 网格搜索 yaw 在 [-90, -80] 度，pitch/roll 在 [-3, 3] 度
best = None
best_err = 1e9
for dyaw in np.arange(-5.0, 5.0, 0.25):
    for dpitch in np.arange(-2.0, 2.0, 0.5):
        for droll in np.arange(-2.0, 2.0, 0.5):
            rpy = [np.pi + np.radians(droll), np.radians(dpitch),
                   -np.pi/2 + np.radians(dyaw)]
            R = tr.euler_matrix(rpy[0], rpy[1], rpy[2])[:3,:3]
            errs = []
            for Tu, Tg, tmsg in samples:
                p, q = tmsg.pose.position, tmsg.pose.orientation
                Tct = tr.quaternion_matrix([q.x,q.y,q.z,q.w]); Tct[:3,3]=[p.x,p.y,p.z]
                out = chain(Tu, Tct, Tg, R, [0,0,-0.17])
                errs.append(np.linalg.norm(out - EXP))
            m = np.mean(errs)
            if m < best_err:
                best_err = m; best = rpy

print('\n最优 cam_rpy（度）: [%.2f, %.2f, %.2f]' %
      (np.degrees(best[0]), np.degrees(best[1]), np.degrees(best[2])))
print('平均误差: %.3f m' % best_err)

# 用最优外参评估所有样本的散布
R = tr.euler_matrix(best[0], best[1], best[2])[:3,:3]
outs = []
for Tu, Tg, tmsg in samples:
    p, q = tmsg.pose.position, tmsg.pose.orientation
    Tct = tr.quaternion_matrix([q.x,q.y,q.z,q.w]); Tct[:3,3]=[p.x,p.y,p.z]
    outs.append(chain(Tu, Tct, Tg, R, [0,0,-0.17]))
outs = np.array(outs)
print('输出散布: x[%.2f~%.2f] y[%.2f~%.2f] z[%.2f~%.2f]' %
      (outs[:,0].min(), outs[:,0].max(), outs[:,1].min(), outs[:,1].max(),
       outs[:,2].min(), outs[:,2].max()))
print('输出均值: [%.2f, %.2f, %.2f]' % tuple(outs.mean(0)))
print('\n写入 sim.yaml 的 cam_rpy（弧度）: [%.8f, %.8f, %.8f]' % tuple(best))