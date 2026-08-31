#!/usr/bin/env python3
"""诊断：实时打印 高度 vs 标签在 map_uav 中的位置（期望恒定 (1.5, 0, 0.2)）。

如果外参正确，"标签在 map_uav"应恒定；如果它随高度线性漂移 => 外参角度误差。
同时对比两套外参：理论值 [180,0,-90] 和 校准值。
"""
import rospy
import numpy as np
import tf.transformations as tr
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped

rospy.init_node('diag', anonymous=True)

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

# 两套外参
THEORY = [np.pi, 0, -np.pi/2]
CALIB = [3.10668607, 0.02617994, -1.65806279]

def tag_in_map(Tu, Tct, rpy):
    R = tr.euler_matrix(rpy[0], rpy[1], rpy[2])[:3,:3]
    Tbc = np.eye(4); Tbc[:3,:3] = R; Tbc[:3,3] = [0,0,-0.17]
    return (Tu @ Tbc @ Tct)[:3,3]

print('采集 20 秒（请移动无人机：悬停 3m -> 8m，或平移，让标签一直可见）', flush=True)
t0 = rospy.Time.now()
rows = []
while (rospy.Time.now()-t0).to_sec() < 20:
    if data['u'] is not None and data['t'] is not None:
        Tu = om(data['u']); Tg = om(data['g'])
        p, q = data['t'].pose.position, data['t'].pose.orientation
        Tct = tr.quaternion_matrix([q.x,q.y,q.z,q.w]); Tct[:3,3]=[p.x,p.y,p.z]
        h = Tu[2,3]
        t1 = tag_in_map(Tu, Tct, THEORY)
        t2 = tag_in_map(Tu, Tct, CALIB)
        rows.append((h, t1, t2))
        data['t'] = None
    rospy.sleep(0.5)

rows = np.array(rows)
print('\n高度      理论外参:标签在map_uav       校准外参:标签在map_uav')
for h, t1, t2 in rows[::3]:
    print('h=%6.2f   [%6.2f %6.2f %6.2f]       [%6.2f %6.2f %6.2f]' %
          (h, t1[0], t1[1], t1[2], t2[0], t2[1], t2[2]))

lo = rows[rows[:,0] < 5]; hi = rows[rows[:,0] >= 5]
for name, seg in (('低空(<5m)', lo), ('高空(>=5m)', hi)):
    if len(seg):
        m1 = seg[:,1].mean(0); m2 = seg[:,2].mean(0)
        print('\n%s: 理论外参均值 [%.2f %.2f %.2f]  校准外参均值 [%.2f %.2f %.2f]' %
              (name, m1[0], m1[1], m1[2], m2[0], m2[1], m2[2]))
print('\n期望: 标签在 map_uav = (1.5, 0, 0.2) 恒定（无人车在 Gazebo(1.5,0)，标签高 0.4，map_uav 原点≈Gazebo(0,0,0.2)）')
print('哪套外参在低空/高空更接近且一致，哪套就正确')