#!/usr/bin/env python3
"""快速精修：基于理论值 [180,0,-90] 小范围搜索（约 20 秒）。
数据从上一个脚本保存（若未保存则重新采集）。"""
import rospy, sys, pickle, os
import numpy as np
import tf.transformations as tr
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped

rospy.init_node('multi_calib_fast', anonymous=True)

def om(m):
    p, q = m.pose.pose.position, m.pose.pose.orientation
    M = tr.quaternion_matrix([q.x,q.y,q.z,q.w]); M[:3,3] = [p.x,p.y,p.z]
    return M

SAMPLES_FILE = '/tmp/opencode/calib_samples.pkl'
samples = []
if os.path.exists(SAMPLES_FILE):
    samples = pickle.load(open(SAMPLES_FILE, 'rb'))
    print('载入已保存的 %d 组数据' % len(samples))

if len(samples) < 20:
    print('重新采集（10 秒，请悬停标签可见处）...', flush=True)
    data = {'u': None, 'g': None, 't': None}
    def cu(m): data['u'] = m
    def cg(m): data['g'] = m
    def ct(m): data['t'] = m
    rospy.Subscriber('/uav0/mavros/local_position/odom', Odometry, cu)
    rospy.Subscriber('/ugv1/mavros/local_position/odom', Odometry, cg)
    rospy.Subscriber('/tag_detector/tag_pose', PoseStamped, ct)
    t0 = rospy.Time.now()
    while len(samples) < 40 and (rospy.Time.now()-t0).to_sec() < 15:
        if data['u'] and data['g'] and data['t']:
            samples.append((om(data['u']), om(data['g']), data['t']))
            data['t'] = None
        rospy.sleep(0.2)
    pickle.dump(samples, open(SAMPLES_FILE, 'wb'))
    print('采到 %d 组' % len(samples))

EXP = np.array([1.5, 0.0, 0.0])
T_tag_bG = tr.translation_matrix([0,0,0.30])

def eval_rpy(rpy):
    R = tr.euler_matrix(rpy[0], rpy[1], rpy[2])[:3,:3]
    Tbc = np.eye(4); Tbc[:3,:3] = R; Tbc[:3,3] = [0,0,-0.17]
    errs = []
    for Tu, Tg, t in samples:
        p, q = t.pose.position, t.pose.orientation
        Tct = tr.quaternion_matrix([q.x,q.y,q.z,q.w]); Tct[:3,3]=[p.x,p.y,p.z]
        out = (Tu @ Tbc @ Tct @ np.linalg.inv(T_tag_bG) @ np.linalg.inv(Tg))[:3,3]
        errs.append(np.linalg.norm(out - EXP))
    return np.mean(errs)

rpy0 = [np.pi, 0, -np.pi/2]
best = rpy0; best_err = eval_rpy(rpy0)
print('理论值 [180,0,-90] 误差: %.3f m' % best_err, flush=True)
for dyaw in np.arange(-3, 3.001, 0.2):
    for dpitch in np.arange(-2, 2.001, 0.2):
        for droll in np.arange(-2, 2.001, 0.2):
            rpy = [np.pi+np.radians(droll), np.radians(dpitch), -np.pi/2+np.radians(dyaw)]
            e = eval_rpy(rpy)
            if e < best_err:
                best_err = e; best = rpy
print('最优 cam_rpy（度）: [%.2f, %.2f, %.2f]  误差 %.3f m' %
      (np.degrees(best[0]), np.degrees(best[1]), np.degrees(best[2]), best_err))
print('写入 sim.yaml 的 cam_rpy（弧度）: [%.8f, %.8f, %.8f]' % tuple(best))
