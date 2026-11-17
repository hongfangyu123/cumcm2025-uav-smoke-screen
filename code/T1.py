import numpy as np

# 定义参数和变量
g = 9.8
R = 10.0                 
v_c = 3.0             
t_window = 20.0      
R0_U1 = np.array([17800.0, 0.0, 1800.0])
v_U1 = 120.0        
r0_M1 = np.array([20000.0, 0.0, 2000.0])
v_M1 = 300.0
origin = np.array([0.0, 0.0, 0.0])
t_d_11 = 1.5
t_e_11_interval = 3.6
t_e_11 = t_d_11 + t_e_11_interval
T_R = 7.0
T_H = 10.0
T_center_y = 200.0

r_d_11 = R0_U1 + np.array([-v_U1 * t_d_11, 0.0, 0.0])
v_G11 = np.array([-v_U1, 0.0, 0.0])
c0_11 = r_d_11 + v_G11 * t_e_11_interval + np.array([0.0, 0.0, -0.5 * g * t_e_11_interval**2])

# 计算导弹M1在时刻t的位置向量 (r_M1(t))
def r_M1(t):
    d = (origin - r0_M1) / np.linalg.norm(origin - r0_M1)
    return r0_M1 + v_M1 * t * d

# 计算烟幕云团C11在时刻t的位置向量 (c_11(t))
def c_11(t):
    return c0_11 + np.array([0.0, 0.0, -v_c * (t - t_e_11)])

# 计算点C到线段AB的最短距离
def point_segment_distance(C, A, B):
    u = B - A
    uu = np.dot(u, u)
    if uu == 0.0:
        return np.linalg.norm(C - A)
    s = np.dot(C - A, u) / uu
    s_clamped = max(0.0, min(1.0, s))
    P = A + s_clamped * u
    return np.linalg.norm(C - P)

# 生成真目标圆柱体表面的采样点（用于遮蔽判定）
def cylinder_surface_points(n_theta=108, n_z=30):
    thetas = np.linspace(0, 2*np.pi, n_theta, endpoint=False)
    zs = np.linspace(0.0, T_H, n_z)
    pts = []
    for th in thetas:
        x = T_R * np.cos(th)
        y = T_center_y + T_R * np.sin(th)
        for z in zs:
            pts.append(np.array([x, y, z]))
    return pts

Q_pts = cylinder_surface_points()

# 判断时刻t烟幕云团是否对导弹形成有效遮蔽
def is_covered_cylinder(t):
    C = c_11(t)
    M = r_M1(t)
    if np.linalg.norm(M - C) <= R:
        return True
    for B in Q_pts:
        if point_segment_distance(C, M, B) <= R:
            return True
    return False

dt = 0.01
ts = np.arange(t_e_11, t_e_11 + t_window + dt, dt)
cover_flags = [is_covered_cylinder(t) for t in ts]

segments = []
if any(cover_flags):
    start = None
    for t, flag in zip(ts, cover_flags):
        if flag and start is None:
            start = t
        elif not flag and start is not None:
            segments.append((start, t - dt))
            start = None
    if start is not None:
        segments.append((start, ts[-1]))
total_cover = sum(b - a for a, b in segments)

print(f"有效遮蔽时长: {total_cover:.4f} s")

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
from matplotlib.collections import PatchCollection

# 定义参数和变量 (保持不变)
g = 9.8
R = 10.0                 
v_c = 3.0             
t_window = 20.0      
R0_U1 = np.array([17800.0, 0.0, 1800.0])
v_U1 = 120.0        
r0_M1 = np.array([20000.0, 0.0, 2000.0])
v_M1 = 300.0
origin = np.array([0.0, 0.0, 0.0])
t_d_11 = 1.5
t_e_11_interval = 3.6
t_e_11 = t_d_11 + t_e_11_interval
T_R = 7.0
T_H = 10.0
T_center_y = 200.0

r_d_11 = R0_U1 + np.array([-v_U1 * t_d_11, 0.0, 0.0])
v_G11 = np.array([-v_U1, 0.0, 0.0])
c0_11 = r_d_11 + v_G11 * t_e_11_interval + np.array([0.0, 0.0, -0.5 * g * t_e_11_interval**2])

# 计算导弹M1在时刻t的位置向量 (r_M1(t))
def r_M1(t):
    d = (origin - r0_M1) / np.linalg.norm(origin - r0_M1)
    return r0_M1 + v_M1 * t * d

# 计算烟幕云团C11在时刻t的位置向量 (c_11(t))
def c_11(t):
    return c0_11 + np.array([0.0, 0.0, -v_c * (t - t_e_11)])

# 计算点C到线段AB的最短距离
def point_segment_distance(C, A, B):
    u = B - A
    uu = np.dot(u, u)
    if uu == 0.0:
        return np.linalg.norm(C - A)
    s = np.dot(C - A, u) / uu
    s_clamped = max(0.0, min(1.0, s))
    P = A + s_clamped * u
    return np.linalg.norm(C - P)

# 生成真目标圆柱体表面的采样点（用于遮蔽判定）
def cylinder_surface_points(n_theta=108, n_z=30):
    thetas = np.linspace(0, 2*np.pi, n_theta, endpoint=False)
    zs = np.linspace(0.0, T_H, n_z)
    pts = []
    for th in thetas:
        x = T_R * np.cos(th)
        y = T_center_y + T_R * np.sin(th)
        for z in zs:
            pts.append(np.array([x, y, z]))
    return pts

Q_pts = cylinder_surface_points()

# 判断时刻t烟幕云团是否对导弹形成有效遮蔽
def is_covered_cylinder(t):
    C = c_11(t)
    M = r_M1(t)
    if np.linalg.norm(M - C) <= R:
        return True
    for B in Q_pts:
        if point_segment_distance(C, M, B) <= R:
            return True
    return False

dt = 0.01
ts = np.arange(t_e_11, t_e_11 + t_window + dt, dt)
cover_flags = [is_covered_cylinder(t) for t in ts]

segments = []
if any(cover_flags):
    start = None
    for t, flag in zip(ts, cover_flags):
        if flag and start is None:
            start = t
        elif not flag and start is not None:
            segments.append((start, t - dt))
            start = None
    if start is not None:
        segments.append((start, ts[-1]))
total_cover = sum(b - a for a, b in segments)

print(f"有效遮蔽时长: {total_cover:.4f} s")

# 画图

def plot_cover_status(ts, cover_flags, segments, t_start, t_window, total_cover):
    plt.figure(figsize=(12, 6))

    t_values = np.array(ts)
    cover_array = np.array(cover_flags, dtype=int)
    plt.step(t_values, cover_array, where='post', color='b', linewidth=1.5, 
             label='遮蔽状态 (1=遮蔽, 0=未遮蔽)')
    plt.axvline(x=t_start, color='r', linestyle='--', alpha=0.7, 
                label=f'烟幕起爆时间 ({t_start:.1f}s)')
    t_end = t_start + t_window
    plt.axvline(x=t_end, color='m', linestyle='--', alpha=0.7, 
                label=f'烟幕结束时间 ({t_end:.1f}s)')
    for i, (start, end) in enumerate(segments):
        plt.axvspan(start, end, color='green', alpha=0.2)
        mid = (start + end) / 2
        plt.text(mid, 0.5, f'{end - start:.2f}s', 
                 ha='center', va='center', fontsize=10, 
                 bbox=dict(facecolor='white', alpha=0.7))
    
    plt.text(t_start + 0.1, 0.9, f'总有效遮蔽时间: {total_cover:.4f}s', 
             fontsize=12, bbox=dict(facecolor='white', alpha=0.8))
    plt.title('导弹在飞行过程中的遮蔽状态时间序列', fontsize=14)
    plt.xlabel('时间 (s)', fontsize=12)
    plt.ylabel('遮蔽状态', fontsize=12)
    
    plt.xlim(t_start - 0.5, t_end + 0.5)
    plt.ylim(-0.1, 1.1)
    
    plt.legend(loc='upper right', fontsize=10)
    plt.grid(True, alpha=0.3, linestyle='--')
    
    plt.text(t_d_11, -0.15, f'投放时间\n({t_d_11:.1f}s)', 
             ha='center', va='top', fontsize=9)
    plt.text(t_e_11, -0.15, f'起爆时间\n({t_e_11:.1f}s)', 
             ha='center', va='top', fontsize=9)
    
    plt.tight_layout()
    plt.show()

def plot_distance_time_focused(ts, segments, t_start, t_window, R):
    focus_start = min(segments[0][0] - 1.0, t_start)
    focus_end = max(segments[-1][1] + 1.0, t_start + 2.0)
    focus_ts = np.arange(focus_start, focus_end + dt, dt)
    min_distances = []
    for t in focus_ts:
        C = c_11(t)
        M = r_M1(t)
        dist_to_cloud = np.linalg.norm(M - C)
        min_dist_to_cylinder = min(point_segment_distance(C, M, B) for B in Q_pts)
        min_distances.append(min(dist_to_cloud, min_dist_to_cylinder))
    
    min_distances = np.array(min_distances)
    plt.figure(figsize=(12, 6))
    plt.plot(focus_ts, min_distances, color='blue', linewidth=1.5, 
             label='导弹与真目标视线到烟幕中心的最小距离')
    plt.axhline(y=R, color='red', linestyle='--', linewidth=1.5,
                label=f'烟幕有效半径 (R={R}m)')
    for i, (start, end) in enumerate(segments):
        plt.axvspan(start, end, color='green', alpha=0.2, 
                    label='有效遮蔽时段' if i == 0 else "")
        mid = (start + end) / 2
        plt.text(mid, R + 1, f'{end - start:.2f}s', 
                 ha='center', va='bottom', fontsize=10, 
                 bbox=dict(facecolor='white', alpha=0.7))
    plt.fill_between(focus_ts, 0, min_distances, 
                     where=(min_distances <= R), 
                     color='lightgreen', alpha=0.3,
                     label='有效遮蔽区域')
    plt.title('导弹飞行过程中到烟幕中心的距离变化', fontsize=14)
    plt.xlabel('时间 (s)', fontsize=12)
    plt.ylabel('距离 (m)', fontsize=12)
    plt.xlim(focus_start, focus_end)
    plt.ylim(0, max(min_distances) * 1.1)
    
    plt.legend(loc='upper right', fontsize=10)
    
    plt.grid(True, alpha=0.3, linestyle='--')
    
    plt.axvline(x=t_d_11, color='purple', linestyle=':', alpha=0.7)
    plt.text(t_d_11, plt.ylim()[1]*0.95, f'烟幕弹投放\n({t_d_11:.1f}s)', 
             ha='center', va='top', fontsize=9)
    
    plt.axvline(x=t_e_11, color='orange', linestyle=':', alpha=0.7)
    plt.text(t_e_11, plt.ylim()[1]*0.85, f'烟幕弹起爆\n({t_e_11:.1f}s)', 
             ha='center', va='top', fontsize=9)
    
    plt.tight_layout()
    plt.show()

# 绘制遮蔽状态时间序列图
plot_cover_status(ts, cover_flags, segments, t_e_11, t_window, total_cover)

# 绘制聚焦距离图
plot_distance_time_focused(ts, segments, t_e_11, t_window, R)
