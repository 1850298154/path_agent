# FuzzyVO基线算法 - 工具函数
import json
import os
import datetime
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Circle, Rectangle
import cv2


def get_timestamp():
    """获取时间戳"""
    return datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")


def create_output_dir(base_dir, timestamp):
    """创建输出目录"""
    output_dir = os.path.join(base_dir, "output", timestamp)
    savefig_dir = os.path.join(output_dir, "savefig")
    os.makedirs(savefig_dir, exist_ok=True)
    return output_dir, savefig_dir


def save_statistics(output_dir, stats):
    """保存统计指标"""
    stats_path = os.path.join(output_dir, "a_statistics.json")
    with open(stats_path, 'w', encoding='utf-8') as f:
        json.dump(stats, f, indent=4)
    print(f"统计指标已保存: {stats_path}")


def calculate_statistics(agent_list, total_steps):
    """计算统计指标"""
    num_agents = len(agent_list)

    success_count = sum(1 for a in agent_list if a.reached_target)
    success_rate = success_count / num_agents

    collision_count = sum(1 for a in agent_list if a.collision)
    collision_rate = collision_count / num_agents

    ex_collision_count = sum(1 for a in agent_list if a.ex_collision)
    ex_collision_rate = ex_collision_count / num_agents

    all_plan_times = []
    for a in agent_list:
        all_plan_times.extend(a.plan_time_list)
    average_planning_time = np.mean(all_plan_times) if all_plan_times else 0.0

    return {
        "success_rate": float(success_rate),
        "collision_rate": float(collision_rate),
        "average_planning_time": float(average_planning_time),
        "ex_collision_rate": float(ex_collision_rate),
        "total_steps": total_steps,
        "success_count": success_count,
        "collision_count": collision_count,
        "ex_collision_count": ex_collision_count
    }


def get_obstacle_vertices(obstacle, radius=0):
    """
    获取障碍物顶点列表（支持膨胀）
    障碍物格式: (x, y, size)，其中(x,y)是左下角坐标

    Args:
        obstacle: (x, y, size)
        radius: 膨胀半径

    Returns:
        [(x1,y1), (x2,y2), (x3,y3), (x4,y4)] 四个顶点
    """
    x, y, size = obstacle[0], obstacle[1], obstacle[2]
    # 膨胀后的左下角和边长
    x_new = x - radius
    y_new = y - radius
    size_new = size + 2 * radius

    # 四个顶点（逆时针）
    vertices = [
        (x_new, y_new),               # 左下
        (x_new + size_new, y_new),    # 右下
        (x_new + size_new, y_new + size_new),  # 右上
        (x_new, y_new + size_new),    # 左上
    ]
    return vertices


def plot_trajectory_step(agent_list, obstacles, savefig_dir, step, map_xlim, map_ylim):
    """
    绘制单步轨迹图

    Args:
        agent_list: 智能体列表
        obstacles: 障碍物列表，格式为 [(x, y, size), ...]
        savefig_dir: 保存目录
        step: 当前步数
        map_xlim, map_ylim: 地图范围
    """
    fig, ax = plt.subplots(figsize=(10, 10))

    # 颜色列表
    colors = plt.cm.tab20(np.linspace(0, 1, 20))

    # 绘制障碍物（原始大小，不膨胀）
    for ob in obstacles:
        vertices = get_obstacle_vertices(ob, radius=0)
        X = [v[0] for v in vertices] + [vertices[0][0]]  # 闭合
        Y = [v[1] for v in vertices] + [vertices[0][1]]
        ax.fill(X, Y, facecolor='forestgreen', alpha=0.3, edgecolor='darkgreen', linewidth=1)

    # 绘制智能体
    for i, agent in enumerate(agent_list):
        color = colors[i % 20]

        # 当前位置（圆形）- 使用安全半径radius（1.0）而非物理半径physical_radius（0.25）
        if agent.damaged:
            circle = Circle(agent.p, agent.radius, facecolor='red', edgecolor='black', alpha=0.7)
        elif agent.reached_target:
            circle = Circle(agent.p, agent.radius, facecolor='lime', edgecolor='black', alpha=0.7)
        else:
            circle = Circle(agent.p, agent.radius, facecolor=color, edgecolor='black')

        ax.add_patch(circle)

        # 智能体编号
        ax.annotate(str(i), agent.p, textcoords="offset points", xytext=(0, 5), ha='center', fontsize=8)

        # 目标位置（菱形）
        ax.scatter(agent.target[0], agent.target[1], marker='D', s=100, color=color, edgecolor='black', alpha=0.5)

        # 绘制轨迹（最近20步）
        if len(agent.trajectory) > 1:
            traj = np.array(agent.trajectory[-20:])
            ax.plot(traj[:, 0], traj[:, 1], '-', color=color, alpha=0.5, linewidth=1)

    ax.set_xlim(0, map_xlim)
    ax.set_ylim(0, map_ylim)
    ax.set_aspect('equal')
    ax.set_title(f'FuzzyVO Baseline - Step {step}')
    ax.grid(True, alpha=0.3)

    # 保存图片
    plt.savefig(os.path.join(savefig_dir, f'episode-{step}.jpg'), bbox_inches='tight', dpi=100)
    plt.close()


def generate_video(savefig_dir, output_dir, max_steps, step_interval=20):
    """生成视频（只使用每隔step_interval步的图片）"""
    img_path_list = []
    for step in range(0, max_steps, step_interval):
        img_path = os.path.join(savefig_dir, f'episode-{step}.jpg')
        if os.path.exists(img_path):
            img_path_list.append(img_path)

    # 加上最后一步
    last_img = os.path.join(savefig_dir, f'episode-{max_steps-1}.jpg')
    if os.path.exists(last_img) and last_img not in img_path_list:
        img_path_list.append(last_img)

    if not img_path_list:
        print("没有找到图片文件，无法生成视频")
        return

    # 读取第一张图片获取尺寸
    img = cv2.imread(img_path_list[0])
    height, width = img.shape[:2]

    # 创建视频写入器
    video_path = os.path.join(output_dir, 'a_video.avi')
    fps = 15
    fourcc = cv2.VideoWriter_fourcc(*'DIVX')
    out = cv2.VideoWriter(video_path, fourcc, fps, (width, height))

    for img_path in img_path_list:
        img = cv2.imread(img_path)
        if img is not None:
            img = cv2.resize(img, (width, height))
            out.write(img)

    out.release()
    print(f"视频已保存到: {video_path}")


def plot_trajectory_final(agent_list, obstacles, output_dir, map_xlim, map_ylim, title='Trajectory'):
    """
    绘制最终完整轨迹图

    Args:
        agent_list: 智能体列表
        obstacles: 障碍物列表
        output_dir: 输出目录
        map_xlim, map_ylim: 地图范围
        title: 图标题
    """
    fig, ax = plt.subplots(figsize=(12, 12))

    # 颜色列表
    colors = plt.cm.tab20(np.linspace(0, 1, 20))

    # 绘制障碍物
    for ob in obstacles:
        vertices = get_obstacle_vertices(ob, radius=0)
        X = [v[0] for v in vertices] + [vertices[0][0]]
        Y = [v[1] for v in vertices] + [vertices[0][1]]
        ax.fill(X, Y, facecolor='forestgreen', alpha=0.3, edgecolor='darkgreen', linewidth=1)

    # 绘制每个智能体的轨迹
    for i, agent in enumerate(agent_list):
        color = colors[i % 20]

        # 起点（方形）
        ax.scatter(agent.start_pos[0], agent.start_pos[1],
                   marker='s', s=60, zorder=3, edgecolor='black', color=color, alpha=0.7)

        # 终点（菱形）
        ax.scatter(agent.target[0], agent.target[1],
                   marker='d', s=60, zorder=3, edgecolor='black', color=color, alpha=0.5)

        # 完整轨迹线
        if len(agent.trajectory) > 1:
            traj = np.array(agent.trajectory)
            ax.plot(traj[:, 0], traj[:, 1], '-', color=color, linewidth=1.5, alpha=0.7)

    ax.set_xlim(0, map_xlim)
    ax.set_ylim(0, map_ylim)
    ax.set_aspect('equal')
    ax.set_title(f'{title} - Final Trajectory', fontsize=14)
    ax.set_xlabel('X', fontsize=12)
    ax.set_ylabel('Y', fontsize=12)
    ax.grid(True, alpha=0.3)

    # 保存
    save_path = os.path.join(output_dir, 'trajectory.jpg')
    plt.savefig(save_path, bbox_inches='tight', dpi=150)
    plt.close()
    print(f"完整轨迹图已保存: {save_path}")
