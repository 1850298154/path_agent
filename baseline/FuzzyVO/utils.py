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


def plot_trajectory_step(agent_list, obstacles, savefig_dir, step, map_xlim, map_ylim):
    """绘制单步轨迹"""
    fig, ax = plt.subplots(figsize=(10, 10))
    colors = plt.cm.tab20(np.linspace(0, 1, 20))

    # 障碍物
    for ob in obstacles:
        ob_x, ob_y, ob_size = ob[0], ob[1], ob[2]
        half_size = ob_size / 2
        rect = Rectangle((ob_x - half_size, ob_y - half_size), ob_size, ob_size,
                        facecolor='forestgreen', alpha=0.3, edgecolor='darkgreen')
        ax.add_patch(rect)

    # 智能体
    for i, agent in enumerate(agent_list):
        color = colors[i % 20]
        if agent.damaged:
            circle = Circle(agent.p, agent.physical_radius, facecolor='red', edgecolor='black', alpha=0.7)
        elif agent.reached_target:
            circle = Circle(agent.p, agent.physical_radius, facecolor='lime', edgecolor='black', alpha=0.7)
        else:
            circle = Circle(agent.p, agent.physical_radius, facecolor=color, edgecolor='black')
        ax.add_patch(circle)
        ax.annotate(str(i), agent.p, textcoords="offset points", xytext=(0, 5), ha='center', fontsize=8)
        ax.scatter(agent.target[0], agent.target[1], marker='D', s=100, color=color, edgecolor='black', alpha=0.5)

        if len(agent.trajectory) > 1:
            traj = np.array(agent.trajectory[-10:])
            ax.plot(traj[:, 0], traj[:, 1], '-', color=color, alpha=0.5, linewidth=1)

    ax.set_xlim(0, map_xlim)
    ax.set_ylim(0, map_ylim)
    ax.set_aspect('equal')
    ax.set_title(f'FuzzyVO - Step {step}')
    ax.grid(True, alpha=0.3)
    plt.savefig(os.path.join(savefig_dir, f'episode-{step}.jpg'), bbox_inches='tight', dpi=100)
    plt.close()


def generate_video(savefig_dir, output_dir, max_steps):
    """生成视频"""
    img_path_list = []
    for step in range(max_steps):
        img_path = os.path.join(savefig_dir, f'episode-{step}.jpg')
        if os.path.exists(img_path):
            img_path_list.append(img_path)

    if not img_path_list:
        print("没有图片文件")
        return

    img = cv2.imread(img_path_list[0])
    height, width = img.shape[:2]

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
    print(f"视频已保存: {video_path}")
