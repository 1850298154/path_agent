# 完全独立的测试 - 3个智能体，2个障碍物
import numpy as np
import os
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

def main():
    # 创建输出目录
    output_dir = os.path.dirname(sys.argv[1])
    savefig_dir = os.path.join(output_dir, 'savefig')
    os.makedirs(savefig_dir, exist_ok=True)

    # 智能体配置
    agents_start = [[50, 50], [150, 100], [250, 250]]
    agents_target = [[300, 50], [50, 300], [50, 250]]
    agents_type = ['unicycle', 'unicycle', 'unicycle', 'unicycle']

    # 障碍物配置（方形障碍物）
    obstacles = [
        [[150, 150], 40],  # 障碍物1: 中心(150,150), 边长40
        [200, 200, 40]   # 障碍物2: 中心(200,200), 边长40
    ]

    # 运行仿真
    paths = [[] for _ in range(3)]
    max_steps = 50
    step_size = 10.0

    for step in range(max_steps):
        for i in range(3):
            # 简化移动：每个agent朝目标移动一定步数
            if step < 15:
                progress = (step + 1) / 15
                agent_pos = agents_start[i] + (agents_target[i] - agents_start[i]) * progress
            elif step < 30:
                progress = 0.3 + (step - 15) / 45 * 0.7
                agent_pos = agents_start[i] + (agents_target[i] - agents_start[i]) * progress
            else:
                agent_pos = agents_target[i]

            paths[i].append(agent_pos.copy())

        print(f'Step {step}: agent positions saved')

    # 绘制轨迹和障碍物
    fig, ax = plt.subplots(figsize=(10, 8))
    ax.set_xlim(0, 330)
    ax.set_ylim(0, 330)
    ax.set_aspect('equal')

    # 绘制障碍物
    for ob in obstacles:
        center = ob[0]
        size = ob[1]
        rect = matplotlib.patches.Rectangle(
            (center[0] - size/2, center[1] - size/2),
            size, size,
            linewidth=2, edgecolor='black', facecolor='lightgray', alpha=0.5
        )
        ax.add_patch(rect)

    # 绘制轨迹
    colors = ['r', 'g', 'b']
    for i, path in enumerate(paths):
        if len(path) > 0:
            path_arr = np.array(path)
            ax.plot(path_arr[:, 0], path_arr[:, 1],
                   color=colors[i], linewidth=2, alpha=0.8, label=f'Agent {i}')

    # 绘制起点和终点
    for i in range(3):
        ax.scatter(*agents_start[i], color=colors[i], s=100, label=f'Start {i}', zorder=5)
        ax.scatter(*agents_target[i], color=colors[i], marker='x', s=100, label=f'End {i}', zorder=5)

    ax.legend()
    ax.grid(True)
    ax.set_title('3 Agents with 2 Obstacles')
    plt.savefig(os.path.join(savefig_dir, 'trajecotry.jpg'), dpi=150)
    print('轨迹图已生成')
