# FuzzyVO基线算法 - 主程序
import sys
import os

# 添加ob_2d路径
ob_2d_path = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))), "ob_2d")
sys.path.insert(0, ob_2d_path)

import time
import numpy as np
from config import Config
from agent import FuzzyVOAgent
from utils import (
    get_timestamp, create_output_dir, save_statistics,
    calculate_statistics, plot_trajectory_step, generate_video
)


def main():
    print("=" * 60)
    print("FuzzyVO基线算法 - 多智能体路径规划")
    print("=" * 60)

    config = Config()

    # 测试模式
    config.set_test_mode(num_agents=5, max_steps=500)  # 500步足以让智能体到达目标

    num_agents = config.get_num_agents()
    max_steps = config.get_max_steps()

    print(f"智能体数量: {num_agents}")
    print(f"最大步数: {max_steps}")

    # 创建输出目录
    timestamp = get_timestamp()
    base_dir = os.path.dirname(os.path.abspath(__file__))
    output_dir, savefig_dir = create_output_dir(base_dir, timestamp)
    print(f"输出目录: {output_dir}\n")

    # 初始化智能体
    agent_list = []
    for i in range(num_agents):
        agent = FuzzyVOAgent(
            index=i,
            start_pos=config.agent_start_list[i],
            target_pos=config.agent_end_list[i],
            config=config
        )
        agent_list.append(agent)
    print(f"已初始化 {num_agents} 个智能体\n")

    # 主循环
    print("开始仿真...")
    start_time = time.time()

    for step in range(max_steps):
        step_start = time.time()

        # 计算速度
        velocity_commands = []
        for agent in agent_list:
            if agent.damaged or agent.reached_target:
                velocity_commands.append(np.zeros(2))
            else:
                v_cmd = agent.compute_velocity_command(agent_list, config.obstacle_list)
                velocity_commands.append(v_cmd)

        # 更新位置
        for agent, v_cmd in zip(agent_list, velocity_commands):
            agent.update(v_cmd)

        # 内部碰撞
        for i in range(len(agent_list)):
            for j in range(i + 1, len(agent_list)):
                if agent_list[i].check_internal_collision(agent_list[j]):
                    if not agent_list[i].damaged:
                        agent_list[i].collision = True
                        agent_list[i].mark_damaged()
                        print(f"  Step {step}: Agent {i} 内部碰撞")
                    if not agent_list[j].damaged:
                        agent_list[j].collision = True
                        agent_list[j].mark_damaged()
                        print(f"  Step {step}: Agent {j} 内部碰撞")

        # 外部碰撞
        for agent in agent_list:
            if not agent.damaged and agent.check_external_collision(config.obstacle_list):
                agent.ex_collision = True
                agent.mark_damaged()
                print(f"  Step {step}: Agent {agent.index} 外部碰撞")

        # 绘图
        if step % 10 == 0 or step == max_steps - 1:
            plot_trajectory_step(agent_list, config.obstacle_list, savefig_dir, step, config.map_xlim, config.map_ylim)

        # 检查完成
        all_done = all(a.reached_target or a.damaged for a in agent_list)
        if all_done:
            print(f"  所有智能体完成于 Step {step}")
            max_steps = step + 1
            break

        # 打印进度
        reached = sum(1 for a in agent_list if a.reached_target)
        damaged = sum(1 for a in agent_list if a.damaged)
        step_time = time.time() - step_start
        if step % 10 == 0:
            print(f"  Step {step}/{max_steps}: 到达={reached}, 损毁={damaged}, 耗时={step_time:.3f}s")

    total_time = time.time() - start_time
    print(f"\n仿真完成，耗时: {total_time:.2f}秒\n")

    # 统计
    stats = calculate_statistics(agent_list, max_steps)

    print("=" * 60)
    print("统计结果:")
    print("=" * 60)
    print(f"成功率: {stats['success_rate']:.4f} ({stats['success_count']}/{num_agents})")
    print(f"碰撞率: {stats['collision_rate']:.4f} ({stats['collision_count']}/{num_agents})")
    print(f"外部碰撞率: {stats['ex_collision_rate']:.4f} ({stats['ex_collision_count']}/{num_agents})")
    print(f"平均规划时间: {stats['average_planning_time']:.6f}秒")
    print()

    save_statistics(output_dir, stats)
    print("正在生成视频...")
    generate_video(savefig_dir, output_dir, max_steps)

    print("\n" + "=" * 60)
    print("程序结束")
    print("=" * 60)

    return stats


if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        print(f"错误: {e}")
        import traceback
        traceback.print_exc()
