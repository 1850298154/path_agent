import json
import numpy as np

# 读取所有数据
with open('data/01_original_data/uav_positions_over_time.json', 'r', encoding='utf-8') as f:
    uav_data = json.load(f)

with open('data/01_original_data/result_criticalpath_new.json', 'r', encoding='utf-8') as f:
    schedule_data = json.load(f)

with open('data/01_original_data/precomputed_data.json', 'r', encoding='utf-8') as f:
    precomputed = json.load(f)

schedule = schedule_data['task_schedule']

print("="*80)
print("UAV集群任务执行多时刻快照分析")
print("="*80)
print(f"\n总体信息:")
print(f"  总时长: {uav_data['makespan']}s ({uav_data['makespan']/60:.1f}分钟)")
print(f"  UAV数量: {len(uav_data['uavs'])}架")
print(f"  任务数量: {len(schedule)}个")
print()

# 时间点分析
time_points = [0, 440, 668, 764, 830, 885]
time_labels = ['(a)', '(b)', '(c)', '(d)', '(e)', '(f)']

for idx, (t, label) in enumerate(zip(time_points, time_labels)):
    print(f"\n{'='*80}")
    print(f"{label} 时间 t = {t}s ({t/60:.1f}分钟)")
    print(f"{'='*80}")

    # 任务状态分类
    active_tasks = []
    completed_tasks = []
    pending_tasks = []

    for task_id, times in schedule.items():
        task_info = {
            'id': int(task_id),
            'start': times['start'],
            'end': times['end'],
            'duration': times['end'] - times['start']
        }

        if times['start'] <= t <= times['end']:
            task_info['progress'] = (t - times['start']) / task_info['duration'] * 100
            active_tasks.append(task_info)
        elif times['end'] < t:
            completed_tasks.append(task_info)
        else:
            pending_tasks.append(task_info)

    active_tasks.sort(key=lambda x: x['id'])
    completed_tasks.sort(key=lambda x: x['end'], reverse=True)
    pending_tasks.sort(key=lambda x: x['start'])

    print(f"\n【任务状态统计】")
    print(f"  已完成: {len(completed_tasks)}个 ({len(completed_tasks)/len(schedule)*100:.1f}%)")
    print(f"  正在执行: {len(active_tasks)}个 ({len(active_tasks)/len(schedule)*100:.1f}%)")
    print(f"  待执行: {len(pending_tasks)}个 ({len(pending_tasks)/len(schedule)*100:.1f}%)")

    # 正在执行的任务详情
    if active_tasks:
        print(f"\n【正在执行的任务】")
        for task in active_tasks:
            task_id = task['id']
            task_pos = precomputed['task_positions'].get(str(task_id), 'Unknown')

            # 查找任务需要的技能
            skill_a = precomputed['task_skills_A'].get(str(task_id), 0)
            skill_b = precomputed['task_skills_B'].get(str(task_id), 0)

            print(f"  任务{task_id}:")
            print(f"    时间: {task['start']:.1f}s-{task['end']:.1f}s (时长{task['duration']:.1f}s)")
            print(f"    进度: {task['progress']:.1f}% (已执行{t-task['start']:.1f}s, 剩余{task['end']-t:.1f}s)")
            print(f"    位置: {task_pos}")
            print(f"    技能需求: A={skill_a}, B={skill_b}")

            # 查找前驱和后继任务
            preds = precomputed['predecessors'].get(str(task_id), [])
            succs = precomputed['successors'].get(str(task_id), [])
            if preds:
                print(f"    前驱任务: {preds}")
            if succs:
                print(f"    后继任务: {succs}")

    # 最近完成的任务
    if completed_tasks:
        print(f"\n【最近完成的任务】(前3个)")
        for task in completed_tasks[:3]:
            task_id = task['id']
            skill_a = precomputed['task_skills_A'].get(str(task_id), 0)
            skill_b = precomputed['task_skills_B'].get(str(task_id), 0)
            print(f"  任务{task_id}: 完成{t-task['end']:.1f}s前, "
                  f"时长{task['duration']:.1f}s, 技能A={skill_a}, B={skill_b}")

    # 即将开始的任务
    if pending_tasks:
        print(f"\n【即将开始的任务】(前3个)")
        for task in pending_tasks[:3]:
            task_id = task['id']
            task_pos = precomputed['task_positions'].get(str(task_id), 'Unknown')
            skill_a = precomputed['task_skills_A'].get(str(task_id), 0)
            skill_b = precomputed['task_skills_B'].get(str(task_id), 0)
            print(f"  任务{task_id}: {task['start']:.1f}s开始 (还有{task['start']-t:.1f}s), "
                  f"时长{task['duration']:.1f}s, 技能A={skill_a}, B={skill_b}")

    # UAV位置和状态分析
    print(f"\n【UAV空间分布与战术密度】")
    positions_at_t = []
    moving_uavs = []
    static_uavs = []

    for uav_id, uav_info in uav_data['uavs'].items():
        pos_idx = min(int(t), len(uav_info['positions'])-1)
        pos = uav_info['positions'][pos_idx]

        # 判断是否在移动
        is_moving = False
        if pos_idx > 0:
            prev_pos = uav_info['positions'][pos_idx-1]
            is_moving = (abs(pos[0]-prev_pos[0]) > 0.01 or abs(pos[1]-prev_pos[1]) > 0.01)

        uav_status = {
            'id': int(uav_id),
            'pos': pos,
            'moving': is_moving
        }

        positions_at_t.append(uav_status)
        if is_moving:
            moving_uavs.append(uav_status)
        else:
            static_uavs.append(uav_status)

    print(f"  移动中UAV: {len(moving_uavs)}架")
    print(f"  静止UAV: {len(static_uavs)}架")

    # 计算空间分布
    if positions_at_t:
        xs = [p['pos'][0] for p in positions_at_t]
        ys = [p['pos'][1] for p in positions_at_t]

        x_min, x_max = min(xs), max(xs)
        y_min, y_max = min(ys), max(ys)
        x_range = x_max - x_min
        y_range = y_max - y_min

        area = max(x_range * y_range, 0.01)  # 避免除零
        density = len(positions_at_t) / area

        print(f"  X坐标范围: [{x_min:.2f}, {x_max:.2f}] (跨度{x_range:.2f})")
        print(f"  Y坐标范围: [{y_min:.2f}, {y_max:.2f}] (跨度{y_range:.2f})")
        print(f"  战术密度: {density:.3f} UAV/单位面积")

        # 计算中心点
        center_x = sum(xs) / len(xs)
        center_y = sum(ys) / len(ys)
        print(f"  集群中心: ({center_x:.2f}, {center_y:.2f})")

        # 计算平均间距
        distances = []
        for i in range(min(len(positions_at_t), 20)):  # 只计算前20个避免太慢
            for j in range(i+1, min(len(positions_at_t), 20)):
                p1 = positions_at_t[i]['pos']
                p2 = positions_at_t[j]['pos']
                dist = np.sqrt((p1[0]-p2[0])**2 + (p1[1]-p2[1])**2)
                distances.append(dist)

        if distances:
            avg_dist = sum(distances) / len(distances)
            print(f"  平均UAV间距: {avg_dist:.2f}")

    # 阶段特征总结
    print(f"\n【阶段特征】")
    if t == 0:
        print("  - 初始阶段，所有UAV静止在起始位置")
        print("  - 任务即将开始，集群处于待命状态")
        print("  - UAV密集排列在起始区域")
    elif t < 200:
        print("  - 早期阶段，首批任务正在执行")
        print("  - UAV开始向任务区域移动")
        print("  - 集群开始分散展开")
    elif t < 600:
        print("  - 中期阶段，任务执行高峰")
        print("  - UAV活跃移动，多个任务并行执行")
        print("  - 集群在任务区域灵活分布")
    else:
        print("  - 后期阶段，收尾任务执行")
        print("  - 部分UAV完成任务后停止")
        print("  - 集群趋于稳定")

print(f"\n\n{'='*80}")
print("分析完成")
print(f"{'='*80}")
