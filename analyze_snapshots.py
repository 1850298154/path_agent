import json
import numpy as np

# 读取UAV位置数据
with open('data/01_original_data/uav_positions_over_time.json', 'r', encoding='utf-8') as f:
    uav_data = json.load(f)

# 读取任务调度数据
with open('data/01_original_data/result_criticalpath_new.json', 'r', encoding='utf-8') as f:
    schedule_data = json.load(f)

# 读取预处理数据（包含任务信息）
with open('data/01_original_data/precomputed_data.json', 'r', encoding='utf-8') as f:
    precomputed = json.load(f)

schedule = schedule_data['task_schedule']

print(f"总时长: {uav_data['makespan']}s")
print(f"UAV数量: {len(uav_data['uavs'])}")
print(f"任务数量: {len(schedule)}")

# 时间点
time_points = [0, 440, 668, 764, 830, 885]

# 分析每个时间点
for t in time_points:
    print(f"\n{'='*60}")
    print(f"时间 t = {t}s")
    print(f"{'='*60}")

    # 1. 找出正在执行的任务
    active_tasks = []
    for task_id, times in schedule.items():
        if times['start'] <= t <= times['end']:
            active_tasks.append({
                'id': int(task_id),
                'start': times['start'],
                'end': times['end'],
                'duration': times['end'] - times['start']
            })
    active_tasks.sort(key=lambda x: x['id'])

    # 2. 找出已完成和待执行的任务
    completed_tasks = []
    pending_tasks = []

    for task_id, times in schedule.items():
        if times['end'] <= t:
            completed_tasks.append({
                'id': int(task_id),
                'end': times['end']
            })
        elif times['start'] > t:
            pending_tasks.append({
                'id': int(task_id),
                'start': times['start']
            })

    completed_tasks.sort(key=lambda x: x['id'])
    pending_tasks.sort(key=lambda x: x['start'])

    print(f"\n任务状态统计:")
    print(f"  已完成: {len(completed_tasks)}个")
    print(f"  正在执行: {len(active_tasks)}个")
    print(f"  待执行: {len(pending_tasks)}个")

    # 3. 正在执行的任务详情
    if active_tasks:
        print(f"\n正在执行的任务:")
        for task in active_tasks:
            print(f"  任务{task['id']}: {task['start']:.1f}s-{task['end']:.1f}s "
                  f"(时长{task['duration']:.1f}s, 已执行{t-task['start']:.1f}s, "
                  f"剩余{task['end']-t:.1f}s)")

            # 查找执行该任务的UAV
            # 从precomputed_data中查找任务信息
            if str(task['id']) in precomputed.get('tasks', {}):
                task_info = precomputed['tasks'][str(task['id'])]
                if 'agent' in task_info:
                    print(f"    -> UAV {task_info['agent']}")

    # 4. 最近完成的任务
    if completed_tasks:
        recent_completed = sorted(completed_tasks, key=lambda x: x['end'], reverse=True)[:3]
        print(f"\n最近完成的任务:")
        for task in recent_completed:
            print(f"  任务{task['id']}: 完成于 {task['end']:.1f}s (距今{t-task['end']:.1f}s)")

    # 5. 即将开始的任务
    if pending_tasks:
        upcoming_tasks = sorted(pending_tasks, key=lambda x: x['start'])[:3]
        print(f"\n即将开始的任务:")
        for task in upcoming_tasks:
            print(f"  任务{task['id']}: 将于 {task['start']:.1f}s 开始 (还有{task['start']-t:.1f}s)")

    # 6. UAV位置分析
    print(f"\nUAV位置分析:")
    # 分析UAV分布
    positions_at_t = []
    for uav_id, uav_info in uav_data['uavs'].items():
        pos_idx = min(int(t), len(uav_info['positions'])-1)
        pos = uav_info['positions'][pos_idx]
        positions_at_t.append({
            'id': int(uav_id),
            'pos': pos,
            'moving': pos_idx > 0 and uav_info['positions'][pos_idx] != uav_info['positions'][pos_idx-1]
        })

    # 统计移动和静止的UAV
    moving_count = sum(1 for uav in positions_at_t if uav['moving'])
    static_count = len(positions_at_t) - moving_count
    print(f"  移动中UAV: {moving_count}个")
    print(f"  静止UAV: {static_count}个")

    # 计算空间分布（战术密度）
    if positions_at_t:
        xs = [p['pos'][0] for p in positions_at_t]
        ys = [p['pos'][1] for p in positions_at_t]
        x_range = max(xs) - min(xs)
        y_range = max(ys) - min(ys)
        print(f"  空间分布: X范围[{min(xs):.1f}, {max(xs):.1f}] "
              f"Y范围[{min(ys):.1f}, {max(ys):.1f}]")
        print(f"  战术密度: {len(positions_at_t)/(x_range*y_range+0.01):.3f} UAV/单位面积")

print(f"\n\n{'='*60}")
print("分析完成")
print(f"{'='*60}")
