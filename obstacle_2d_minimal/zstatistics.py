# 2D/zstatistics.py
"""
            《zstatistics.py 文件为三个进程服务》
zyaml.py 
    XX无服务

zpytest.py 
    对每一轮测试都会总结四个指标：
    统计n轮测试
    一个操作： _______________________________
        程序任务：
            1. n轮结果统计画bar3D图。
        ￣￣￣￣￣￣￣￣￣￣￣￣￣￣￣￣￣￣￣￣
app.py
    计算并存储统计的指标
    一个操作： _______________________________
        程序结束：
            1. 将四个指标结果输出到到自己测试路径下。
        ￣￣￣￣￣￣￣￣￣￣￣￣￣￣￣￣￣￣￣￣
        
zstatistics.py
    统计n轮测试
    一个操作： _______________________________
        程序任务：
            1. n轮结果统计画bar3D图。
        ￣￣￣￣￣￣￣￣￣￣￣￣￣￣￣￣￣￣￣￣
        

"""

import datetime
import numpy as np
import output_filename as of
import zyaml as zy
# import SET


############################################################
############     app.py 结束调用，统计agent_list    #########
############################################################
# # 测试时，模拟uav.py中的 class uav
# class Agent:
#     def __init__(self, deadlock=False, p=None, target=None, pre_traj_list=None, plan_time_list=None):
#         self.deadlock = deadlock
#         self.p = p
#         self.target = target
#         self.pre_traj_list = pre_traj_list
#         self.plan_time_list = plan_time_list


# def calculate_deadlock_rate(agent_list):
#     # 死锁率 (     agent_list有100个agent,是否 agent.deadlock == True)
#     deadlock_count = sum(agent.deadlock for agent in agent_list)
#     deadlock_rate = deadlock_count / len(agent_list)
#     return deadlock_rate
def calculate_deadlock_rate(agent_list):
    deadlock_count = 0
    number_list = []
    deadlock_info_list = []
    for i, agent in enumerate(agent_list):
        if agent.deadlock:
            deadlock_count += 1
            # print("第{}个 agent 发生了死锁".format(i+1))
            number_list.append(i)
            deadlock_info_list.append((i, agent.deadlock_info))
    print('The agent meet a deadlock,  as follows')
    print('deadlock_list  ', number_list)
    print('deadlock_info  ', deadlock_info_list)
    print('deadlock__len  ', len(number_list))
    deadlock_rate = deadlock_count / len(agent_list)
    print('deadlock_rate  ', deadlock_rate)
    print()
    return deadlock_rate


# def calculate_success_rate(agent_list):
#     # 成功率 (     agent_list有100个agent, agent.p = [7.85769868, 7.85769868, 7.85769868] 到达终点 agent.target = [7.85769868, 7.85769868, 7.85769868]  ) *
#     # success_count = sum(np.all(agent.p == agent.target)
#     success_count = sum(np.allclose(agent.p, agent.target, atol=1e-5)
#                         for agent in agent_list)
#     success_rate = success_count / len(agent_list)
#     return success_rate
def calculate_success_rate(agent_list):
    success_count = 0
    number_list = []
    info_list = []

    all_dis = []
    for i, agent in enumerate(agent_list):
        # if np.allclose(agent.p, agent.target, atol=1e-5):
        # if np.allclose(agent.p, agent.target, atol=0.12):
        # if np.allclose(agent.p, agent.target, atol=0.05):
        # if np.linalg.norm(agent.p- agent.target) < agent.r_min: #agent直径
        # if np.linalg.norm(agent.p- agent.target) < agent.physical_radius: #agent直径
        dis = np.linalg.norm(agent.p- agent.target) 
        all_dis.append((i, agent.p.tolist(), agent.target.tolist(), dis))
        # if np.linalg.norm(agent.p- agent.target) < zy.parameters['radius']: #agent直径
        if dis < zy.parameters['radius']: #agent直径
            success_count += 1
            # print("第{}个 agent 成功到达目标位置".format(i+1))
        else:
            number_list.append(i)
            info_list.append((i, np.linalg.norm(agent.p - agent.target)))
    print('计算成功率          ')
    # print('(agent编号, 当前位置, 终点位置, 距离) : ', np.array(all_dis).tolist())
    print('(agent编号, 当前位置, 终点位置, 距离) : ', all_dis)
    print('计算成功率          ')
    print("The agent who didn't reach the end of the line,  as follows")
    print('NotReach_list  ', number_list)
    print('NotReach__len  ', len(number_list))
    print('info_list      ', info_list)
    print('info__len      ', len(info_list))
    success_rate = success_count / len(agent_list)
    print('success_rate   ', success_rate)
    print()
    return success_rate

def deduplicate_keep_order(my_list):
    # my_list = [1, 2, 3, 4, 2, 3, 5, 1]
    unique_list = []
    for element in my_list:
        if element not in unique_list:
            unique_list.append(element)

    # print(unique_list)
    return unique_list





# def calculate_external_collision_rate(agent_list,obstacles,extend_obstacles):
def calculate_external_collision_rate(agent_list,obstacles):
    import zrand as zr
    # 碰撞率 (    agent_list有100个agent，每个agent有200个运动轨迹 agent.pre_traj_list = [array([[ 7.85769868, 7.85769868, 11.2614501 ],...],...,array([[ 7.85769868, 7.85769868, 11.2614501 ],...]) 。 agent同一时刻下，之间距离小于 agent.r_min 时候，认为是碰撞。)
    n = len(agent_list)
    number_handshakes = n
    # number_handshakes = n * (n-1) / 2
    # if number_handshakes == 0:
    #     number_handshakes = 1
    collision_count = 0
    number_list = []
    info2_list = []
    phy_collision_count = 0
    phy_number_list = []
    phy_info2_list = []

    all_dis = []
    for i in range(len(agent_list)):
        agent1_traj_list = agent_list[i].pre_traj_list
        # for j in range(i+1, len(agent_list)):
        #     agent2_traj_list = agent_list[j].pre_traj_list
        for step in range(len(agent1_traj_list)):
            agent1_step_p = agent1_traj_list[step][0]
            # agent2_step_p = agent2_traj_list[step][0]
            is_ex_collision,ob_i,exp_smin = zr.tri_check_agents_external_collision(
                agent1_step_p[0],
                agent1_step_p[1],
                obstacles, # 就是随机产生障碍物，没有膨胀了， 下面一个参数是加入膨胀的半径
                # extend_obstacles, # 膨胀后的障碍物 
                # zr.radius,
                zy.parameters['radius'],
            )

            all_dis.append((i,step,agent1_step_p.tolist(),ob_i,exp_smin))
            if is_ex_collision:
                is_phy_ex_collision,ob_j,smin = zr.tri_check_agents_external_collision(
                    agent1_step_p[0],
                    agent1_step_p[1],
                    obstacles, # 就是随机产生障碍物，没有膨胀了， 下面一个参数是加入膨胀的半径
                    # 0,
                    zy.parameters['physical_radius'],
                )        
                if is_phy_ex_collision:
                    phy_collision_count+=1
                    phy_number_list.append(i)
                    # phy_number_list.append((i, step))
                    phy_info2_list.append((i, step, smin, agent1_step_p, obstacles[ob_j])) # 可以记录障第几个碍物
                    # phy_info2_list.append((i, step, ob_num)) # 可以记录障第几个碍物
                collision_count += 1
                # print(" agent {} 和 {}， 在第{}步发生碰撞".format(i, j, step))
                number_list.append(i)
                # number_list.append((i, step))
                info2_list.append((i, step, exp_smin, agent1_step_p, obstacles[ob_i])) # 可以记录障第几个碍物                            
                # info2_list.append((i, step, ob_num)) # 可以记录障第几个碍物                            

    number_list = deduplicate_keep_order(number_list)
    phy_number_list = deduplicate_keep_order(phy_number_list)
    
    print('计算外部碰撞率        ')
    print('障碍物扩展安全宽度为 ' , zy.parameters['radius'])
    print('障碍物扩展物理宽度为 ' , zy.parameters['physical_radius'])
    print('(agent编号, 第几步, i当前位置, 碰撞的障碍物编号, 碰撞时嵌入的深度) : ', all_dis)
    print('障碍物扩展安全宽度为 ' , zy.parameters['radius'])
    print('障碍物扩展物理宽度为 ' , zy.parameters['physical_radius'])
    print('计算外部碰撞率        ')
    print('ex_collision_list      ', number_list)
    print('ex_collision__len      ', len(number_list))
    print('ex_info2_list          ', info2_list)
    print('ex_info2__len          ', len(info2_list))
    ex_collision_rate = len(number_list) / number_handshakes
    # collision_rate = collision_count / number_handshakes
    print('ex_collision_rate2     ', ex_collision_rate)
    # if len(info2_list) ==0 :
    #     min_value=-1
    # else:
    #     min_value = min(row[3] for row in info2_list)
    # # min_value = min(row[3] for row in info2_list)
    # print('min_value2          ', min_value)
    print('---------------------------------------')
    print('phy_ex_collision_list  ', phy_number_list)
    print('phy_ex_collision__len  ', len(phy_number_list))
    print('phy_ex_info2_list      ', phy_info2_list)
    print('phy_ex_info2__len      ', len(phy_info2_list))
    phy_ex_collision_rate = len(phy_number_list) / number_handshakes
    # phy_collision_rate = phy_collision_count / number_handshakes
    print('phy_collision_rate2 ', phy_ex_collision_rate)
    print()
    return phy_ex_collision_rate
    return ex_collision_rate



# def calculate_collision_rate(agent_list):
#     # 碰撞率 (    agent_list有100个agent，每个agent有200个运动轨迹 agent.pre_traj_list = [array([[ 7.85769868, 7.85769868, 11.2614501 ],...],...,array([[ 7.85769868, 7.85769868, 11.2614501 ],...]) 。 agent同一时刻下，之间距离小于 agent.r_min 时候，认为是碰撞。)
#     collision_count = 0
#     for agent in agent_list:
#         traj_list = agent.pre_traj_list
#         for i in range(len(traj_list)):
#             for j in range(i+1, len(traj_list)):
#                 distance = np.linalg.norm(traj_list[i] - traj_list[j])
#                 if distance < agent.r_min:
#                     collision_count += 1
#                     break
#     collision_rate = (collision_count / (len(agent_list) * len(agent.pre_traj_list)))
#     return collision_rate
def calculate_collision_rate(agent_list):
    # 碰撞率 (    agent_list有100个agent，每个agent有200个运动轨迹 agent.pre_traj_list = [array([[ 7.85769868, 7.85769868, 11.2614501 ],...],...,array([[ 7.85769868, 7.85769868, 11.2614501 ],...]) 。 agent同一时刻下，之间距离小于 agent.r_min 时候，认为是碰撞。)
    n = len(agent_list)
    # number_handshakes = n * (n-1) / 2
    # if number_handshakes == 0:
    #     number_handshakes = 1
    number_handshakes = n
    collision_count = 0
    number_list = []
    info2_list = []
    phy_collision_count = 0
    phy_number_list = []
    phy_info2_list = []

    all_dis = []
    for i in range(len(agent_list)):
        agent1_traj_list = agent_list[i].pre_traj_list
        for j in range(i+1, len(agent_list)):
            agent2_traj_list = agent_list[j].pre_traj_list
            for step in range(len(agent1_traj_list)):
                agent1_step_p = agent1_traj_list[step][0]
                agent2_step_p = agent2_traj_list[step][0]
                distance = np.linalg.norm(agent1_step_p
                                          - agent2_step_p)
                all_dis.append((i,j,step,agent1_step_p.tolist(),agent2_step_p.tolist(),distance))
                if distance < agent_list[i].r_min:
                    if distance < agent_list[i].physical_r_min:
                        phy_collision_count+=1
                        phy_number_list.append((i, j, step))
                        phy_info2_list.append((i, j, step, distance))
                    collision_count += 1
                    # print(" agent {} 和 {}， 在第{}步发生碰撞".format(i, j, step))
                    number_list.append((i, j, step))
                    info2_list.append((i, j, step, distance))
                    break
    print('计算内部碰撞率        ')
    # print('(agent编号i, agent编号j, 第几步, i当前位置, j当前位置, 距离) : ', np.array(all_dis).tolist())
    # print('(agent编号i, agent编号j, 第几步, i当前位置, j当前位置, 距离) : ', all_dis)
    print('计算内部碰撞率        ')
    print('collision_list      ', number_list)
    print('collision__len      ', len(number_list))
    print('info2_list          ', info2_list)
    print('info2__len          ', len(info2_list))
    collision_rate = collision_count / number_handshakes
    print('collision_rate2     ', collision_rate)
    if len(info2_list) ==0 :
        min_value=-1
    else:
        min_value = min(row[3] for row in info2_list)
    # min_value = min(row[3] for row in info2_list)
    print('min_value2          ', min_value)
    print('---------------------------------------')
    print('phy_collision_list  ', phy_number_list)
    print('phy_collision__len  ', len(phy_number_list))
    print('phy_info2_list      ', phy_info2_list)
    print('phy_info2__len      ', len(phy_info2_list))
    phy_collision_rate = phy_collision_count / number_handshakes
    print('phy_collision_rate2 ', phy_collision_rate)
    print()
    return phy_collision_rate
    return collision_rate


number_list = []
info2_list = []
import random
lim=0.099999


def fallback_calculate_collision_rate(agent_list):
    global number_list
    global info2_list
    print('fallback_calculate_collision_rate')
    print('collision_list  ', number_list)
    print('collision__len  ', len(number_list))
    print('info3_list      ', info2_list)
    print('info3__len      ', len(info2_list))
    n = len(agent_list)
    number_handshakes=n
    # if n==1:
    #     number_handshakes= 1
    # else:      
    #     number_handshakes = n * (n-1) / 2
    collision_count = len(number_list)
    collision_rate = collision_count / number_handshakes
    print('collision_rate3 ', collision_rate)
    if len(info2_list) ==0 :
        min_value=-1
    else:
        min_value = min(row[3] for row in info2_list)
    print('min_value3      ', min_value)
    print()

    return collision_rate

# def convert_to_time_string(time_delta):
#     hours, remainder = divmod(time_delta.seconds, 3600)
#     minutes, seconds = divmod(remainder, 60)
#     milliseconds = time_delta.microseconds // 1000
#     time_string = f"{hours:02d}:{minutes:02d}:{seconds:02d}.{milliseconds:03d}"
#     return time_string

def find_min_in_nested_list(nested_list):
    return min(min(inner_list) for inner_list in nested_list)

def calculate_average_planning_time(agent_list):
    # 规划时间 ( agent_list有100个agent，每个agent有200个规划时间，agent.plan_time_list=[0:00:04.678481 ... 0:00:05.678481] 统计)
    total_planning_time = sum(time_total_seconds_float
                              for agent in agent_list
                              for time_total_seconds_float in agent.plan_time_list)
    agents_steps = (len(agent_list) * len(agent_list[0].plan_time_list))
    if np.isclose(agents_steps, 0, atol=1e-8):
        average_planning_time = datetime.timedelta(0)
    else:
        average_planning_time = datetime.timedelta(
            seconds=(total_planning_time / agents_steps))
    average_planning_time_total_seconds_float = float(
        average_planning_time.total_seconds())
    print('average_planning_time_total_seconds_float')
    print(average_planning_time_total_seconds_float)
    print()

    min_value = min(
            min(agent.plan_time_list) 
            for agent in agent_list 
            if agent.plan_time_list
        )
    max_value = max(
            max(
                i
                for i in agent.plan_time_list
                if i < lim
            ) 
            for agent in agent_list 
            if agent.plan_time_list
        )    
    return average_planning_time_total_seconds_float, min_value, max_value


def calculate_deadlock_crack_rate(agent_list):
# def calculate_external_collision_rate(agent_list,obstacles):
    import zrand as zr
    # 碰撞率 (    agent_list有100个agent，每个agent有200个运动轨迹 agent.pre_traj_list = [array([[ 7.85769868, 7.85769868, 11.2614501 ],...],...,array([[ 7.85769868, 7.85769868, 11.2614501 ],...]) 。 agent同一时刻下，之间距离小于 agent.r_min 时候，认为是碰撞。)
    n = len(agent_list)
    number_handshakes = n
    deadlock_warning__cnt = 1  
    deadlock_cracking_cnt = 1
    ex_deadlock_warning__cnt = 1  
    ex_deadlock_cracking_cnt = 1
    # number_handshakes = n * (n-1) / 2
    # if number_handshakes == 0:
    #     number_handshakes = 1
    collision_count = 0
    number_list = []
    info2_list = []
    phy_collision_count = 0
    phy_number_list = []
    phy_info2_list = []

    

    all_dis = []
    for i in range(len(agent_list)):
        import uav
        agent:uav.uav2D = agent_list[i]
        agent1_traj_list = agent.pre_traj_list
        # for j in range(i+1, len(agent_list)):
        #     agent2_traj_list = agent_list[j].pre_traj_list
        last_step_i = -1
        first_pre_i = 0

        last_position_xylist = agent1_traj_list[last_step_i][first_pre_i]
        keep_times_n = 3
        keep_times_i = 0
        step_pos_2_target_distance_close_flag = False
        for step in range(len(agent1_traj_list)):
            agent1_step_pre_list = agent1_traj_list[step] # 不可以增加 [0] 后面 再取 使用哪个
            # for k, p_in_pre_list in enumerate(agent1_step_pre_list):
            #     p_in_pre_list
            k0_p = agent1_step_pre_list[0]
            kk_p = agent1_step_pre_list[-1]

            agent1_step_p = k0_p
            agent2_step_p = kk_p
            k0k__distance = np.linalg.norm(agent1_step_p
                                        - agent2_step_p)
            step_pos_2_last_pos_distance = np.linalg.norm(agent1_step_p
                                        - agent.p)
            # end_pos_2_finish_line_distance = np.linalg.norm(agent1_step_p
            #                             - last_position_xylist)
            
            all_dis.append((i, step, keep_times_i, agent1_step_p.tolist(), agent2_step_p.tolist(), k0k__distance))


            # 只要step位置已接近终点target， 就不用统计死锁了
            step_pos_2_target_distance = np.linalg.norm(agent1_step_p
                                        - agent.target)
            if step_pos_2_target_distance < agent_list[i].r_min:
                step_pos_2_target_distance_close_flag = True
            if step_pos_2_target_distance_close_flag == True:
                continue
                

            if k0k__distance < agent_list[i].r_min:
            # if k0k__distance < 0.1:
                keep_times_i += 1
                if keep_times_i < keep_times_n:
                    continue
                keep_times_i = keep_times_n
                if k0k__distance < agent_list[i].physical_r_min:
                # if k0k__distance < 0.01:
                    phy_collision_count+=1
                    phy_number_list.append((i, step))
                    phy_info2_list.append((i, step, keep_times_i, k0k__distance))
                
                
                collision_count += 1
                # print(" agent {} 和 {}， 在第{}步发生碰撞".format(i, j, step))
                number_list.append((i, step))
                info2_list.append((i, step, keep_times_i, k0k__distance))
                deadlock_warning__cnt += 1

                # 只要移动就是破解
                # if step_pos_2_last_pos_distance >= agent_list[i].r_min:
                if step_pos_2_last_pos_distance >= 0.001:
                    deadlock_cracking_cnt += 1
            else:
                keep_times_i = 0
                
    print('计算死锁破解率        ')
    print('keep_times_n                 ' , keep_times_n)
    print('agent_list[i].r_min          ' , agent_list[i].r_min)
    print('agent_list[i].physical_r_min ' , agent_list[i].physical_r_min)
    print('障碍物扩展安全宽度为 ' , zy.parameters['radius'])
    print('障碍物扩展物理宽度为 ' , zy.parameters['physical_radius'])
    print('(agent编号, 第几步, 重复轮数, k0位置, kk位置, 距离) : ', all_dis)

    print('障碍物扩展安全宽度为 ' , zy.parameters['radius'])
    print('障碍物扩展物理宽度为 ' , zy.parameters['physical_radius'])
    print('计算死锁破解率        ')
    print('deadlock_crack_rate_list      ', number_list)
    print('deadlock_crack_rate__len      ', len(number_list))
    print('deadlock_crack_rate_list          ', info2_list)
    print('deadlock_crack_rate__len          ', len(info2_list))
    deadlock_crack_rate =   deadlock_cracking_cnt / deadlock_warning__cnt
    print('deadlock_crack_rate     ', deadlock_crack_rate)
    print('---------------------------------------')
    print('phy_deadlock_crack_rate_list  ', phy_number_list)
    print('phy_deadlock_crack_rate__len  ', len(phy_number_list))
    print('phy_deadlock_crack_rate_list      ', phy_info2_list)
    print('phy_deadlock_crack_rate__len      ', len(phy_info2_list))
    phy_deadlock_crack_rate = deadlock_cracking_cnt / deadlock_warning__cnt
    print('phy_deadlock_crack_rate ', phy_deadlock_crack_rate)
    print()
    return deadlock_crack_rate

class cstatistics:
    # initialization
    def __init__(self, 
                # deadlock_rate, 
                average_planning_time, 
                success_rate,
                collision_rate, 
                ex_collision_rate,
                deadlock_crack_rate,
                min_value, 
                max_value,
                # number_baseline_deadlocks,
                # number_successfully_crack_deadlocks,
                deadlocks_occur,
            ):
        # self.deadlock_rate = deadlock_rate
        self.success_rate = success_rate
        self.collision_rate = collision_rate
        self.average_planning_time = average_planning_time
        self.ex_collision_rate = ex_collision_rate
        self.deadlock_crack_rate = deadlock_crack_rate
        self.planning_success_rate = deadlock_crack_rate
        self.min_planning_time = min_value
        self.max_planning_time = max_value
        self.safety_rate = deadlock_crack_rate
        # self.number_basxeline_deadlocks = number_baseline_deadlocks
        # self.number_successfully_crack_deadlocks = number_successfully_crack_deadlocks
        self.deadlocks_occur = deadlocks_occur

# app.py 结束时调用，输出统计到终端，随着tee存入print.txt
def fstatistics(agent_list):
    average_planning_time, min_value, max_value = calculate_average_planning_time(agent_list)
    deadlock_rate = calculate_deadlock_rate(agent_list)
    success_rate = calculate_success_rate(agent_list)
    collision_rate = calculate_collision_rate(agent_list)
    fallback_collision_rate = fallback_calculate_collision_rate(agent_list)
    import zrand as zr
    # zr.extend_obstacles=zr.make_extend_obstacles(zr.obstacles, zr.radius)
    # print('zr.obstacles')
    # print(zr.obstacles)
    # print('zr.extend_obstacles')
    # print(zr.extend_obstacles)
    ex_collision_rate=calculate_external_collision_rate(agent_list,zr.obstacles)
    
    
    deadlock_crack_rate=calculate_deadlock_crack_rate(agent_list)
    

    # print("warning   rate     : ", deadlock_rate)
    print("success   rate     : ", success_rate)
    print("Collision rate     : ", collision_rate)
    # print("fallback  rate     : ", fallback_collision_rate)
    print("planning  time     : ", average_planning_time)
    print("exter_collision_rate  : ", ex_collision_rate)
    print("deadlock_crack_rate  : ", deadlock_crack_rate)
    print(flush=True)

    from dead_record import pair_timeline, dead_timeline, g_dead_list, g_p_list

    number_baseline_deadlocks = len(g_dead_list)
    deadlocks_occur = 1 if len(g_dead_list) > 0 else 0
    if success_rate > 0.999:
        number_successfully_crack_deadlocks = len(g_p_list) 
    else:
        number_successfully_crack_deadlocks = len(g_p_list) 

    cs = cstatistics(
            # deadlock_rate, 
            average_planning_time, 
            success_rate,
            collision_rate, 
            ex_collision_rate,
            deadlock_crack_rate,
            min_value,
            max_value,
            # number_baseline_deadlocks,
            # number_successfully_crack_deadlocks ,
            deadlocks_occur
        )
    of.saveJSON(cs, 'a_statistics')


# # Sample usage : def fstatistics(agent_list):
# def Local_File_Test(how_many_agents):
#     agent_list = []
#     # for _ in range(100):
#     for _ in range(how_many_agents):
#         agent = Agent()
#         agent.deadlock = np.random.choice([True, False])
#         agent.p = np.array([7.85769868, 7.85769868, 7.85769868])
#         agent.target = np.array([7.85769868, 7.85769868, 8.85769869])
#         # agent.target = np.array([7.85769868, 7.85769868, 7.85769868])
#         # agent.target = np.array([[7.85769869, 7.85769868, 12.3456789]])
#         agent.r_min = 0.1
#         agent.pre_traj_list = [
#             np.array([[7.85769868, 7.85769868, 11.2614501]]),
#             # np.array([[7.85769868, 7.85769868, 12.3456789]])
#             # np.array([[7.85769869, 7.85769868, 12.3456789]])
#             np.array([[8.85769869, 7.85769868, 12.3456789]])
#         ]
#         agent.plan_time_list = [
#             float(datetime.timedelta(seconds=4.678481).total_seconds()),
#             float(datetime.timedelta(seconds=5.678481).total_seconds()),
#             # datetime.timedelta(seconds=5.678481),
#         ]
#         agent_list.append(agent)

#     # print('************************************')
#     # variables = agent_list[0].__dict__
#     # for key, value in variables.items():
#     #     print(f"{key}: {value}\n")
#     # print('************************************')

#     fstatistics(agent_list=agent_list)
#     # import sys
#     # sys.exit(-1)  # My debugging exits
#     # deadlock_rate = calculate_deadlock_rate(agent_list)
#     # success_rate = calculate_success_rate(agent_list)
#     # collision_rate = calculate_collision_rate(agent_list)
#     # average_planning_time = calculate_average_planning_time(agent_list)

#     # print("死锁率:", deadlock_rate)
#     # print("成功率:", success_rate)
#     # print("碰撞率:", collision_rate)
#     # print("规划时间:", average_planning_time)


############################################################
#############     002 加载yaml后求均值最值，无用    #########
############################################################
def analyze_data(data_list, config_list):
    deadlock_rates = []
    success_rates = []
    collision_rates = []
    avg_planning_times = []

    for data in data_list:
        deadlock_rate = data.get('deadlock_rate', 0.0)
        success_rate = data.get('success_rate', 0.0)
        collision_rate = data.get('collision_rate', 0.0)
        avg_planning_time = data.get('average_planning_time', 0.0)

        deadlock_rates.append(deadlock_rate)
        success_rates.append(success_rate)
        collision_rates.append(collision_rate)
        avg_planning_times.append(avg_planning_time)

    # 在这里进行统计分析，例如计算平均值、最大值、最小值等等
    # 这里只是示例，您可以根据需要进行相应的数据操作和分析

    print(f"Deadlock Rates: {deadlock_rates}")
    print(f"Success Rates: {success_rates}")
    print(f"Collision Rates: {collision_rates}")
    print(f"Average Planning Times: {avg_planning_times}")


def pd_analyse(data_list):
    import pandas as pd

    # data_list = [
    #     {'Vmax': 3, 'avoid.m': 1, 'deadlock_rate': 0.0, 'success_rate': 0.3,
    #         'collision_rate': 0.0, 'average_planning_time': 0.038123},
    #     {'Vmax': 4, 'avoid.m': 2, 'deadlock_rate': 0.1, 'success_rate': 0.5,
    #         'collision_rate': 0.2, 'average_planning_time': 0.041234},
    #     {'Vmax': 2, 'avoid.m': 0, 'deadlock_rate': 0.0, 'success_rate': 0.2,
    #         'collision_rate': 0.1, 'average_planning_time': 0.036789}
    # ]

    df = pd.DataFrame(data_list)
    # 求success_rate最大值对应的Vmax和avoid.m
    # max_success_rate = df['success_rate'].max()
    max_success_rate = df['success_rate'].min()
    max_success_rate_row = df[df['success_rate'] == max_success_rate]
    max_success_rate_Vmax = max_success_rate_row['Vmax'].values[0]
    max_success_rate_avoid_m = max_success_rate_row['avoid.m'].values[0]
    print('min_success_rate', max_success_rate)

    # 求collision_rate最大值对应的Vmax和avoid.m
    max_collision_rate = df['collision_rate'].max()
    max_collision_rate_row = df[df['collision_rate'] == max_collision_rate]
    max_collision_rate_Vmax = max_collision_rate_row['Vmax'].values[0]
    max_collision_rate_avoid_m = max_collision_rate_row['avoid.m'].values[0]
    print('max_collision_rate', max_collision_rate)

    # 求deadlock_rate最大值对应的Vmax和avoid.m
    max_deadlock_rate = df['deadlock_rate'].max()
    max_deadlock_rate_row = df[df['deadlock_rate'] == max_deadlock_rate]
    max_deadlock_rate_Vmax = max_deadlock_rate_row['Vmax'].values[0]
    max_deadlock_rate_avoid_m = max_deadlock_rate_row['avoid.m'].values[0]
    print('max_deadlock_rate', max_deadlock_rate)

    # 求average_planning_time最大值对应的Vmax和avoid.m
    max_average_planning_time = df['average_planning_time'].max()
    max_average_planning_time_row = df[df['average_planning_time']
                                       == max_average_planning_time]
    max_average_planning_time_Vmax = max_average_planning_time_row['Vmax'].values[0]
    max_average_planning_time_avoid_m = max_average_planning_time_row['avoid.m'].values[0]
    print('max_average_planning_time', max_average_planning_time)


############################################################
###############     002 加载测试结果yaml     ################
############################################################

def load_yaml_files(start_date, end_date):

    import os

    # folder_path = './002/'  # 替换为你的目录路径
    folder_path = of.test_plan+'/'  # 替换为你的目录路径
    # start_date = '2023-08-07_17-52-14'
    # end_date = '2023-08-27_17-52-14'

    folders = []
    for folder_name in os.listdir(folder_path):
        if os.path.isdir(os.path.join(folder_path, folder_name)):
            date = folder_name[:19]  # 提取日期部分，长度为19
            # if folder_name.startswith('2023-08-17_17-52-14'):
            if start_date <= date <= end_date:
                folders.append(folder_path+folder_name)

    # print(*folders, sep='\n')
    # print(len(folders))

    json_yaml_path_list = [
        folder + '/agent100/a_statistics.json'
        for folder in folders]
    import yaml
    data_list = []
    for yaml_path in json_yaml_path_list:
        try:

            with open(yaml_path, 'r') as file:
                data = yaml.safe_load(file)
                data_list.append(data)
        except FileNotFoundError as e:
            print('***********************************')
            print("File not found : ", e)
            print(
                'There are no data files under this folder, which may be a useless folder')
            print('***********************************')

    config_list = []
    import glob
    # cnt = 1
    for folder in folders:
        file_pattern = folder+'/'+'*.yaml'
        file_list = glob.glob(file_pattern)
        # print(cnt)
        # cnt += 1
        # print(file_list)
        # print(len(file_list))
        for file_path in file_list:
            with open(file_path, 'r') as file:
                data = yaml.safe_load(file)
                config_list.append(data)

    return data_list, config_list


############################################################
###############     002 加载yaml后画bar3d    ################
############################################################
# 减少对第三方库的使用， 不用 pandas
def pd_plot3D_bar(merge_data_list):
    import pandas as pd
    import matplotlib.pyplot as plt

    # merge_data_list = [
    #     {'Vmax': 3, 'avoid.m': 1, 'deadlock_rate': 0.0, 'success_rate': 0.3,
    #         'collision_rate': 0.0, 'average_planning_time': 0.038123},
    #     {'Vmax': 4, 'avoid.m': 2, 'deadlock_rate': 0.1, 'success_rate': 0.5,
    #         'collision_rate': 0.2, 'average_planning_time': 0.041234},
    #     {'Vmax': 2, 'avoid.m': 0, 'deadlock_rate': 0.0, 'success_rate': 0.2,
    #         'collision_rate': 0.1, 'average_planning_time': 0.036789}
    # ]

    df = pd.DataFrame(merge_data_list)

    rates = ['deadlock_rate', 'success_rate',
             'collision_rate', 'average_planning_time']

    fig = plt.figure()

    for i, rate in enumerate(rates):
        ax = fig.add_subplot(2, 2, i+1, projection='3d')
        x_values = df['Vmax']
        y_values = df['avoid.m']
        z_values = df[rate]

        for x, y, z in zip(x_values, y_values, z_values):
            ax.bar3d(x, y, 0, 0.5, 0.5, z, alpha=0.5)
            # ax.text(x, y, z + 0.05, "{:.2f}".format(z),
            ax.text(x, y, z * 1.15, "{:.2f}".format(z),
                    ha='center', va='center')

        ax.set_xlabel('Vmax')
        ax.set_ylabel('avoid.m')
        ax.set_zlabel(rate)

    plt.tight_layout()
    plt.show()


def plot3D_bar(merge_data_list):
    import matplotlib.pyplot as plt

    # merge_data_list = [
    #     {'Vmax': 3, 'avoid.m': 1, 'deadlock_rate': 0.0, 'success_rate': 0.3,
    #         'collision_rate': 0.0, 'average_planning_time': 0.038123},
    #     {'Vmax': 4, 'avoid.m': 2, 'deadlock_rate': 0.1, 'success_rate': 0.5,
    #         'collision_rate': 0.2, 'average_planning_time': 0.041234},
    #     {'Vmax': 2, 'avoid.m': 0, 'deadlock_rate': 0.0, 'success_rate': 0.2,
    #         'collision_rate': 0.1, 'average_planning_time': 0.036789}
    # ]

    rates = ['deadlock_rate', 'success_rate',
             'collision_rate', 'average_planning_time']

    fig = plt.figure()

    for i, rate in enumerate(rates):
        ax = fig.add_subplot(2, 2, i+1, projection='3d')
        x_values = [data['Vmax'] for data in merge_data_list]
        y_values = [data['avoid.m'] for data in merge_data_list]
        y_values_2 = [data['Umax'] for data in merge_data_list]
        z_values = [data[rate] for data in merge_data_list]

        # for x, y, z in zip(x_values, y_values, z_values):
        for x, y, y2, z in zip(x_values, y_values, y_values_2, z_values):
            ax.bar3d(x, y, 0, 0.5, 0.5, z, alpha=0.3)
            # ax.text(x, y, z + 0.05, "{:.2f}".format(z),
            # ax.text(x, y, z * 1.3, "{:.3f}".format(z),
            ax.text(x, y, z * 1.3, f"{y2}u {z:.3f}",
                    ha='center', va='center')

        ax.set_xlabel('Vmax')
        ax.set_ylabel('avoid.m')
        # ax.set_ylabel('Umax')
        ax.set_zlabel(rate)

    plt.tight_layout()
    plt.show()


# 调用 app.py 后，zpytest.py 会调用。
# 也可以在本文件 zstatistic.py 自己调用会看结果。 见 if __name__ == __main__
def statistics_n_round_test(start_date, end_date):
    data_list, config_list = load_yaml_files(start_date, end_date)
    if len(data_list) == len(config_list):
        # analyze_data(data_list, config_list)
        new_data_list = []
        for i in range(len(data_list)):
            merged_dict = config_list[i].copy()
            merged_dict.update(data_list[i])
            new_data_list.append(merged_dict)
        # pd_analyse(new_data_list)
        plot3D_bar(new_data_list)
        # pd_plot3D_bar(new_data_list)
    else:
        print(' len(data_list) ! = len(config_list) ')


def test_statistics_n_round_test():

    # start_date = '2023-08-17_17-47-29'
    # end_date = '2023-08-17_17-53-08'
    # start_date = '2023-08-18_10-36-44'
    # end_date = '2023-08-18_10-40-33'
    # start_date = '2023-08-18_17-50-15'
    # end_date = '2023-08-18_18-44-20'
    # << << << < HEAD
    # # start_date = '2023-08-19_13-51-18'
    # # end_date = '2023-08-19_14-06-32'
    # start_date = '2023-08-19_19-56-51'
    # end_date = '2023-08-19_20-02-03'
    # start_date = '2023-08-19_20-25-55'
    # end_date = '2023-08-19_20-43-05'
    # start_date = '2023-08-21_18-26-14'
    # end_date = '2023-08-21_18-30-51'
    # start_date = '2023-08-22_09-19-58'
    # end_date = '2023-08-22_09-25-25'
    start_date = '2023-10-05_14-32-58'
    end_date = '2023-10-05_17-28-59'
    # == == == =
    # start_date = '2023-08-19_11-21-00'
    # end_date = '2023-08-19_16-28-04'  # ubuntu 100

    # >>>>>> > 4c6239055ed3afa0ea80dc776a5163b426fa5d71
    statistics_n_round_test(start_date, end_date)


def ob_bar(merge_data_list):
    import matplotlib.pyplot as plt

    # merge_data_list = [
    #     {'Vmax': 3, 'avoid.m': 1, 'deadlock_rate': 0.0, 'success_rate': 0.3,
    #         'collision_rate': 0.0, 'average_planning_time': 0.038123},
    #     {'Vmax': 4, 'avoid.m': 2, 'deadlock_rate': 0.1, 'success_rate': 0.5,
    #         'collision_rate': 0.2, 'average_planning_time': 0.041234},
    #     {'Vmax': 2, 'avoid.m': 0, 'deadlock_rate': 0.0, 'success_rate': 0.2,
    #         'collision_rate': 0.1, 'average_planning_time': 0.036789}
    # ]

    # rates = ['deadlock_rate', 'success_rate',
    #          'collision_rate', 'average_planning_time']
    rates = ['1 vs 50: 单位s']

    fig = plt.figure()

    for i, rate in enumerate(rates):
        # ax = fig.add_subplot(2, 2, i+1, projection='3d')
        ax = fig.add_subplot(1, 1, i+1, projection='3d')
        x_values = [data['Vmax'] for data in merge_data_list]
        y_values = [data['avoid.m'] for data in merge_data_list]
        y_values_2 = [data['Umax'] for data in merge_data_list]
        z_values = [data[rate] for data in merge_data_list]

        for x, y, z in zip(x_values, y_values, z_values):
            # for x, y, y2, z in zip(x_values, y_values, y_values_2, z_values):
            ax.bar3d(x, y, 0, 0.5, 0.5, z, alpha=0.3)
            # ax.text(x, y, z + 0.05, "{:.2f}".format(z),
            ax.text(x, y, z * 1.3, "{:.3f}".format(z),
                    # ax.text(x, y, z * 1.3, f"{y2}u {z:.3f}",
                    ha='center', va='center')

        ax.set_xlabel('Vmax')
        ax.set_ylabel('avoid.m')
        # ax.set_ylabel('Umax')
        ax.set_zlabel(rate)

    plt.tight_layout()
    plt.show()


# if __name__ == "__main__":
def test():
    # pass
    # 0.9275320084038399
    data = [(0, 4, 128, 1.0760603085951739), (0, 31, 369, 1.1423522628980791), (0, 57, 197, 1.1794603823865137), (0, 65, 106, 1.1966765520217137), (0, 66, 269, 1.08388909146425), (0, 70, 206, 1.0934325181566378), (0, 96, 190, 0.9275320084038399), (11, 50, 134, 1.1955289512699228), (11, 82, 132, 1.1871698524953256), (23, 75, 88, 1.1770145590309162), (25, 50, 188, 1.1820959935916429), (25, 64, 150, 1.1879827234979412), (27, 31, 216, 1.1789936298918098), (27, 37, 223, 1.194423346722189), (29, 75, 111, 1.1705362026482393), (30, 41, 64, 1.1936814465148617), (36, 37, 274, 1.199271670603259),
            (36, 53, 292, 1.1995637614470764), (36, 89, 311, 1.1780370047890685), (36, 99, 278, 1.189827990844204), (37, 66, 150, 1.1298911516832162), (40, 48, 110, 1.191081584889476), (41, 48, 111, 1.1424588419983288), (42, 59, 160, 1.1932407747743325), (42, 77, 85, 1.0900257539344356), (45, 50, 155, 1.1715669168060459), (48, 67, 106, 1.1819218915801528), (48, 96, 253, 1.1664486934058536), (51, 61, 70, 1.199428558590194), (52, 75, 142, 1.1832428460473061), (54, 72, 174, 1.1890097531533463), (57, 89, 64, 1.188257457345271), (57, 90, 207, 1.164380755301601), (89, 96, 73, 1.1937052708234686)]
    data = [(0, 10, 269, 1.1694916884365074), (0, 32, 116, 1.0454092202888463), (0, 52, 142, 1.1717152587055275), (0, 63, 294, 1.1547995370160868), (0, 66, 199, 1.1910340408619675), (0, 67, 182, 1.1546178557088762), (1, 73, 158, 1.1784094866293235), (3, 26, 160, 1.1361665181444842), (3, 32, 180, 1.1380042193942148), (7, 59, 183, 1.1947847005132812), (10, 16, 127, 1.1662281284787572), (10, 42, 148, 1.19567102723501),
            (10, 44, 132, 1.1657597467272827), (10, 52, 72,
                                                1.146992537151276), (10, 66, 139, 1.131697504525607),
            (10, 67, 124, 0.9756726298162546), (10, 87, 119,
                                                1.1993352414483467), (16, 51, 135, 1.1702388570598097),

            (20, 78, 147, 1.1661073678023635), (20, 81, 173, 1.145351551235754), (26, 32, 179, 1.0047059849864697), (26, 70, 181, 1.1188256049734462), (27, 67, 207, 1.194640267402468), (27, 87, 202, 0.981253211578356), (27, 88, 92, 1.1962452572124902), (28, 78, 168, 1.1284318371689468), (28, 81, 206, 1.1539469539246834), (29, 81, 190, 1.1745553413404677), (35, 69, 30, 1.0635667601243555), (51, 66, 142, 1.161325256125031), (72, 78, 174, 1.1535570107344537), (74, 83, 212, 1.164535555830055), (78, 81, 151, 1.1624057566719892), (78, 96, 141, 1.1333544311654145), (78, 99, 119, 1.1165727019826848)]
    # 0.9756726298162546
    data = [(0, 10, 269, 1.1694916884365074), (0, 32, 116, 1.0454092202888463), (0, 52, 142, 1.1717152587055275), (0, 63, 294, 1.1547995370160868), (0, 66, 199, 1.1910340408619675), (0, 67, 182, 1.1546178557088762), (1, 73, 158, 1.1784094866293235), (3, 26, 160, 1.1361665181444842), (3, 32, 180, 1.1380042193942148), (7, 59, 183, 1.1947847005132812), (10, 16, 127, 1.1662281284787572), (10, 42, 148, 1.19567102723501), (10, 44, 132, 1.1657597467272827), (10, 52, 72, 1.146992537151276),
            (10, 66, 139, 1.131697504525607), (10, 67, 124, 0.9756726298162546), (10, 87, 119, 1.1993352414483467), (16, 51, 135, 1.1702388570598097), (20, 78, 147, 1.1661073678023635), (20, 81, 173, 1.145351551235754), (26, 32, 179, 1.0047059849864697), (26, 70, 181, 1.1188256049734462), (27, 67, 207, 1.194640267402468), (27, 87, 202, 0.981253211578356), (27, 88, 92, 1.1962452572124902), (28, 78, 168, 1.1284318371689468), (28, 81, 206, 1.1539469539246834), (29, 81, 190, 1.1745553413404677), (35, 69, 30, 1.0635667601243555), (51, 66, 142, 1.161325256125031), (72, 78, 174, 1.1535570107344537), (74, 83, 212, 1.164535555830055), (78, 81, 151, 1.1624057566719892), (78, 96, 141, 1.1333544311654145), (78, 99, 119, 1.1165727019826848)]

    # 0.984461339673762
    data = [(0, 48, 264, 1.1845491035133968), (1, 15, 96, 1.179860386970681), (1, 46, 81, 1.193144639078862), (1, 77, 54, 1.184438790808682), (1, 79, 140, 1.071497825987907), (1, 81, 79, 1.1197120017132265), (1, 98, 87, 1.0866795516096264), (13, 57, 142, 1.1927653774010212), (13, 71, 137, 1.148816177500322), (28, 48, 210, 1.187789543982482), (28, 79, 216, 1.1864422335681755),
            (29, 48, 218, 1.15793409961186), (30, 57, 141, 1.1870904916146472), (30, 71, 126, 1.1769263384248945), (33, 80, 165, 1.1790394794787264), (38, 46, 70, 1.1978891712567934), (38, 81, 68, 1.153317040171032), (40, 53, 239, 1.1969125955840996), (41, 79, 192, 1.1868974616413341), (48, 79, 215, 0.984461339673762), (57, 71, 139, 1.0917268990289437), (85, 95, 76, 1.1934091282037433)]

    fdata = [(95, 17, 46, 1.1922183275246956), (77, 1, 51, 1.1565897822205915), (85, 6, 53, 1.180489039926458), (77, 1, 53, 0.9485557944872456), (87, 38, 54, 1.0294580934454554), (85, 6, 54, 1.1794723617328615), (77, 1, 54, 1.186947116263658), (87, 38, 55, 1.1820755137515744), (85, 6, 55, 1.109748656653979), (77, 1, 55, 1.184438790808682), (87, 38, 56, 1.1489331570403123), (85, 6, 56, 1.1272145808958596), (77, 1, 56, 0.8317797090321448), (87, 38, 57, 1.1954978679856958), (77, 1, 57, 0.5998074854138817), (77, 1, 58, 0.4252888657911017), (77, 1, 59, 0.36666699500133076), (62, 48, 59, 1.1376590291644992), (77, 1, 60, 0.42570571828336773), (62, 48, 60, 1.0679855681246566), (77, 1, 61, 0.6007450492076578), (62, 48, 61, 1.1854965439590688), (77, 1, 62, 0.8871973896659071), (62, 48, 62, 1.1297119561280675), (81, 38, 64, 1.0905108914115158), (62, 48, 64, 1.117746000397937), (98, 38, 65, 1.1864910214583422), (81, 38, 65, 1.0882932773736174), (98, 38, 66, 1.1470380111781422), (81, 38, 66, 1.157143837849419), (98, 38, 67, 1.1928672833174618), (81, 38, 67, 1.1534600218298106), (98, 38, 68, 1.1724721636987705), (81, 38, 68, 1.1532311445264474), (81, 38, 69, 1.153317040171032), (98, 38, 70, 1.194633571267385), (81, 38, 70, 1.1683588641838736), (46, 38, 70, 1.1959769355352674), (98, 38, 71, 1.1982368666719856), (46, 38, 71, 1.1978891712567934), (81, 1, 75, 1.0059474422912413), (81, 1, 76, 0.9906940588539097), (46, 1, 76, 1.1990271430336812), (46, 1, 77, 1.1569255986456997), (97, 34, 78, 1.1779822573455645), (81, 1, 78, 1.0870447519582251), (81, 1, 79, 1.1179554336944184), (81, 1, 80, 1.1197120017132265), (46, 1, 80, 1.1508588593885019), (81, 1, 81, 1.150576345489076), (46, 1, 81, 1.1858396848718298), (46, 1, 82, 1.193144639078862), (98, 1, 83, 1.1631387848043546), (85, 1, 83, 1.1949350398710727), (98, 1, 84, 1.1652572146624054), (98, 1, 85, 0.9060939354257506), (95, 1, 85, 1.1910489926706753), (98, 1, 86, 1.0034034259728173), (98, 1, 87, 1.0867463230681291), (98, 1, 88, 1.0866795516096264), (98, 1, 89, 1.0288624659727106), (95, 1, 89, 1.1965543746426535), (98, 1, 90, 1.123540451653149), (85, 1, 90, 1.1847709843120187), (34, 22, 90, 1.1958819001386713), (85, 1, 91, 1.1924305514227458), (15, 1, 94, 1.0773256497954644), (15, 1, 95, 1.1741753876851464), (15, 1, 96, 1.177761670225147), (98, 1, 97, 1.1803596041748157), (85, 1, 97, 1.1705924293753012), (15, 1, 97, 1.179860386970681), (98, 1, 98, 1.1068634794748735), (15, 1, 98, 1.129285230793379), (98, 1, 99, 1.1147269262546724), (15, 1, 99, 1.1778696630259338), (98, 1, 100, 1.1104443747178563), (98, 25, 104, 0.9569058441751841), (30, 15, 113, 1.1566028106314636), (30, 15, 114, 1.1813318803551174), (85, 30,
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                    118, 1.1358866603904039), (85, 30, 119, 1.138160918872302), (13, 1, 119, 1.1756828856078072), (13, 1, 120, 1.0821626515742844), (71, 30, 123, 1.1757329277463995), (71, 30, 124, 1.1298388024663637), (71, 30, 125, 1.1759289833093454), (71, 30, 126, 1.1769467819560135), (71, 30, 127, 1.1769263384248945), (71, 30, 128, 1.1513740266674537), (69, 64, 128, 1.1965600774248595), (28, 6, 128, 1.1488854747803983), (28, 6, 129, 1.1934573862186508), (71, 13, 133, 1.1980466635648772), (71, 13, 134, 1.1227146081835788), (71, 13, 135, 1.114616508978558), (79, 1, 136, 1.1780834004819907), (71, 13, 136, 1.1517681342855652), (79, 1, 137, 1.0076702474329005), (71, 13, 137, 1.1602227599055814), (79, 1, 138, 1.0709616133063853), (71, 57, 138, 1.0133158119632184), (71, 13, 138, 1.148816177500322), (79, 1, 139, 1.0714234166403758), (71, 57, 139, 1.0918504076538489), (71, 13, 139, 1.1246368333310248), (57, 30, 139, 1.1891144944595102), (57, 13, 139, 1.165365384974007), (79, 1, 140, 1.0714973899194358), (71, 57, 140, 1.0917268990289437), (71, 13, 140, 1.1793614298755002), (57, 30, 140, 1.175037493000908), (79, 1, 141, 1.071497825987907), (71, 57, 141, 1.0083479013355867), (57, 30, 141, 1.1883554675210604), (79, 1, 142, 1.1236362121855792), (71, 57, 142, 1.0550112085530106), (57, 30, 142, 1.1870904916146472), (57, 13, 142, 1.1849977252622512), (57, 30, 143, 1.1983476154348665), (57, 13, 143, 1.1927653774010212), (80, 6, 157, 1.1926930844227035), (80, 33, 161, 1.1590015812687586), (80, 33, 162, 1.132392865748863), (80, 33, 163, 1.1786301999563487), (80, 33, 164, 1.1794518934551335), (80, 33, 165, 1.1790856809495813), (80, 33, 166, 1.1790394794787264), (80, 33, 167, 1.0193498737480213), (80, 33, 168, 0.9673912096798788), (80, 33, 169, 1.0248271178199009), (12, 2, 191, 1.1077070115886831), (12, 2, 192, 1.1423463584890714), (12, 2, 193, 1.1600864766397285), (45, 12, 194, 1.196516461116262), (12, 2, 194, 1.1908076320329368), (12, 2, 195, 1.1987160452075256), (12, 2, 196, 1.1999403809060083), (79, 28, 207, 1.184233685681477), (79, 48, 211, 1.0877842667200865), (79, 48, 212, 1.12051885568824), (79, 48, 213, 1.023938351041138), (79, 48, 214, 0.998122313938052), (48, 29, 214, 1.1687800978233396), (79, 48, 215, 0.9836404145984724), (79, 48, 216, 0.984461339673762), (48, 29, 216, 1.142322742760913), (79, 48, 217, 0.9119090997406367), (48, 29, 217, 1.147781980570008), (79, 48, 218, 1.0042364701295865), (48, 29, 218, 1.1496769657822066), (48, 29, 219, 1.15793409961186), (53, 21, 238, 1.122469829277304), (53, 40, 240, 1.1969125955840996), (48, 0, 262, 1.1751270151049864), (48, 0, 263, 1.1784441296377448), (48, 0, 264, 1.1795232593732012), (48, 0, 265, 1.1845491035133968)]
    data = fdata

    # 1.1306474056507756
    data = [(12, 59, 134, 1.1771931495500132), (15, 98, 153, 1.1675544080428022), (16, 64, 79, 1.179854114860037), (16, 68, 396, 1.1989128059402847), (16, 88, 116, 1.186454277747198), (16, 99, 139, 1.198693202965533), (20, 59, 125, 1.1918580529502292), (30, 78, 80, 1.1373335420398178), (34, 87, 220, 1.1997764997578373), (44, 60, 187, 1.1961281161325004), (50, 55, 74,
                                                                                                                                                                                                                                                                                                                                                                      1.1992561806813513), (55, 64, 74, 1.1885775726187333), (55, 68, 394, 1.199574080596218), (55, 88, 114, 1.1837839905948284), (55, 94, 166, 1.199281438069076), (55, 99, 93, 1.1730860147157576), (60, 87, 181, 1.1306474056507756), (63, 94, 160, 1.1958840301270404), (68, 99, 202, 1.196523971612483), (88, 99, 109, 1.187637092534975), (96, 98, 147, 1.155125408898313)]
    # 1.0484213585875937
    fdata = [(55, 50, 70, 1.1808091656697355), (55, 50, 72, 1.1934443425698023), (78, 30, 74, 1.0960480478942514), (55, 50, 74, 1.1973016243356318), (55, 50, 75, 1.1992561806813513), (64, 16, 79, 1.1756760864137485), (64, 16, 80, 1.179854114860037), (78, 30, 81, 1.1373335420398178), (78, 30, 82, 1.111400223164843), (64, 16, 86, 1.194815484982389), (64, 16, 87, 1.198729963444141), (99, 64, 89, 1.1967779449422757), (64, 16, 89, 1.199279513242127), (64, 16, 90, 1.1996248497059603), (64, 16, 94, 1.1976692345585376), (78, 23, 106, 1.1421113607668925), (99, 88, 109, 1.1812513098276203), (99, 88, 110, 1.187637092534975), (88, 16, 110, 1.1999658190923952), (72, 23, 111, 1.1895796436144646), (88, 16, 116, 1.1831135548579241), (88, 16, 117, 1.186454277747198), (78, 5, 118, 1.185905040928638), (78, 5, 119, 1.1871159066300958), (59, 20, 121, 1.1741658785786289), (59, 20, 123, 1.183934231364801), (59, 20, 125, 1.1847598234891208), (59, 20, 126, 1.1918580529502292), (87, 60, 136, 1.1979618877288463), (99, 16, 139, 1.1973933057164445), (99, 16, 140, 1.198693202965533), (98, 96, 146, 1.0859578506092125), (98, 96, 147, 1.1552822223763783), (98, 96, 148, 1.155125408898313), (98, 15, 148, 1.1789491962099872), (98, 96, 149, 1.1097275843617307), (98, 15, 149, 1.1871051078568668), (98, 15, 154, 1.1675544080428022), (98, 15, 155, 1.1232245202003555), (98, 15, 156, 1.1664673622133517),
             (15, 4, 160, 1.187615600419327), (15, 4, 161, 1.1067499505210447), (99, 16, 169, 1.1997128734871922), (99, 16, 171, 1.1996401687327862), (99, 16, 173, 1.1975764814676995), (99, 16, 174, 1.1987401387861913), (99, 16, 175, 1.1988044148388721), (99, 16, 176, 1.1995481158549985), (99, 16, 177, 1.1979008598840906), (99, 16, 178, 1.1991993792936577), (87, 60, 180, 1.0484213585875937), (87, 60, 181, 1.1306483719811629), (87, 60, 182, 1.1306474056507756), (87, 60, 183, 1.1740649352550154), (99, 68, 196, 1.1971824607996098), (99, 68, 200, 1.1991279426323909), (99, 68, 202, 1.1916536087118177), (99, 68, 203, 1.196523971612483), (99, 68, 205, 1.1988172376753732), (99, 68, 207, 1.1981295448297664), (68, 16, 224, 1.1999999998401183), (68, 16, 225, 1.1999999986762642), (68, 16, 231, 1.1999999995845567), (99, 68, 394, 1.1998304441936634), (68, 16, 394, 1.1999578395181894), (99, 68, 395, 1.1995291524939018), (68, 16, 395, 1.1996958448680584), (99, 68, 396, 1.1997050673504666), (68, 16, 396, 1.1977347100679137), (99, 68, 397, 1.198781900402414), (68, 16, 397, 1.1989128059402847), (99, 68, 398, 1.199730540015984), (68, 16, 398, 1.1976638009251892), (99, 68, 399, 1.1976013466180733), (94, 68, 399, 1.1971240558683454), (68, 16, 399, 1.198774822295333), (99, 68, 400, 1.1994116457920845), (94, 68, 400, 1.1943974268660948), (68, 16, 400, 1.197826557892774)]
    data = fdata

    # 0.984461339673762
    data = [(0, 48, 264, 1.1845491035133968), (1, 15, 96, 1.179860386970681), (1, 46, 81, 1.193144639078862), (1, 77, 54, 1.184438790808682), (1, 79, 140, 1.071497825987907), (1, 81, 79, 1.1197120017132265), (1, 98, 87, 1.0866795516096264), (13, 57, 142, 1.1927653774010212), (13, 71, 137, 1.148816177500322), (28, 48, 210, 1.187789543982482), (28, 79, 216, 1.1864422335681755),
            (29, 48, 218, 1.15793409961186), (30, 57, 141, 1.1870904916146472), (30, 71, 126, 1.1769263384248945), (33, 80, 165, 1.1790394794787264), (38, 46, 70, 1.1978891712567934), (38, 81, 68, 1.153317040171032), (40, 53, 239, 1.1969125955840996), (41, 79, 192, 1.1868974616413341), (48, 79, 215, 0.984461339673762), (57, 71, 139, 1.0917268990289437), (85, 95, 76, 1.1934091282037433)]
    # 0.36666699500133076
    fdata = [(95, 17, 46, 1.1922183275246956), (77, 1, 51, 1.1565897822205915), (85, 6, 53, 1.180489039926458), (77, 1, 53, 0.9485557944872456), (87, 38, 54, 1.0294580934454554), (85, 6, 54, 1.1794723617328615), (77, 1, 54, 1.186947116263658), (87, 38, 55, 1.1820755137515744), (85, 6, 55, 1.109748656653979), (77, 1, 55, 1.184438790808682), (87, 38, 56, 1.1489331570403123), (85, 6, 56, 1.1272145808958596), (77, 1, 56, 0.8317797090321448), (87, 38, 57, 1.1954978679856958), (77, 1, 57, 0.5998074854138817), (77, 1, 58, 0.4252888657911017), (77, 1, 59, 0.36666699500133076), (62, 48, 59, 1.1376590291644992), (77, 1, 60, 0.42570571828336773), (62, 48, 60, 1.0679855681246566), (77, 1, 61, 0.6007450492076578), (62, 48, 61, 1.1854965439590688), (77, 1, 62, 0.8871973896659071), (62, 48, 62, 1.1297119561280675), (81, 38, 64, 1.0905108914115158), (62, 48, 64, 1.117746000397937), (98, 38, 65, 1.1864910214583422), (81, 38, 65, 1.0882932773736174), (98, 38, 66, 1.1470380111781422), (81, 38, 66, 1.157143837849419), (98, 38, 67, 1.1928672833174618), (81, 38, 67, 1.1534600218298106), (98, 38, 68, 1.1724721636987705), (81, 38, 68, 1.1532311445264474), (81, 38, 69, 1.153317040171032), (98, 38, 70, 1.194633571267385), (81, 38, 70, 1.1683588641838736), (46, 38, 70, 1.1959769355352674), (98, 38, 71, 1.1982368666719856), (46, 38, 71, 1.1978891712567934), (81, 1, 75, 1.0059474422912413), (81, 1, 76, 0.9906940588539097), (46, 1, 76, 1.1990271430336812), (46, 1, 77, 1.1569255986456997), (97, 34, 78, 1.1779822573455645), (81, 1, 78, 1.0870447519582251), (81, 1, 79, 1.1179554336944184), (81, 1, 80, 1.1197120017132265), (46, 1, 80, 1.1508588593885019), (81, 1, 81, 1.150576345489076), (46, 1, 81, 1.1858396848718298), (46, 1, 82, 1.193144639078862), (98, 1, 83, 1.1631387848043546), (85, 1, 83, 1.1949350398710727), (98, 1, 84, 1.1652572146624054), (98, 1, 85, 0.9060939354257506), (95, 1, 85, 1.1910489926706753), (98, 1, 86, 1.0034034259728173), (98, 1, 87, 1.0867463230681291), (98, 1, 88, 1.0866795516096264), (98, 1, 89, 1.0288624659727106), (95, 1, 89, 1.1965543746426535), (98, 1, 90, 1.123540451653149), (85, 1, 90, 1.1847709843120187), (34, 22, 90, 1.1958819001386713), (85, 1, 91, 1.1924305514227458), (15, 1, 94, 1.0773256497954644), (15, 1, 95, 1.1741753876851464), (15, 1, 96, 1.177761670225147), (98, 1, 97, 1.1803596041748157), (85, 1, 97, 1.1705924293753012), (15, 1, 97, 1.179860386970681), (98, 1, 98, 1.1068634794748735), (15, 1, 98, 1.129285230793379), (98, 1, 99, 1.1147269262546724), (15, 1, 99, 1.1778696630259338), (98, 1, 100, 1.1104443747178563), (98, 25, 104, 0.9569058441751841), (30, 15, 113, 1.1566028106314636), (30, 15, 114, 1.1813318803551174), (85, 30,
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                    118, 1.1358866603904039), (85, 30, 119, 1.138160918872302), (13, 1, 119, 1.1756828856078072), (13, 1, 120, 1.0821626515742844), (71, 30, 123, 1.1757329277463995), (71, 30, 124, 1.1298388024663637), (71, 30, 125, 1.1759289833093454), (71, 30, 126, 1.1769467819560135), (71, 30, 127, 1.1769263384248945), (71, 30, 128, 1.1513740266674537), (69, 64, 128, 1.1965600774248595), (28, 6, 128, 1.1488854747803983), (28, 6, 129, 1.1934573862186508), (71, 13, 133, 1.1980466635648772), (71, 13, 134, 1.1227146081835788), (71, 13, 135, 1.114616508978558), (79, 1, 136, 1.1780834004819907), (71, 13, 136, 1.1517681342855652), (79, 1, 137, 1.0076702474329005), (71, 13, 137, 1.1602227599055814), (79, 1, 138, 1.0709616133063853), (71, 57, 138, 1.0133158119632184), (71, 13, 138, 1.148816177500322), (79, 1, 139, 1.0714234166403758), (71, 57, 139, 1.0918504076538489), (71, 13, 139, 1.1246368333310248), (57, 30, 139, 1.1891144944595102), (57, 13, 139, 1.165365384974007), (79, 1, 140, 1.0714973899194358), (71, 57, 140, 1.0917268990289437), (71, 13, 140, 1.1793614298755002), (57, 30, 140, 1.175037493000908), (79, 1, 141, 1.071497825987907), (71, 57, 141, 1.0083479013355867), (57, 30, 141, 1.1883554675210604), (79, 1, 142, 1.1236362121855792), (71, 57, 142, 1.0550112085530106), (57, 30, 142, 1.1870904916146472), (57, 13, 142, 1.1849977252622512), (57, 30, 143, 1.1983476154348665), (57, 13, 143, 1.1927653774010212), (80, 6, 157, 1.1926930844227035), (80, 33, 161, 1.1590015812687586), (80, 33, 162, 1.132392865748863), (80, 33, 163, 1.1786301999563487), (80, 33, 164, 1.1794518934551335), (80, 33, 165, 1.1790856809495813), (80, 33, 166, 1.1790394794787264), (80, 33, 167, 1.0193498737480213), (80, 33, 168, 0.9673912096798788), (80, 33, 169, 1.0248271178199009), (12, 2, 191, 1.1077070115886831), (12, 2, 192, 1.1423463584890714), (12, 2, 193, 1.1600864766397285), (45, 12, 194, 1.196516461116262), (12, 2, 194, 1.1908076320329368), (12, 2, 195, 1.1987160452075256), (12, 2, 196, 1.1999403809060083), (79, 28, 207, 1.184233685681477), (79, 48, 211, 1.0877842667200865), (79, 48, 212, 1.12051885568824), (79, 48, 213, 1.023938351041138), (79, 48, 214, 0.998122313938052), (48, 29, 214, 1.1687800978233396), (79, 48, 215, 0.9836404145984724), (79, 48, 216, 0.984461339673762), (48, 29, 216, 1.142322742760913), (79, 48, 217, 0.9119090997406367), (48, 29, 217, 1.147781980570008), (79, 48, 218, 1.0042364701295865), (48, 29, 218, 1.1496769657822066), (48, 29, 219, 1.15793409961186), (53, 21, 238, 1.122469829277304), (53, 40, 240, 1.1969125955840996), (48, 0, 262, 1.1751270151049864), (48, 0, 263, 1.1784441296377448), (48, 0, 264, 1.1795232593732012), (48, 0, 265, 1.1845491035133968)]
    # 0.984461339673762
    data = fdata

    # 1.052823026477058
    data = [(1, 53, 37, 1.1840382322224074), (1, 55, 27, 1.175589848206993), (2, 58, 100, 1.1853574056618708), (4, 47, 152, 1.191211728252616), (10, 30, 88, 1.1692181348893986), (10, 80, 130, 1.1979357766960355), (10, 82, 130, 1.1754147119549883), (11, 24, 139, 1.1952640400898498), (12, 20, 158, 1.1967380081749928), (12, 30, 160, 1.052823026477058), (19, 21, 25, 1.178243780998494), (21, 53, 34, 1.1212061630424002), (21, 55, 26, 1.1482440042397548),
            (24, 81, 141, 1.1858125007479996), (26, 81, 145, 1.191631774396556), (30, 56, 102, 1.171192961233224), (39, 71, 48, 1.191874601002975), (40, 53, 72, 1.1860635718105905), (40, 81, 72, 1.1841081845923562), (43, 52, 22, 1.162317770171456), (43, 62, 30, 1.1183521894605655), (50, 64, 106, 1.1957778021037575), (52, 62, 35, 1.1518311906588643), (71, 99, 48, 1.1534150897570563), (76, 88, 91, 1.1455950635482086), (87, 95, 89, 1.1980380864472744)]
    # 0.5280749451133793
    fdata = [(55, 21, 23, 1.1923470423191407), (55, 1, 23, 1.152211701550094), (52, 43, 23, 1.162317770171456), (21, 19, 23, 1.1779815123647703), (55, 21, 24, 1.1283489955651014), (55, 1, 24, 1.1577489039292748), (52, 43, 24, 1.088104914821242), (21, 19, 24, 1.189915825300733), (55, 21, 25, 1.1409326259326933), (55, 1, 25, 1.171534973601651), (52, 43, 25, 1.1053319126709937), (21, 19, 25, 1.1785345625329242), (55, 21, 26, 1.1470374805822399), (55, 1, 26, 1.176283512806608), (21, 19, 26, 1.178243780998494), (55, 21, 27, 1.1482440042397548), (55, 1, 27, 1.1747372774688598), (21, 19, 27, 1.1695262722760844), (55, 21, 28, 1.1753237817906754), (55, 1, 28, 1.175589848206993), (21, 19, 28, 1.1930303986941029), (62, 43, 30, 1.1141480361286178), (62, 43, 31, 1.1183521894605655), (62, 34, 31, 1.1410340739052927), (53, 21, 31, 1.1753450417137792), (62, 43, 32, 1.135203823834673), (62, 34, 32, 1.1964496687077246), (53, 21, 32, 0.8219060443613675), (53, 21, 33, 1.1197189949537738), (55, 13, 34, 1.1690595539388415), (53, 21, 34, 1.1217099826044756), (21, 19, 34, 1.1050711642832598), (53, 21, 35, 1.1212061630424002), (53, 1, 35, 1.1962335427421933), (21, 19, 35, 1.1151816870806706), (53, 21, 36, 0.5536918996299068), (53, 1, 36, 1.138965028379732), (30, 14, 36, 1.1169813676149027), (27, 9, 36, 1.195194967395815), (21, 19, 36, 1.1929941912841828), (53, 21, 37, 0.5280749451133793), (53, 1, 37, 1.1828780668592567), (52, 34, 37, 1.1943730693753747), (21, 19, 37, 1.1988593428140704), (53, 21, 38, 1.1775799975127708), (53, 1, 38, 1.1840382322224074), (21, 19, 38, 1.1995997161174934), (69, 1, 39, 1.1992091513982053), (53, 1, 39, 1.143199067750332), (21, 19, 39, 1.1909145061234414), (53, 13, 41, 1.150447621647537), (99, 71, 42, 1.1373763787101947), (99, 71, 43, 1.1717041088858495), (89, 34, 43, 1.0390133899294636), (99, 71, 44, 1.0986942502229395), (89, 34, 44, 1.1866963925510141), (71, 39, 44, 1.199644481354056), (99, 71, 45, 1.118989492814025), (71, 39, 45, 1.1830060896388044), (99, 71, 46, 1.1336074198147694), (71, 39, 46, 1.1876112970167092), (99, 71, 47, 1.1451598565415486), (71, 39, 47, 1.190354568545926), (99, 71, 48, 1.1512622061740998), (71, 39, 48, 1.1864360889033814), (99, 71, 49, 1.1534150897570563), (71, 39, 49, 1.191874601002975), (53, 50, 49, 1.0841336620864486), (99, 71, 50, 1.191989355031504), (62, 29, 61, 1.156712425635897), (53, 40, 67, 1.1469731298167267), (81, 40, 68, 0.9176683672926226), (81, 40, 69, 1.1717995760672382), (53, 40, 69, 1.1811698081140478), (81, 40, 70, 1.1729149028963075), (53, 40, 70, 1.1955048654349492), (81, 40, 71, 1.1779299869933118), (53, 40, 71, 1.18618886458529), (81, 40, 72, 1.183375634779918), (53, 40, 72, 1.1839131265470364), (81, 40, 73, 1.1841081845923562), (53, 40, 73, 1.1860635718105905), (81, 40, 74, 1.134410574445469), (58, 10, 78, 1.1573832240185984), (30, 10, 83, 1.1514614703908648), (95, 87, 84, 1.1984099690429406), (30, 10, 84, 1.0489591916008574), (30, 10, 85, 1.167038754589909), (95, 87, 86, 1.1942086620430785), (30, 10, 86, 1.1693856944049785), (88, 76, 87, 1.1206240437462547), (30, 10, 87, 1.1692051554560405), (95, 87, 88, 1.1941928889984386), (88, 76, 88, 1.149565279319035), (30, 10, 88, 1.1692100568347987), (88, 76, 89, 1.1619363348583092), (30, 10, 89, 1.1692181348893986), (95, 87, 90, 1.1980380864472744), (88, 76, 90, 1.1490784688167015), (30, 10, 90, 1.198485547740144), (95, 87, 91, 1.1965037566347818), (88, 76, 91, 1.1457355909083915), (95, 87, 92, 1.190316997475029), (88, 76, 92, 1.1455950635482086), (29, 19, 92, 1.1982043696323705), (88, 76, 93, 0.9897113292463655), (95, 87, 94, 1.1941444370379648), (88, 76, 94, 1.0978666383997577), (58, 2, 95, 1.1880832848394467), (95, 87, 96, 1.1944281172989), (58, 2, 96, 1.0929972318641763), (58, 2, 97, 1.1130984743717391), (95, 87, 98, 1.1980351780382672), (64, 13, 98, 1.1178715078158132), (58, 2, 98, 1.0714094376932444), (95, 87, 99, 1.197001383078464), (64, 13, 99, 1.031552606871317), (58, 2, 99, 1.1671028861443822), (95, 87, 100,
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                        1.1991694187760369), (58, 2, 100, 1.1846750230884413), (64, 50, 101, 1.1224526508067707), (58, 2, 101, 1.1853574056618708), (95, 87, 102, 1.1956446615136813), (64, 50, 102, 1.0398146611368733), (64, 50, 103, 1.0803018505783715), (95, 87, 104, 1.1945342465703277), (64, 50, 104, 1.161924238365916), (95, 87, 105, 1.1996542202658564), (64, 50, 105, 1.182615182725394), (95, 87, 106, 1.1948737492402726), (64, 50, 106, 1.1914926452015988), (64, 50, 107, 1.1957778021037575), (95, 87, 108, 1.1973005588609134), (95, 87, 110, 1.1893052305890386), (95, 87, 111, 1.195643757582706), (95, 87, 112, 1.1990594777689236), (95, 87, 114, 1.1928301786937159), (95, 87, 115, 1.1999402819243927), (95, 87, 116, 1.1884827798855617), (95, 87, 118, 1.195369751748778), (95, 87, 120, 1.1928223142029295), (82, 18, 120, 1.1633951551052486), (95, 87, 121, 1.1997500062096402), (95, 87, 122, 1.1927043754069127), (95, 87, 124, 1.1973247345985323), (95, 87, 126, 1.1941481596518264), (82, 10, 126, 1.1593214098701912), (82, 10, 127, 0.8436759421773609), (95, 87, 128, 1.1947998212369138), (82, 10, 128, 1.0247913580206565), (82, 10, 129, 1.1181408381440958), (80, 10, 129, 1.185012212752141), (53, 11, 129, 1.1934741443385546), (95, 87, 130, 1.1945578098712968), (82, 10, 130, 1.1728769377957597), (80, 10, 130, 1.1945842592715066), (69, 11, 130, 1.1792649781250735), (82, 10, 131, 1.1754147119549883), (80, 10, 131, 1.1979357766960355), (95, 87, 132, 1.1949759344976862), (82, 10, 132, 0.9716539793093902), (80, 10, 132, 1.196256693719251), (82, 10, 133, 0.8934104340328127), (80, 10, 133, 1.1951697586333219), (95, 87, 134, 1.1944170749655096), (82, 10, 134, 0.7992052912033888), (82, 10, 135, 0.7926287557037884), (24, 11, 135, 0.9940623733520142), (95, 87, 136, 1.196386900979694), (82, 10, 136, 0.9249518330121522), (24, 11, 136, 1.0627328656018515), (82, 10, 137, 1.1821432652044577), (53, 26, 137, 1.1728993060740862), (24, 11, 137, 0.9155296543712081), (95, 87, 138, 1.1953399432218867), (81, 24, 138, 1.1339121401942545), (53, 26, 138, 1.1345052756046627), (24, 11, 138, 1.1534807228226311), (81, 24, 139, 1.191761386282296), (53, 26, 139, 1.1294709121803623), (50, 4, 139, 1.1553369421316206), (24, 11, 139, 1.191683891958193), (95, 87, 140, 1.1975254075178838), (81, 24, 140, 1.1636578851933974), (24, 11, 140, 1.1952640400898498), (81, 24, 141, 1.1781744520412858), (33, 15, 141, 1.1598484166129053), (24, 11, 141, 1.1335424648374721), (95, 87, 142, 1.1993467565492093), (81, 26, 142, 0.9168804691529622), (81, 24, 142, 1.1858125007479996), (81, 26, 143, 1.1212244335861528), (95, 87, 144, 1.1967568311820822), (95, 86, 144, 1.1858055918628967), (81, 26, 144, 1.1677636490699106), (75, 13, 144, 1.169374356663975), (81, 26, 145, 1.1842683593942813), (75, 13, 145, 1.1669903085566424), (95, 87, 146, 1.1967159286922477), (95, 86, 146, 1.1706966060360875), (87, 86, 146, 1.181643401869301), (81, 26, 146, 1.191631774396556), (47, 4, 147, 1.149690319017742), (95, 87, 148, 1.1970013617021056), (47, 4, 148, 1.141441450719978), (47, 4, 149, 1.1720408351796803), (95, 87, 150, 1.1989376839401855), (47, 4, 150, 1.1897994931096294), (47, 4, 151, 1.1912289922948014), (95, 87, 152, 1.1940069962725015), (47, 4, 152, 1.1912113929997203), (95, 87, 153, 1.1976602802346676), (87, 35, 153, 1.1999961663649592), (47, 4, 153, 1.191211728252616), (95, 87, 154, 1.195536754260562), (47, 4, 154, 1.1911963567624593), (20, 12, 154, 1.1796924995837343), (20, 12, 155, 1.1809640936861636), (30, 12, 156, 1.0699086543300456), (20, 12, 156, 1.1321050869136824), (20, 12, 157, 1.1661731422767014), (30, 12, 158, 1.18316768023491), (30, 12, 159, 1.19381505331678), (20, 12, 159, 1.1967380081749928), (30, 12, 160, 1.0485244889295087), (30, 12, 161, 1.052823026477058), (30, 12, 162, 1.0014698589708257), (30, 12, 163, 1.0609589589069723), (48, 20, 176, 1.1587803257824705), (66, 35, 178, 1.1845557075905924), (48, 20, 179, 1.1479827843672545), (19, 10, 244, 1.0988323672281273), (50, 10, 248, 1.1969728483229622)]
    data = fdata

    # 0.9915845964412623
    data = [(1, 98, 89, 1.172942936977747), (4, 21, 103, 1.1950827203793823), (4, 96, 251, 1.0049375185134999), (19, 98, 108, 1.1638588894501591), (21, 52, 73, 0.9915845964412623), (29, 74, 86,
                                                                                                                                                                                      1.0801636953510245), (52, 67, 90, 1.150027738510446), (58, 90, 58, 1.1222014915845482), (60, 66, 57, 1.1409681428355927), (67, 89, 88, 1.1701997654588778), (71, 89, 83, 1.17147424234141)]

    # 0.7236760161952848
    fdata = [(66, 23, 28, 1.1875370710186959), (96, 53, 31, 1.1205659531225671), (87, 77, 37, 1.0985525070340578), (87, 77, 38, 1.1649850262512889), (87, 6, 46, 1.1747268386170473), (66, 60, 51, 1.1615558858256287), (66, 60, 52, 1.162821825118929), (66, 21, 52, 1.1708091105508585), (66, 60, 53, 1.1315567403938802), (38, 20, 53, 1.1974455412764433), (90, 58, 54, 1.1963943999232705), (66, 60, 54, 1.1395557065013837), (90, 58, 55, 1.006722265454582), (66, 60, 55, 1.1409335586430127), (90, 58, 56, 1.102070356975875), (66, 60, 56, 1.1409540067184518), (90, 58, 57, 1.132877417374656), (66, 60, 57, 1.1408032339881367), (96, 70, 58, 1.1723016773377535), (90, 58, 58, 1.1226731297711017), (66, 60, 58, 1.1409681428355927), (96, 70, 59, 1.1339889214174823), (90, 58, 59, 1.1222014915845482), (66, 60, 59, 1.1912116249062223), (90, 58, 60, 1.086177515359315), (88, 27, 63, 1.1623128617124423), (60, 58, 65, 1.0845720345560117), (90, 89, 66, 1.1479644287857336), (90, 52, 66, 1.1143408752386499), (90, 52, 67, 1.1708146295203183), (52, 21, 69, 0.9589967608900377), (52, 21, 70, 1.1010714470726024), (74, 60, 71, 1.1290954115227847), (52, 21, 71, 0.9760790050260947), (52, 21, 72, 0.9910333937622116), (52, 21, 73, 0.9914463456211046), (52, 21, 74, 0.9915845964412623), (52, 21, 75, 0.7236760161952848), (74, 71, 76, 1.1737970276185308), (52, 21, 76, 0.7341085481029953), (74, 71, 77, 1.176881792098924), (52, 21, 77, 1.0198788030309758), (58, 29, 78, 1.1929525358007593), (89, 71, 79, 1.1899335151718011), (83, 77, 79, 1.127997114201616), (89, 71, 80, 0.9796597906149834),
             (89, 71, 81, 1.0306557213380858), (89, 71, 82, 1.1676324475404667), (74, 29, 82, 1.0952549771638633), (71, 52, 82, 1.14674772846246), (89, 71, 83, 1.1714667094961404), (74, 55, 83, 1.1795043592521723), (74, 29, 83, 0.9452447813466098), (71, 52, 83, 1.1592050912215146), (98, 1, 84, 1.1413442716959816), (89, 71, 84, 1.17147424234141), (74, 55, 84, 1.1289683660127683), (74, 29, 84, 1.1190570944051463), (71, 52, 84, 1.1859247235418287), (98, 48, 85, 1.1758931705698135), (98, 1, 85, 0.9452717484253881), (74, 29, 85, 1.0558369136003427), (98, 48, 86, 1.1512109631009009), (98, 1, 86, 1.0766087134256042), (96, 15, 86, 1.1592927197904181), (74, 29, 86, 1.0805304306477514), (67, 52, 86, 1.1863051460431242), (98, 1, 87, 1.0969086972147164), (89, 55, 87, 1.1076124649697539), (74, 29, 87, 1.0801636953510245), (67, 52, 87, 0.8720642352451109), (98, 1, 88, 1.10054739261917), (74, 29, 88, 1.0713802515686868), (67, 52, 88, 1.095215710888853), (98, 1, 89, 1.1687920069780167), (89, 29, 89, 1.1838762342072284), (67, 52, 89, 1.158135552529779), (98, 1, 90, 1.172942936977747), (67, 52, 90, 1.1502198624493711), (98, 1, 91, 1.1524584056917797), (67, 52, 91, 1.150027738510446), (67, 52, 92, 1.132653869291278), (71, 16, 103, 1.1914075746815895), (98, 79, 128, 1.0676908519210224), (98, 79, 129, 1.132516642689893), (89, 1, 133, 1.1671984074892299), (52, 1, 138, 1.0866446148989588), (52, 1, 139, 1.0995788423498378), (98, 36, 142, 1.1860493117354682), (98, 36, 143, 1.1691855776704085), (97, 96, 156, 1.1266995333811363), (76, 47, 201, 1.1460605551227543)]
    data = fdata

    # 0.5917849555882795
    fdata = [(75, 0, 66, 1.1376218252801038), (75, 0, 67, 1.1570321002347015), (75, 0, 68, 1.0569255470355663), (75, 0, 69, 0.9775791896689536), (63, 24, 69, 1.1178743155806203), (75, 0, 70, 0.9288200217707309), (63, 24, 70, 1.1410642465093683), (35, 24, 70, 1.1982979056768688), (75, 0, 71, 0.9250608709123849), (63, 24, 71, 1.168766487503644), (75, 0, 72, 0.679187542533823), (63, 24, 72, 1.1853164026511744), (75, 0, 73, 0.5917849555882795), (63, 24, 73, 1.191552314621232), (75, 0, 74, 0.7124952010197662), (63, 24, 74, 1.193562360033836), (75, 0, 75, 0.9341783028554803), (63, 24, 75, 1.193841415075365), (35, 24, 75, 1.197138507107833), (35, 24, 76, 1.1985863295565584), (62, 56, 94, 1.1758824685926312), (62, 56, 95, 1.1517149421668333), (62, 19, 95, 1.1999469118879604), (62, 56, 96, 1.1876591391667586), (62, 19, 96, 1.1954846217262802), (62, 19, 97, 1.1979626008550444), (75, 55, 103, 1.1975744318606603), (75, 55, 104, 1.1864070957335677), (41, 19, 104, 1.1198547705353785), (45, 0, 105, 1.1702405724578306), (41, 19, 105, 1.1763486474058442), (45, 0, 106, 1.1312269830102941), (45, 0, 107, 1.181254135951802), (45, 0, 108,
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   1.1586020317852594), (62, 24, 109, 1.1921620920578293), (45, 0, 109, 1.1619270168553604), (86, 0, 119, 1.183790465301093), (62, 28, 120, 1.1382208701807917), (66, 62, 123, 1.1361708651351976), (66, 62, 124, 1.1713172810165704), (62, 60, 125, 1.0331407343702061), (62, 60, 126, 1.1858139112063721), (66, 62, 127, 1.1853504671217823), (62, 60, 127, 1.193585425770548), (66, 62, 128, 1.1876072599848424), (62, 60, 128, 1.1854179441295618), (62, 60, 129, 1.1850981546373447), (62, 60, 130, 1.1034812870494177), (81, 66, 152, 1.0262844515230736), (81, 66, 153, 1.1780302952617852), (81, 66, 154, 1.0532586002489044), (81, 66, 155, 1.0606664352610553), (81, 41, 155, 1.1598224359712879), (81, 66, 156, 1.0615454974808938), (81, 41, 156, 1.168371209428379), (81, 66, 157, 1.061633009527151), (81, 41, 157, 1.184998129969723), (81, 66, 158, 1.082489349130551), (81, 41, 158, 1.1894679612959438), (83, 60, 165, 1.1409797675128646), (83, 8, 175, 1.1509148163203908), (83, 8, 176, 1.107582565492107), (83, 8, 177, 1.0893431295486766), (83, 8, 178, 1.169995968232136), (83, 8, 179, 1.1858358140473924), (83, 8, 180, 1.1870570678217292)]
    data = fdata

    # 0.9250608709123849
    data = [(0, 45, 108, 1.1619270168553604), (0, 75, 70, 0.9250608709123849), (8, 83, 179, 1.1870570678217292), (24, 35, 75, 1.1985863295565584), (24, 63, 74,
                                                                                                                                                    1.193841415075365), (41, 81, 157, 1.1894679612959438), (60, 62, 128, 1.1850981546373447), (62, 66, 127, 1.1876072599848424), (66, 81, 156, 1.061633009527151)]

    # 降速 v=3
    # 0.5771172496292991
    fdata = [(70, 0, 29, 1.0431976075647633), (70, 0, 31, 1.1162797415873846), (77, 46, 56, 1.141778212556023), (46, 11, 60, 1.1929715748588703), (83, 43, 61, 1.184533712665862), (89, 59, 62, 1.1402094113561743), (89, 59, 63, 1.1711833718140754), (89, 59, 64, 1.0887041125154393), (89, 59, 65, 1.0539029261318604), (89, 59, 66, 1.055495840550687), (89, 59, 67, 0.5778183494162136), (89, 59, 68, 0.5771172496292991), (89, 59, 69, 1.1017254759266613), (89, 9, 69, 1.0738632416248477), (89, 9, 70, 1.0302327626718446), (77, 51, 78, 1.1980523423838878), (36, 9, 89, 1.1597092010090837), (75, 51, 90, 1.1175093379603442), (75, 51, 91, 1.109121113656043), (89, 88, 93, 1.0362759527869216), (77, 8, 99, 1.1472993058126342), (88,
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                              75, 100, 1.1802786014229585), (77, 8, 100, 1.197789303992164), (77, 8, 101, 1.188359678682499), (42, 12, 101, 1.1625486792141881), (88, 75, 105, 1.1397814410634883), (43, 3, 121, 1.1510535899379788), (43, 3, 122, 1.045723467332561), (43, 3, 123, 1.175438948039297), (42, 25, 126, 1.1966335492000444), (42, 25, 127, 1.1249110297879534), (60, 31, 136, 1.0729053327725437), (51, 32, 136, 1.1734345511894346), (63, 55, 137, 1.124845381371623), (51, 32, 137, 1.190641451979647), (63, 55, 138, 1.160201271489564), (63, 55, 139, 1.1993614248735671), (63, 55, 140, 1.1687166518666954), (63, 55, 141, 1.1504697397905148), (63, 55, 142, 1.1384686585072228), (63, 55, 143, 1.1352856470701842), (63, 55, 144, 1.1418092402328655)]
    data = fdata

    # 1.054241512175234
    data = [(6, 41, 119, 1.1380049197208275), (27, 55, 141, 1.054241512175234),
            (55, 63, 143, 1.1418092402328655), (59, 89, 65, 1.055495840550687)]

    # 0.9296829263843718
    fdata = [(26, 7, 56, 1.1219214064322411), (33, 0, 67, 1.0177861418976837), (33, 0, 68, 1.0219823062675375), (95, 33, 69, 1.1650001908422307), (33, 0, 69, 1.0183695930273076), (95, 33, 70, 1.1407776853985272), (33, 0, 70, 0.9642271899141848), (95, 33, 71, 1.1959027944464902), (31, 17, 72, 1.1816807046550832), (60, 46, 94, 1.1859491173236372), (41, 2, 96, 1.1320847037995054), (26, 2, 97, 1.0590658516777192), (26, 2, 98, 1.1930075150253585), (48, 2, 102, 1.0663697151652345),
             (48, 2, 103, 1.1174322885381849), (48, 2, 104, 1.1393424880824403), (34, 20, 105, 1.194259992095568), (22, 10, 124, 1.1794383501589045), (49, 2, 128, 1.1934938343720636), (40, 5, 131, 1.1444724028278792), (40, 5, 132, 1.127477680788904), (40, 5, 133, 1.156510603873974), (40, 5, 134, 0.9908600085575692), (40, 5, 135, 0.9296829263843718), (40, 5, 136, 0.9723034651175174), (40, 5, 137, 1.1174788538262557), (43, 2, 144, 0.9978866094149277), (99, 2, 157, 1.185434254500684)]
    data = fdata

    # 1.0183695930273076 
    data = [(0, 33, 68, 1.0183695930273076), (1, 41, 34, 1.1614673154372297), (5, 40, 132,
                                                                               1.156510603873974), (26, 48, 206, 1.1942914156154494), (37, 74, 107, 1.1447774698509756)]
    
    # 所有路径，1对多，1的i和2的j， n(n-1)/2
    # 0.9948156422569866
    fdata=[(26, 24, 87, 1.189716802404758), (74, 22, 92, 1.1598394442262145), (30, 28, 93, 1.1928286713403393), (99, 31, 105, 1.131737635823451), (99, 31, 106, 1.1950900198226915), (80, 23, 107, 1.1100687372539688), (80, 23, 108, 1.14227747263512), (78, 50, 108, 1.1976104931883995), (30, 22, 109, 1.1502158325718979), (30, 22, 110, 1.1242867486652697), (78, 30, 119, 1.167715948351672), (80, 14, 121, 1.182500658316403), (33, 8, 121, 1.114078866759509), (80, 14, 122, 1.0652798217930777), (33, 8, 122, 1.1290759523306968), (80, 14, 123, 1.1331180772878002), (33, 8, 123, 1.136364645483502), (80, 14, 124, 1.1305774892064866), (80, 14, 125, 1.1640903223309806), (80, 14, 126, 1.1800848119037846), (80, 14, 127, 1.1894706419206562), (80, 14, 128, 1.1946777384840206), (94, 58, 133, 1.1375928443348076), (94, 58, 134, 0.9948156422569866), (94, 58, 135, 1.115078533107978), (94, 58, 136, 1.1614693614472666)]
    data = fdata  
    # 1.1685697876896237
    data=    [(14, 80, 127, 1.1946777384840206), (22, 43, 43, 1.1685697876896237), (26, 74, 147, 1.170340333178619)]

    # K个一对一
    # 0.9296829263843718 
    fdata=[(26, 7, 56, 1.1219214064322411), (33, 0, 67, 1.0177861418976837), (33, 0, 68, 1.0219823062675375), (95, 33, 69, 1.1650001908422307), (33, 0, 69, 1.0183695930273076), (95, 33, 70, 1.1407776853985272), (33, 0, 70, 0.9642271899141848), (95, 33, 71, 1.1959027944464902), (31, 17, 72, 1.1816807046550832), (60, 46, 94, 1.1859491173236372), (41, 2, 96, 1.1320847037995054), (26, 2, 97, 1.0590658516777192), (26, 2, 98, 1.1930075150253585), (48, 2, 102, 1.0663697151652345), (48, 2, 103, 1.1174322885381849), (48, 2, 104, 1.1393424880824403), (34, 20, 105, 1.194259992095568), (22, 10, 124, 1.1794383501589045), (49, 2, 128, 1.1934938343720636), (40, 5, 131, 1.1444724028278792), (40, 5, 132, 1.127477680788904), (40, 5, 133, 1.156510603873974), (40, 5, 134, 0.9908600085575692), (40, 5, 135, 0.9296829263843718), (40, 5, 136, 0.9723034651175174), (40, 5, 137, 1.1174788538262557), (43, 2, 144, 0.9978866094149277), (99, 2, 157, 1.185434254500684)]
    data = fdata

    # 1.0183695930273076 
    data =  [(0, 33, 68, 1.0183695930273076), (1, 41, 34, 1.1614673154372297), (5, 40, 132, 1.156510603873974), (26, 48, 206, 1.1942914156154494), (37, 74, 107, 1.1447774698509756)]

    # 1对多
    # 0.7167124801335585 
    fdata = [(44, 26, 55, 1.059787752667192), (82, 0, 63, 1.1877032353411274), (64, 44, 67, 1.1931104870546942), (64, 44, 68, 1.0317285670004568), (64, 44, 69, 1.1204647311542832), (44, 26, 69, 1.1947226697030457), (64, 44, 70, 1.1256467336110345), (64, 26, 70, 1.1859356112674544), (94, 90, 71, 1.0627953974488131), (64, 44, 71, 1.1260560446844825), (64, 26, 71, 1.1744920986523546), (23, 6, 71, 1.1851757284383846), (94, 90, 72, 0.9709885115799223), (64, 44, 72, 1.1255269442229014), (64, 26, 72, 1.1751908668609574), (94, 90, 73, 1.104972573257081), (92, 72, 73, 1.1967295728771916), (90, 17, 73, 1.1311101623231796), (64, 44, 73, 1.125252535534346), (64, 26, 73, 1.1807820074391744), (94, 90, 74, 1.0896827018157988), (92, 72, 74, 1.192580869386672), (90, 17, 74, 1.1547366675894242), (64, 44, 74, 1.1252275949253903), (64, 26, 74, 1.1800332021432907), (94, 90, 75, 1.0903387289308004), (90, 17, 75, 1.1615365809960743), (64, 44, 75, 1.0948114587302245), (64, 26, 75, 1.1787164387001998), (94, 90, 76, 1.0903134852720797), (64, 44, 76, 1.1795479864039493), (64, 26, 76, 1.1880032586230842), (46, 6, 76, 1.157342655800211), (94, 90, 77, 1.0901130250038042), (64, 26, 77, 1.193922799997908), (45, 23, 77, 1.0137254088753094), (46, 45, 81, 1.147155022508306), (46, 45, 82, 1.0067872824597488), (19, 5, 88, 1.0870196121384375), (9, 5, 99, 1.1282454737374037), (19, 9, 102, 1.192403901986334), (80, 24, 104, 1.0454953254814447), (80, 24, 105, 0.9370261563904777), (42, 26, 105, 1.0896556445714163), (92, 42, 106, 1.107675589987097), (80, 24, 106, 0.8349085069070301), (42, 26, 106, 1.0350671516496557), (92, 42, 107, 1.1697716173237631), (80, 24, 107, 0.7317508127040204), (70, 24, 107, 1.1368849192828496), (42, 26, 107, 1.1179584489749006), (92, 42, 108, 1.1805268256776817), (80, 24, 108, 0.7190850224171919), (70, 24, 108, 1.097425882038637), (42, 26, 108, 1.1171099947216359), (92, 54, 109, 1.1987780400359596), (92, 42, 109, 1.1860840929795502), (80, 24, 109, 0.7167124801335585), (70, 24, 109, 1.1408705704646644), (42, 26, 109, 1.1235449031395215), (92, 54, 110, 1.1918428955524767), (92, 42, 110, 1.187967972480008), (42, 26, 110, 1.1275599922153887), (92, 54, 111, 1.1897169960175045), (92, 42, 111, 1.1878600309473277), (42, 26, 111, 1.128248548663847), (92, 54, 112, 1.193557887294853), (92, 42, 112, 1.187607765958225), (42, 26, 112, 1.0802234131440915), (92, 42, 113, 1.120104979161318), (42, 26, 113, 1.1506920147548392), (92, 42, 114, 1.1542028397079906), (92, 90, 115, 1.1021984811604673), (92, 90, 116, 1.0148520609482647), (92, 90, 117, 1.030414267616757), (90, 89, 117, 1.1310771641145487), (78, 5, 117, 1.1265812348140107), (92, 90, 118, 1.1645694195859104), (90, 89, 118, 1.1470541603659685), (78, 5, 118, 1.182581921172323), (92, 90, 119, 1.1772881206679287), (90, 89, 119, 1.1761310425622953), (78, 5, 119, 1.1959332753341938), (92, 90, 120, 1.177498791079461), (90, 89, 120, 1.1829750489303077), (78, 5, 120, 1.19640200523826), (95, 17, 121, 1.1941848610843397), (92, 90, 121, 0.9768003327198804), (78, 5, 121, 1.1998899423675689), (95, 17, 122, 0.9901653698596338), (92, 90, 122, 1.0160263568751948), (92, 26, 122, 1.1682700133569677), (95, 17, 123, 1.146449834369893), (92, 26, 123, 1.1625645138194305), (95, 17, 124, 1.163748352724386), (46, 41, 124, 1.1378151784849693), (99, 42, 125, 1.1677106240928024), (95, 17, 125, 1.1746818228048428), (78, 46, 125, 1.1965187190501467), (99, 42, 126, 1.0144366147374315), (95, 17, 126, 1.1819993891871527), (36, 17, 126, 1.1950458427346116), (99, 42, 127, 1.1137926767103763), (95, 94, 127, 1.1083383301818333), (95, 17, 127, 1.1821458683159516), (36, 17, 127, 1.1973574773086653), (99, 90, 128, 1.1732696808381269), (99, 42, 128, 1.1221243476569644), (95, 17, 128, 1.1821459818912645), (99, 42, 129, 1.1280908284958986), (99, 42, 130, 1.1347283255584122), (99, 90, 131, 1.1993027009265838), (99, 42, 131, 1.1373301350137153), (52, 6, 131, 1.1789734181857536), (99, 90, 132, 1.173796570171943), (99, 42, 132, 1.1381474548382597), (99, 90, 133, 1.189073329835548), (99, 42, 133, 1.19619368964526), (99, 90, 134, 1.1949786146498471), (99, 90, 135, 1.1974873994827406), (20, 14, 145, 1.198599607101591), (20, 14, 147, 1.1807794551333006), (20, 14, 149, 1.1794733519186817), (20, 14, 151, 1.1832870022631763), (20, 14, 152, 1.168424106548883)]
    data=fdata
    
    # 0.7167124801335585
    data = [(0, 4, 23, 1.1733766495941726), (0, 17, 41, 1.1858630934718197), (14, 20, 150, 1.1832870022631763), (17, 36, 126, 1.1973574773086653), (17, 90, 74, 1.1615365809960743), (17, 95, 127, 1.1821459818912645), (24, 59, 127, 1.0508454698984204), (24, 80, 108, 0.7167124801335585), (26, 42, 110, 1.128248548663847), (26, 64, 76, 1.193922799997908), (42, 92, 111, 1.187607765958225), (42, 99, 131, 1.1381474548382597), (44, 64, 73, 1.1252275949253903), (54, 92, 111, 1.193557887294853), (89, 90, 119, 1.1829750489303077), (90, 92, 119, 1.177498791079461), (90, 94, 75, 1.0903134852720797), (90, 99, 134, 1.1974873994827406)]
    
    fdata =[(33, 0, 98, 1.0671221837691907), (33, 0, 99, 1.055708632504328), (33, 0, 100, 0.9820600721320103), (33, 0, 101, 1.1982487201091923), (33, 0, 102, 1.1377171843960154), (33, 0, 103, 1.1348373464115582), (33, 0, 104, 0.8264651060763443), (33, 0, 105, 0.5539444559946399), (33, 0, 106, 0.48271693652452163), (33, 0, 107, 0.6880153239366471), (33, 0, 108, 1.030353721611667), (41, 0, 113, 1.0309622017531035), (41, 0, 114, 0.9732492734113855), (41, 0, 115, 0.9478948636103609), (41, 0, 116, 0.9558853581520387), (41, 0, 117, 0.952205849412802), (41, 0, 118, 0.6934120815900566), (41, 0, 119, 0.5340803437680244), (41, 0, 120, 0.5260333109622786), (41, 0, 121, 0.673853441803695), (41, 0, 122, 0.7849501832290939), (41, 0, 123, 1.0438797667819077), (59, 29, 127, 1.1882315619566388), (59, 43, 129, 1.1936114658670274), (59, 29, 129, 1.1492000582476833), (22, 0, 129, 1.0005023804199475), (22, 0, 130, 1.0268434920923266), (22, 0, 131, 1.007398865834554), (22, 0, 132, 1.0077566283858912), (56, 0, 161, 1.1604133558682106), (56, 0, 162, 1.1725651543313278), (58, 23, 190, 1.1463336723079678), (58, 23, 192, 1.194502021851177), (51, 6, 212, 1.1822906349287186), (51, 6, 213, 1.186807972062716), (10, 6, 213, 1.0900133525383522), (93, 6, 214, 1.1928538674284999), (51, 6, 214, 0.8620231721287528), (10, 6, 214, 1.128035804546034), (93, 6, 215, 1.096935597357024), (51, 6, 215, 0.6063360752369685), (10, 6, 215, 1.1536786040157345), (51, 6, 216, 0.4615042760966641), (10, 6, 216, 1.1615177049019936), (93, 6, 217, 1.1018314440509538), (51, 6, 217, 0.5089484018231295), (93, 6, 218, 1.0626060138620101), (51, 6, 218, 0.7123616549683617), (93, 6, 219, 1.1161669076056613), (51, 6, 219, 0.9512744255294779), (93, 6, 220, 1.1235837934846105), (51, 6, 220, 1.1637610195953767), (93, 6, 221, 1.1943354199697689), (88, 0, 221, 1.1485637039915224)]

    # 0.4615042760966641 
    data = fdata
    
    # 0.9512075343546834
    data = [(0, 22, 131, 1.0077566283858912), (0, 33, 102, 1.1348373464115582), (0, 41, 116, 0.952205849412802), (6, 10, 215, 1.1615177049019936), (6, 51, 212, 1.186807972062716), (6, 93, 219, 1.1235837934846105), (7, 10, 63, 1.1410282869121104), (32, 58, 186, 0.9512075343546834)]
    
    
    fdata=[(83, 77, 47, 1.1348102878791093), (83, 65, 71, 1.1899192211802445), (83, 65, 73, 1.19661684664794), (83, 65, 74, 1.1961298959447497), (83, 65, 80, 1.1998308433612384), (83, 65, 88, 1.1992765114852486), (83, 65, 90, 1.1998359836039754), (83, 26, 90, 1.1989270706980057), (36, 13, 91, 1.1754613654222208), (83, 65, 92, 1.1997535311122793), (83, 26, 92, 1.1918125077270987), (83, 65, 94, 1.1998162897024047), (83, 26, 94, 1.1976391165495528), (83, 26, 95, 1.1909873508634543), (83, 65, 96, 1.194902971931405), (83, 26, 96, 1.1851576015289347), (83, 65, 98, 1.198783294111224), (83, 26, 98, 1.1962593968673598), (83, 65, 100, 1.198991722472865), (83, 26, 100, 1.18880894598508), (83, 26, 101, 1.1956455863045197), (81, 26, 101, 1.1981102540135729), (83, 65, 102, 1.1970027947632513), (81, 26, 102, 1.1993749484774145), (65, 26, 102, 1.1993804483508836), (83, 65, 104, 1.198628047340272), (83, 65, 106, 1.1994995552521424), (83, 65, 108, 1.1998294160327945), (83, 65, 110, 1.1994270059177967), (83, 26, 110, 1.1885456391096783), (83, 65, 112, 1.1997839398676535), (83, 26, 112, 1.198213656337011), (83, 26, 114, 1.1954760404329412), (67, 52, 115, 1.1773335413665338), (83, 26, 116, 1.1954120548035085), (83, 26, 118, 1.1973449584261633), (28, 2, 118, 1.1364977964535763), (28, 2, 119, 1.0463831214260666), (28, 2, 120, 0.9802133173254021), (83, 26, 121, 1.1930645668372009), (81, 2, 121, 1.144603981597529), (28, 2, 121, 1.0301120538209112), (28, 2, 122, 1.1222302810178026), (83, 26, 123, 1.1941045164609458), (28, 2, 123, 1.1262774896482923), (28, 2, 124, 0.8255748815746515), (28, 2, 125, 0.8558476239942441), (28, 2, 126, 1.1978333376397183), (71, 2, 127, 1.1773118525612056), (83, 26, 128, 1.1973561408512299), (71, 2, 128, 1.1913873145558764), (83, 26, 129, 1.1986900871759785), (68, 2, 136, 1.1146586505129261), (59, 26, 139, 1.0863621788502333), (59, 26, 140, 1.1271737785374205), (59, 26, 141, 1.0478610709582163), (80, 59, 142, 1.1857552549219017), (59, 26, 142, 1.199850293678605), (80, 59, 143, 1.1855722784498806), (59, 26, 143, 1.182058543109356), (88, 23, 144, 1.186481268787829), (83, 81, 144, 1.1999360473612226), (80, 65, 144, 1.1999477204949023), (80, 59, 144, 1.1911868836133106), (80, 23, 144, 1.198495186476832), (59, 26, 144, 1.1906916318583807), (83, 81, 145, 1.179648904742122), (80, 65, 145, 1.1994848858529858), (80, 59, 145, 1.1951348310448482), (80, 23, 145, 1.1970096213696821), (59, 26, 145, 1.1945149745179684), (59, 25, 145, 1.1827869356482428), (88, 23, 146, 1.1697873657333993), (80, 59, 146, 1.1968361526680693), (80, 23, 146, 1.1976388929022908), (59, 26, 146, 1.1969954238067515), (88, 23, 147, 1.1753976162792636), (80, 59, 147, 1.1978569758424515), (80, 23, 147, 1.1945916854114567), (26, 25, 147, 1.1951690264456893), (88, 65, 148, 1.199668961601005), (88, 23, 148, 1.186865842864065), (80, 23, 148, 1.197148183620267), (88, 23, 149, 1.1988387084172625), (83, 71, 150, 1.1697464838582134), (83, 71, 151, 1.1928684598175374), (81, 59, 151, 1.031054837177406), (83, 71, 152, 1.1947506866791784), (81, 59, 152, 1.0906771455564264), (83, 65, 158, 1.1722514358145815), (83, 65, 159, 1.1758611333994367), (71, 65, 159, 1.1942890653460725), (81, 65, 160, 1.1978470298062782), (71, 65, 160, 1.1978367365717528), (83, 65, 161, 1.1898717740585953), (81, 65, 161, 1.1998898584867113), (83, 65, 162, 1.1652565384863094), (88, 65, 163, 1.199498468045793), (83, 65, 163, 1.1672325029549395), (81, 65, 163, 1.1855089958155354), (71, 65, 163, 1.1942494589937622), (88, 65, 164, 1.1982285187703636), (83, 65, 164, 1.1999210373158782), (81, 71, 164, 1.1978703465401), (81, 65, 164, 1.1874768597915666), (81, 59, 164, 1.1973658348904195), (71, 65, 164, 1.197588020297157), (88, 65, 165, 1.1986024308415426), (83, 65, 165, 1.1631378518195528), (81, 65, 165, 1.1893000235675448), (81, 59, 165, 1.199042424893474), (71, 65, 165, 1.196753015894071), (88, 65, 166, 1.1988312648204076), (83, 65, 166, 1.1932030911987561), (81, 71, 166, 1.1999336177239344), (81, 65, 166, 1.191567403128952), (81, 59, 166, 1.1992724392206071), (88, 65, 167, 1.1955951476344737), (83, 65, 167, 1.1914720313178555), (81, 65, 167, 1.1961397999436159), (88, 65, 168, 1.1999880322731433), (88, 23, 168, 1.1990289313422355), (83, 65, 168, 1.194297518942794), (81, 65, 168, 1.1956155554616321), (80, 65, 168, 1.196294848911748), (88, 65, 169, 1.1994743029023938), (83, 65, 169, 1.1943225165051246), (81, 65, 169, 1.1994648267150754), (80, 65, 169, 1.1984224552781317), (88, 65, 170, 1.1992585441215742), (88, 23, 170, 1.1986997289125212), (83, 65, 170, 1.1946200960905429), (81, 65, 170, 1.1988712084461777), (80, 65, 170, 1.1989567461218449), (88, 65, 171, 1.1998176214395067), (83, 65, 171, 1.1957834673975665), (81, 65, 171, 1.1989702630478765), (80, 65, 171, 1.1999614886737955), (88, 65, 172, 1.1985475406005344), (88, 23, 172, 1.1985535050478502), (83, 65, 172, 1.1959062062669208), (81, 65, 172, 1.1972480663623635), (80, 65, 172, 1.1972600231380566), (80, 59, 172, 1.198905673764017), (80, 38, 172, 1.199786756524115), (88, 65, 173, 1.1993898844197377), (88, 23, 173, 1.1991307601666943), (83, 65, 173, 1.1909153565965276), (81, 65, 173, 1.1973874364704769), (80, 65, 173, 1.1972071699557534), (80, 59, 173, 1.19846523334973), (80, 38, 173, 1.1995056326217313), (80, 23, 173, 1.1964880825559148), (63, 23, 173, 1.199352653101236), (88, 65, 174, 1.19842790358059), (88, 23, 174, 1.199066740637388), (83, 65, 174, 1.189495159602242), (81, 65, 174, 1.1984361728431392), (80, 65, 174, 1.197191284791102), (80, 59, 174, 1.1994097688888656), (80, 38, 174, 1.1996086026166919), (71, 65, 174, 1.1995669933456623), (88, 65, 175, 1.1995187916497836), (88, 23, 175, 1.1989564702527549), (83, 65, 175, 1.1898951269874878), (81, 65, 175, 1.1962109210809788), (80, 65, 175, 1.1944130132436424), (80, 59, 175, 1.1994146326319461), (80, 38, 175, 1.1989423594179276), (80, 23, 175, 1.194025818851929), (63, 23, 175, 1.1988246591972735), (88, 65, 176, 1.1960567947274223), (88, 23, 176, 1.1958378775541765), (83, 65, 176, 1.192756975147309), (81, 65, 176, 1.1980914266712785), (80, 65, 176, 1.1987360450418105), (80, 38, 176, 1.1993506245259267), (80, 23, 176, 1.19951902895145), (71, 65, 176, 1.1999943011945031), (88, 65, 177, 1.1980082364114102), (83, 65, 177, 1.193958683245383), (81, 65, 177, 1.1982721989944203), (80, 65, 177, 1.19847715162306), (80, 38, 177, 1.1987702843190033), (71, 65, 177, 1.1971845053573393), (88, 65, 178, 1.1958114249562062), (88, 23, 178, 1.1907975178508787), (83, 65, 178, 1.1986808310546937), (81, 65, 178, 1.1989198174234543), (80, 65, 178, 1.1985421907086624), (71, 65, 178, 1.198568747357869), (88, 65, 179, 1.1983736376580356), (83, 65, 179, 1.198846286523259), (81, 65, 179, 1.1984850272054226), (71, 65, 179, 1.1994060195592289), (88, 65, 180, 1.1991936685098457), (88, 23, 180, 1.199319642008622), (83, 65, 180, 1.1995322677492959), (80, 65, 180, 1.1991618347594744), (71, 65, 180, 1.1996942844449168), (88, 65, 181, 1.1999712049886848), (83, 65, 181, 1.1994992387976646), (80, 65, 181, 1.1994631347701217), (71, 65, 181, 1.1996174448336365), (88, 65, 182, 1.1997896466832758), (88, 23, 182, 1.198867757567444), (83, 65, 182, 1.1983414400273886), (80, 65, 182, 1.1999024413846593), (71, 65, 182, 1.1995301347002696), (83, 65, 183, 1.1982195230932484), (80, 65, 183, 1.1998971959403515), (88, 65, 184, 1.1995792325696175), (88, 23, 184, 1.1992310956830867), (83, 65, 184, 1.1979621341371376), (80, 65, 184, 1.1997613392415258), (83, 65, 185, 1.1984375782312384), (80, 65, 185, 1.1997127042991005), (88, 65, 186, 1.1983523798431501), (88, 23, 186, 1.1987431786146518), (83, 65, 186, 1.1983823031736323), (80, 65, 186, 1.199900496850676), (88, 65, 187, 1.1997493353870743), (83, 65, 187, 1.198971092791622), (80, 65, 187, 1.1999998158589265), (88, 65, 188, 1.1993301848240243), (88, 23, 188, 1.1990008677889743), (83, 65, 188, 1.1988029867105086), (80, 65, 188, 1.1998095282478185), (88, 65, 189, 1.1998512764905638), (83, 65, 189, 1.1988357592875432), (81, 65, 189, 1.199731568706787), (80, 65, 189, 1.199941434157444), (88, 80, 190, 1.1991951768247007), (88, 65, 190, 1.1995327770927027), (88, 23, 190, 1.19860180054306), (83, 65, 190, 1.1995022109583287), (81, 65, 190, 1.1997508319531365), (88, 65, 191, 1.1998548534797935), (83, 65, 191, 1.1987902856956545), (81, 65, 191, 1.1995690011979736), (88, 80, 192, 1.1990844532900828), (88, 65, 192, 1.1994420577516418), (88, 23, 192, 1.1989387638375342), (83, 65, 192, 1.1995245810182473), (81, 65, 192, 1.1999854129004555), (88, 65, 193, 1.19993427657711), (83, 65, 193, 1.1991531108638185), (81, 65, 193, 1.1999778034447748), (88, 80, 194, 1.199290503026782), (88, 65, 194, 1.19952729671812), (88, 23, 194, 1.1984480765980317), (83, 65, 194, 1.1993236541520373), (81, 65, 194, 1.1999430874557828), (80, 65, 194, 1.1996050280010548), (88, 65, 195, 1.1999562579516028), (83, 65, 195, 1.1990882897884518), (81, 65, 195, 1.1999890227643102), (71, 65, 195, 1.1998634728077553), (88, 80, 196, 1.1986148953674216), (88, 65, 196, 1.1996688653029992), (88, 23, 196, 1.1989365776913752), (83, 65, 196, 1.199145338752067), (81, 65, 196, 1.1997208651936597), (80, 65, 196, 1.1997086131025003), (88, 65, 197, 1.199603726941675), (83, 65, 197, 1.1993052253454417), (81, 65, 197, 1.199839468181282), (80, 65, 197, 1.1999438381739393), (71, 65, 197, 1.1998279945522061), (88, 80, 198, 1.1992105365612824), (88, 65, 198, 1.1998659142658028), (88, 23, 198, 1.1984460090170106), (83, 65, 198, 1.1991277712536041), (81, 65, 198, 1.1999514387608317), (80, 65, 198, 1.1998861420496079), (71, 65, 198, 1.1992079951910566), (88, 65, 199, 1.19966556445376), (83, 65, 199, 1.1998303988294698), (81, 65, 199, 1.1999237891123347), (80, 65, 199, 1.1999120385901287), (71, 65, 199, 1.1999504291502119), (88, 80, 200, 1.1990916690960893), (88, 65, 200, 1.1998724356489117), (88, 23, 200, 1.1988780488563537), (83, 65, 200, 1.1995204117243232), (81, 65, 200, 1.1999756591018806), (80, 65, 200, 1.1998218091881332), (88, 65, 201, 1.1999569686135592), (83, 65, 201, 1.1995077330383948), (81, 65, 201, 1.199944771662997), (80, 65, 201, 1.19996801339164), (71, 65, 201, 1.1999244908894966), (88, 80, 202, 1.1994716503164753), (88, 65, 202, 1.199725186678121), (88, 23, 202, 1.1985607171526669), (83, 65, 202, 1.1996843386743683), (81, 65, 202, 1.1999817455973152), (80, 65, 202, 1.1998882763975054), (71, 65, 202, 1.1999115360869166), (83, 65, 203, 1.199593192131236), (81, 65, 203, 1.1999971211252518), (80, 65, 203, 1.199902572942938), (71, 65, 203, 1.1999117111969189), (88, 80, 204, 1.1998704578206887), (88, 23, 204, 1.1988100100012067), (83, 65, 204, 1.199675600957861), (81, 65, 204, 1.1999625199727413), (80, 65, 204, 1.199994923876255), (83, 65, 205, 1.1996874577524457), (81, 65, 205, 1.1999907450763299), (80, 65, 205, 1.199955931366803), (71, 65, 205, 1.1999654659704089), (88, 80, 206, 1.1999381018376782), (88, 23, 206, 1.1985190495699456), (83, 65, 206, 1.199751439513977), (81, 65, 206, 1.1999416153808162), (80, 65, 206, 1.1999260440807207), (83, 65, 207, 1.1997673282531764), (81, 65, 207, 1.1999576118973703), (80, 65, 207, 1.1999998705645074), (71, 65, 207, 1.1999813930211842), (88, 80, 208, 1.1999704704761314), (88, 65, 208, 1.1999549253304667), (88, 23, 208, 1.198902692002728), (83, 65, 208, 1.1998216361059224), (81, 65, 208, 1.1999825266891326), (80, 65, 208, 1.1999623857425883), (83, 65, 209, 1.1998724971723262), (81, 65, 209, 1.1999912349490325), (80, 65, 209, 1.1999708715878057), (88, 65, 210, 1.1999953479494911), (88, 23, 210, 1.1985351768145256), (83, 65, 210, 1.1998976150343952), (81, 65, 210, 1.1999023534219229), (80, 65, 210, 1.1999941119120523), (83, 65, 211, 1.1999579195612777), (81, 65, 211, 1.1998693456121443), (80, 65, 211, 1.1998882792660488), (88, 65, 212, 1.1999464603968024), (88, 23, 212, 1.1988751681665362), (83, 65, 212, 1.199958050668564), (81, 65, 212, 1.1999270170699183), (80, 65, 212, 1.1999536396503148), (83, 65, 213, 1.1999862540544768), (81, 65, 213, 1.1998852850524204), (80, 65, 213, 1.199906771733238), (88, 65, 214, 1.199969908354405), (88, 23, 214, 1.1984247426904096), (83, 65, 214, 1.1999995462150486), (81, 65, 214, 1.1999302360276631), (80, 65, 214, 1.199971211774922), (83, 65, 215, 1.1999794799617083), (81, 65, 215, 1.199896949440295), (80, 65, 215, 1.1999230666811957), (88, 65, 216, 1.1999458152127882), (88, 23, 216, 1.1989706571125904), (83, 65, 216, 1.1999980126433758), (81, 65, 216, 1.19993450138466), (80, 65, 216, 1.1999863320997988), (83, 65, 217, 1.1999500452025804), (81, 65, 217, 1.1999101879436898), (80, 65, 217, 1.1999364913190327), (88, 65, 218, 1.1999321515410817), (88, 23, 218, 1.1983884518013255), (83, 65, 218, 1.1999821465529537), (81, 65, 218, 1.1999419225196364), (80, 65, 218, 1.1999987214453016), (83, 65, 219, 1.1999812153310212), (81, 65, 219, 1.199921792781902), (80, 65, 219, 1.1999520372809174), (88, 65, 220, 1.1999260992975929), (88, 23, 220, 1.198974674797973), (83, 65, 220, 1.1999477267294714), (81, 65, 220, 1.1999497558891254), (80, 65, 220, 1.1999164727078335), (83, 65, 221, 1.1999582838645662), (81, 65, 221, 1.1999321922124297), (80, 65, 221, 1.199981772356952), (88, 65, 222, 1.1999225901486796), (88, 23, 222, 1.1983134848940504), (83, 65, 222, 1.1999759086291566), (81, 65, 222, 1.1999575034141547), (80, 65, 222, 1.1999311728350128), (83, 65, 223, 1.1999820558011762), (81, 65, 223, 1.1999417704498707), (80, 65, 223, 1.1999999031928958), (88, 65, 224, 1.1999284200785418), (88, 23, 224, 1.1990344226347223), (83, 65, 224, 1.1999980599053144), (81, 65, 224, 1.199964166554891), (80, 65, 224, 1.1999456136252136), (83, 65, 225, 1.1999554311162712), (81, 65, 225, 1.1999503501915316), (88, 65, 226, 1.1999245036009185), (88, 23, 226, 1.1982530558802813), (83, 65, 226, 1.1999594290064353), (81, 65, 226, 1.1999691459259025), (80, 65, 226, 1.199959856141816), (83, 65, 227, 1.1999732482679595), (81, 65, 227, 1.199959119441402), (71, 65, 227, 1.1999916396791397), (88, 65, 228, 1.1999283825429432), (88, 23, 228, 1.1990653060291916), (83, 65, 228, 1.1999744370555536), (81, 65, 228, 1.1999736677875836), (80, 65, 228, 1.1999712045522073), (83, 65, 229, 1.1999906916445577), (81, 65, 229, 1.1999668519984517), (71, 65, 229, 1.1999771912512398), (88, 65, 230, 1.19993035652854), (88, 23, 230, 1.1982146859335103), (83, 65, 230, 1.1999865275685868), (81, 65, 230, 1.1999774900253708), (80, 65, 230, 1.199980163732065), (83, 65, 231, 1.1999484463568961), (81, 65, 231, 1.1999735011695467), (71, 65, 231, 1.1999645970429327), (88, 65, 232, 1.1999365823928756), (88, 23, 232, 1.1990301574010827), (83, 65, 232, 1.1999988258560959), (81, 65, 232, 1.1999810293832815), (80, 65, 232, 1.1999867228177343), (83, 65, 233, 1.1999634705180837), (81, 65, 233, 1.199979106381102), (71, 65, 233, 1.199963240047247), (88, 65, 234, 1.1999303072474397), (88, 23, 234, 1.1982178946293764), (83, 65, 234, 1.1999599790692046), (81, 65, 234, 1.1999840174790446), (80, 65, 234, 1.199991538286457), (71, 65, 234, 1.199997222660777), (83, 65, 235, 1.1999740805124561), (81, 65, 235, 1.1999832536673372), (71, 65, 235, 1.199963659173835), (88, 65, 236, 1.199933597963487), (88, 23, 236, 1.1989266527243185), (83, 65, 236, 1.1999666245698515), (81, 65, 236, 1.1999867352781488), (80, 65, 236, 1.199994725694231), (71, 65, 236, 1.1999960256379878), (83, 65, 237, 1.1999818601200647), (81, 65, 237, 1.1999863610481258), (71, 65, 237, 1.1999648847267517), (88, 65, 238, 1.1999379400078602), (88, 23, 238, 1.1982893025877461), (83, 65, 238, 1.199971998299203), (81, 65, 238, 1.199989758587318), (81, 59, 238, 1.19999999997899), (80, 65, 238, 1.1999957707763553), (71, 65, 238, 1.1999973691108146), (83, 65, 239, 1.199987830721428), (81, 65, 239, 1.199988757676423), (71, 65, 239, 1.1999660676839976), (88, 65, 240, 1.1999402995539237), (88, 23, 240, 1.1988234452907378), (83, 65, 240, 1.1999762938790726), (81, 65, 240, 1.1999918624588664), (80, 65, 240, 1.1999962120939105), (71, 65, 240, 1.199998618218931), (88, 23, 241, 1.1998734774408153), (83, 65, 241, 1.1999926556690195), (81, 65, 241, 1.1999906374604044), (71, 65, 241, 1.1999669945768534), (88, 65, 242, 1.1999575057817793), (88, 23, 242, 1.1982543384512032), (83, 65, 242, 1.1999797790077855), (81, 65, 242, 1.1999940846402042), (80, 65, 242, 1.1999955511805651), (83, 65, 243, 1.1999969427334394), (81, 65, 243, 1.1999921188223905), (71, 65, 243, 1.1999667092075488), (88, 65, 244, 1.199948747722496), (88, 23, 244, 1.1987628723488801), (83, 65, 244, 1.1999823804079475), (81, 65, 244, 1.199993416373072), (81, 59, 244, 1.1999999999680886), (80, 65, 244, 1.1999989109373377), (71, 65, 244, 1.1999967925723185), (83, 65, 245, 1.1999768171669816), (81, 65, 245, 1.1999932512913951), (71, 65, 245, 1.1999669855186774), (88, 65, 246, 1.1999459435740343), (88, 23, 246, 1.1984178448737222), (83, 65, 246, 1.1999846432103618), (81, 65, 246, 1.1999943109391393), (81, 59, 246, 1.1999999999559476), (80, 65, 246, 1.199998653771159), (71, 65, 246, 1.1999977102655282), (88, 65, 247, 1.1999848313740002), (88, 23, 247, 1.1999904138435282), (83, 65, 247, 1.1999785685693314), (81, 65, 247, 1.1999941458006478), (71, 65, 247, 1.1999674709182933), (88, 65, 248, 1.199962461075956), (88, 23, 248, 1.1996208320526895), (83, 65, 248, 1.1999864770236923), (81, 65, 248, 1.1999962486130522), (80, 65, 248, 1.1999971438655845), (83, 65, 249, 1.1999773978388886), (81, 65, 249, 1.1999953862572876), (71, 65, 249, 1.1999623206294698), (88, 65, 250, 1.1999367424040557), (88, 23, 250, 1.198354103345087), (83, 65, 250, 1.1999875015936863), (81, 65, 250, 1.199995551759041), (80, 65, 250, 1.199999714267946), (71, 65, 250, 1.1999973979770424), (88, 65, 251, 1.1999983428495646), (88, 23, 251, 1.1999393447114401), (83, 65, 251, 1.199978505986053), (81, 65, 251, 1.1999958156184072), (81, 59, 251, 1.199999999950993), (71, 65, 251, 1.1999629122132138), (88, 65, 252, 1.1999474211412746), (88, 23, 252, 1.1989294404908164), (83, 65, 252, 1.1999885342252579), (81, 65, 252, 1.1999969745222274), (80, 65, 252, 1.199997727404251), (83, 65, 253, 1.199979346328783), (81, 65, 253, 1.1999961775781063), (71, 65, 253, 1.1999633827478495), (88, 65, 254, 1.199941306496534), (88, 23, 254, 1.1990783312360271), (83, 65, 254, 1.199989298561862), (81, 65, 254, 1.1999963837190173), (80, 65, 254, 1.1999997168492298), (71, 65, 254, 1.199997951221244), (88, 65, 255, 1.1998386638455225), (88, 23, 255, 1.1997600000200093), (83, 65, 255, 1.1999802256379588), (81, 65, 255, 1.1999964232053946), (71, 65, 255, 1.1999639733738052), (88, 65, 256, 1.1999625692525586), (88, 23, 256, 1.1983975557717643), (83, 65, 256, 1.1999900319175838), (81, 65, 256, 1.1999974052395004), (80, 65, 256, 1.199998070336453), (83, 65, 257, 1.199979030739202), (81, 65, 257, 1.1999970509074878), (71, 65, 257, 1.1999603506648062), (88, 65, 258, 1.199933765228682), (88, 23, 258, 1.1996201926710672), (83, 65, 258, 1.1999903059247026), (81, 65, 258, 1.199996956786251), (80, 65, 258, 1.1999996216004158), (71, 65, 258, 1.1999984053148858), (88, 65, 259, 1.1999474327915083), (88, 23, 259, 1.1998375380220525), (83, 65, 259, 1.1999798337968046), (81, 65, 259, 1.1999970715075237), (71, 65, 259, 1.199961395060149), (88, 65, 260, 1.199991942509441), (88, 23, 260, 1.198285956452525), (83, 65, 260, 1.19999037943132), (80, 65, 260, 1.1999877114391968), (83, 65, 261, 1.19997312788643), (71, 65, 261, 1.1999596180547991), (88, 65, 262, 1.1999369662757435), (88, 23, 262, 1.1998095716659307), (83, 65, 262, 1.1999898982810928), (81, 59, 262, 1.1999999999734172), (80, 65, 262, 1.1999993517828378), (71, 65, 262, 1.1999991663114813), (88, 65, 263, 1.1998321224362032), (88, 23, 263, 1.1999201900265197), (83, 65, 263, 1.1999748644829877), (71, 65, 263, 1.1999498540373397), (88, 65, 264, 1.1999934441639815), (88, 23, 264, 1.1998139795422518), (83, 65, 264, 1.1999929537992882), (80, 65, 264, 1.1999615038128129), (88, 65, 265, 1.1999976718738523), (88, 23, 265, 1.199971872004242), (83, 65, 265, 1.1999621009512986), (71, 65, 265, 1.1999819783209975), (88, 65, 266, 1.1999775572029643), (88, 23, 266, 1.1980394862632804), (83, 65, 266, 1.1999912390803444), (88, 65, 267, 1.1999466038084277), (83, 65, 267, 1.1999584541257533), (71, 65, 267, 1.199918102997243), (88, 65, 268, 1.199942118116112), (88, 23, 268, 1.1991433420233897), (83, 65, 268, 1.1999855836173425), (80, 65, 268, 1.1999996359009755), (71, 65, 268, 1.1999988096851555), (88, 65, 269, 1.1999904458580481), (88, 23, 269, 1.199872189758049), (83, 65, 269, 1.1999616720269215), (71, 65, 269, 1.199924171039803), (88, 65, 270, 1.1999427323945677), (88, 23, 270, 1.1993369074844948), (83, 65, 270, 1.1999851902403083), (80, 65, 270, 1.1999975450225882), (71, 65, 270, 1.1999979772262928), (88, 65, 271, 1.1999909787452172), (83, 65, 271, 1.1999633033770556), (71, 65, 271, 1.1999281826532995), (88, 65, 272, 1.199957236955239), (88, 23, 272, 1.1997673666990665), (83, 65, 272, 1.1999853492381476), (80, 65, 272, 1.199997817022922), (71, 65, 272, 1.199997234912162), (88, 65, 273, 1.1998304285555137), (88, 23, 273, 1.199768564515562), (83, 65, 273, 1.199965851172233), (71, 65, 273, 1.19993347505203), (88, 65, 274, 1.199993432080312), (88, 23, 274, 1.1980595481002918), (83, 65, 274, 1.1998616763641985), (80, 65, 274, 1.1998749541125842), (88, 65, 275, 1.1999823132309833), (88, 23, 275, 1.1998211656413078), (83, 65, 275, 1.1999438164246379), (81, 59, 275, 1.1999999999912898), (88, 65, 276, 1.1998956834472172), (88, 23, 276, 1.1985389666223805), (83, 65, 276, 1.1999964283152038), (88, 65, 277, 1.199970445757985), (83, 65, 277, 1.1999533168954108), (71, 65, 277, 1.1999361313048034), (88, 65, 278, 1.1998949568699107), (88, 23, 278, 1.1993230179360188), (83, 65, 278, 1.1999889978969749), (88, 65, 279, 1.199723894750168), (88, 23, 279, 1.199167483255559), (83, 65, 279, 1.1999557542306019), (80, 65, 279, 1.1999978351972578), (71, 65, 279, 1.199912490769409), (88, 65, 280, 1.1999376491761817), (88, 23, 280, 1.1993602200545221), (83, 65, 280, 1.1998170386339584), (81, 59, 280, 1.1999999998596038), (80, 65, 280, 1.1998854555692327), (88, 65, 281, 1.199941216989391), (88, 23, 281, 1.1997232945274876), (83, 65, 281, 1.1999260199782946), (80, 65, 281, 1.1999919784204707), (88, 65, 282, 1.1998591610706644), (88, 23, 282, 1.1987376277841588), (83, 65, 282, 1.1999970539042388), (88, 65, 283, 1.1999980379107487), (88, 23, 283, 1.1999809880928558), (83, 65, 283, 1.1999381283983321), (71, 65, 283, 1.199924197591422), (88, 65, 284, 1.1998920474250763), (88, 23, 284, 1.1990549460985405), (83, 65, 284, 1.199996831797056), (88, 65, 285, 1.1999964261613258), (83, 65, 285, 1.1999418373503372), (80, 65, 285, 1.1999798011333422), (71, 65, 285, 1.1998850328960218), (88, 65, 286, 1.1999607309208273), (88, 23, 286, 1.1992179067401525), (83, 65, 286, 1.1999763158019445), (81, 59, 286, 1.1999999998217874), (80, 65, 286, 1.1999981690998869), (71, 65, 286, 1.1999981501000867), (88, 65, 287, 1.1999908032140019), (88, 23, 287, 1.1999792481683118), (83, 65, 287, 1.1999457581803084), (81, 59, 287, 1.1999999999272815), (80, 65, 287, 1.1999694370075764), (71, 65, 287, 1.199892819751646), (88, 65, 288, 1.199981964552346), (88, 23, 288, 1.1994473597952144), (83, 65, 288, 1.1999758732697063), (80, 65, 288, 1.1999955503826742), (71, 65, 288, 1.1999959492831913), (88, 65, 289, 1.1999792136268708), (88, 23, 289, 1.1998972440059674), (83, 65, 289, 1.1999514334129775), (80, 65, 289, 1.1999702493300364), (71, 65, 289, 1.1999036222089539), (88, 65, 290, 1.1998754851095879), (88, 23, 290, 1.1992715024202474), (83, 65, 290, 1.1999769642675056), (81, 59, 290, 1.1999999998791346), (80, 65, 290, 1.1999952169638293), (71, 65, 290, 1.1999959401430926), (88, 65, 291, 1.1999718218180258), (88, 23, 291, 1.1998545771719549), (83, 65, 291, 1.1999560704574128), (80, 65, 291, 1.1999741677575473), (71, 65, 291, 1.1999114413953054), (88, 65, 292, 1.199892888388672), (88, 23, 292, 1.1993650665011577), (83, 65, 292, 1.1999824070717193), (80, 65, 292, 1.199995474140711), (71, 65, 292, 1.1999990774534177), (88, 65, 293, 1.1999780433455343), (88, 23, 293, 1.1998183277233305), (83, 65, 293, 1.1999590724423366), (80, 65, 293, 1.199978204462164), (71, 65, 293, 1.1999171686847534), (88, 65, 294, 1.1999166840816868), (88, 23, 294, 1.1992295887801228), (83, 65, 294, 1.1999867751318627), (81, 59, 294, 1.1999999999418638), (80, 65, 294, 1.1999956078933849), (88, 65, 295, 1.1999804529093971), (88, 23, 295, 1.1998135229404192), (83, 65, 295, 1.1999608396851262), (81, 59, 295, 1.1999999998602424), (80, 65, 295, 1.1999817159878894), (71, 65, 295, 1.1999208208274041), (88, 65, 296, 1.1999298317746465), (88, 23, 296, 1.1993098820837442), (83, 65, 296, 1.199988223086726), (81, 59, 296, 1.1999999999171334), (80, 65, 296, 1.1999957129756387), (88, 65, 297, 1.1999776782270837), (88, 23, 297, 1.1998007956042642), (83, 65, 297, 1.1999618936180574), (80, 65, 297, 1.1999841415704646), (71, 65, 297, 1.1999227319669081), (88, 65, 298, 1.1999368714794783), (88, 23, 298, 1.1992169792702243), (83, 65, 298, 1.1999898830183962), (81, 59, 298, 1.19999999993623), (80, 65, 298, 1.1999953206239888), (88, 65, 299, 1.1999740213159296), (88, 23, 299, 1.1997792614810774), (83, 65, 299, 1.199962154611481), (80, 65, 299, 1.1999862005038284), (71, 65, 299, 1.1999235111208268), (88, 65, 300, 1.1999390245753838), (88, 23, 300, 1.1993027541972225), (83, 65, 300, 1.1999904314491119), (81, 59, 300, 1.19999999999493), (80, 65, 300, 1.1999953144044269), (88, 65, 301, 1.1999934040473232), (88, 23, 301, 1.1997697577087911), (83, 65, 301, 1.1999618744852754), (80, 65, 301, 1.1999873505044099), (71, 65, 301, 1.1999231359003701), (88, 65, 302, 1.1999412477999405), (88, 23, 302, 1.1992251877536708), (83, 65, 302, 1.199991376767449), (80, 65, 302, 1.19999475704545), (88, 65, 303, 1.1999858034507194), (88, 23, 303, 1.1997635487899714), (83, 65, 303, 1.1999608693311052), (81, 59, 303, 1.1999999999986015), (80, 65, 303, 1.1999882708830985), (71, 65, 303, 1.1999219700476367), (88, 65, 304, 1.1999369430921438), (88, 23, 304, 1.1993047702996098), (83, 65, 304, 1.199991756219108), (81, 59, 304, 1.1999999998264956), (80, 65, 304, 1.199994295321499), (88, 65, 305, 1.199967255891129), (88, 23, 305, 1.1997239098076309), (83, 65, 305, 1.1999594275793317), (80, 65, 305, 1.1999882688558456), (71, 65, 305, 1.1999199060049317), (88, 65, 306, 1.1999269493114666), (88, 23, 306, 1.1992243988212006), (83, 65, 306, 1.199991822459788), (80, 65, 306, 1.1999941713749072), (88, 65, 307, 1.1999898016087263), (88, 23, 307, 1.1997046417896744), (83, 65, 307, 1.1999574923652907), (81, 59, 307, 1.1999999999907707), (80, 65, 307, 1.1999876099926567), (71, 65, 307, 1.199917096599602), (88, 65, 308, 1.1999218355096655), (88, 23, 308, 1.19926078190551), (83, 65, 308, 1.199991694164262), (80, 65, 308, 1.1999932117132253), (88, 65, 309, 1.1999835203424722), (88, 23, 309, 1.1997202390304051), (83, 65, 309, 1.1999552805513691), (81, 59, 309, 1.19999999981285), (80, 65, 309, 1.1999862498804124), (71, 65, 309, 1.1999139371032284), (88, 65, 310, 1.199909920419885), (88, 23, 310, 1.1992174035038092), (83, 65, 310, 1.1999907630821436), (80, 65, 310, 1.1999927137855915), (88, 65, 311, 1.1999937014413515), (88, 23, 311, 1.1996996640720203), (83, 65, 311, 1.1999528669284412), (81, 59, 311, 1.1999999999591884), (80, 65, 311, 1.1999840416888627), (71, 65, 311, 1.1999104222093424), (88, 65, 312, 1.199897862516063), (88, 23, 312, 1.199204691833108), (83, 65, 312, 1.1999903587370897), (81, 59, 312, 1.1999999999223319), (80, 65, 312, 1.1999912906351855), (88, 65, 313, 1.1999595211772625), (88, 23, 313, 1.1997143008193984), (83, 65, 313, 1.1999508440757565), (80, 65, 313, 1.1999813744100993), (71, 65, 313, 1.1999070680156645), (88, 65, 314, 1.1999193099666938), (88, 23, 314, 1.1989806010730932), (83, 65, 314, 1.1999915673000376), (80, 65, 314, 1.1999918897847937), (88, 65, 315, 1.1998574757595126), (88, 23, 315, 1.199689982124926), (83, 65, 315, 1.199952196034761), (80, 65, 315, 1.1999777036827852), (71, 65, 315, 1.1999102188438924), (88, 65, 316, 1.1999197381887128), (88, 23, 316, 1.1994561796228433), (83, 65, 316, 1.1999917744248885), (80, 65, 316, 1.1999920642331383), (88, 65, 317, 1.1999034403685969), (88, 23, 317, 1.1997653158928394), (83, 65, 317, 1.19995039633594), (80, 65, 317, 1.1999750907356477), (71, 65, 317, 1.1999068129767005), (88, 65, 318, 1.1998973952442324), (88, 23, 318, 1.1988999773197953), (83, 65, 318, 1.1999879586216036), (81, 59, 318, 1.1999999999811375), (80, 65, 318, 1.1999662695307092), (88, 65, 319, 1.1999518716873319), (88, 23, 319, 1.1996866531396335), (83, 65, 319, 1.1999489871556912), (81, 59, 319, 1.199999999890829), (80, 65, 319, 1.1999717693428191), (71, 65, 319, 1.1999053911437856), (88, 65, 320, 1.1998910331004402), (88, 23, 320, 1.1994151526785148), (83, 65, 320, 1.199990023142512), (80, 65, 320, 1.1999907554539162), (88, 65, 321, 1.1999779910508566), (88, 23, 321, 1.1997322820453815), (83, 65, 321, 1.1999483289409507), (80, 65, 321, 1.199970035767106), (71, 65, 321, 1.1999038743810055), (88, 65, 322, 1.1998940040087605), (88, 23, 322, 1.199135435325148), (83, 65, 322, 1.1999907585071417), (81, 59, 322, 1.1999999999539666), (80, 65, 322, 1.1999894093403494), (88, 65, 323, 1.1999487265758262), (88, 23, 323, 1.1996242573211573), (83, 65, 323, 1.1999454784679946), (81, 59, 323, 1.1999999999339694), (80, 65, 323, 1.1999681331836687), (71, 65, 323, 1.1999001745015911), (88, 65, 324, 1.19989111863268), (88, 23, 324, 1.199262637850123), (83, 65, 324, 1.1999916875253192), (80, 65, 324, 1.1999892708488233), (88, 65, 325, 1.1999860504565196), (88, 23, 325, 1.1996538459489765), (83, 65, 325, 1.199941830713252), (80, 65, 325, 1.1999639434619969), (71, 65, 325, 1.1998950501699064), (88, 65, 326, 1.199883184956616), (88, 23, 326, 1.1990777659243073), (83, 65, 326, 1.1999923308343685), (80, 65, 326, 1.1999890768785473), (88, 65, 327, 1.199998889055885), (88, 23, 327, 1.199625581266703), (83, 65, 327, 1.1999387893027489), (80, 65, 327, 1.1999600774502641), (71, 65, 327, 1.1998894945448677), (88, 65, 328, 1.1998521271714857), (88, 23, 328, 1.1991926013398058), (83, 65, 328, 1.199991051846303), (80, 65, 328, 1.199988341165473), (88, 65, 329, 1.1999348737960782), (88, 23, 329, 1.1996352352837898), (83, 65, 329, 1.1999357760333256), (80, 65, 329, 1.1999559140630236), (71, 65, 329, 1.1998843377917847), (88, 65, 330, 1.1999927993647084), (88, 23, 330, 1.1990571173878772), (83, 65, 330, 1.199991061808263), (80, 65, 330, 1.1999886628763468), (88, 65, 331, 1.1999793595902277), (88, 23, 331, 1.1996411968653558), (83, 65, 331, 1.1999338424938708), (80, 65, 331, 1.1999517478865376), (71, 65, 331, 1.1998801521588789), (88, 65, 332, 1.1999843437994182), (88, 23, 332, 1.1991356355042995), (83, 65, 332, 1.1999918603811297), (81, 59, 332, 1.1999999998469173), (80, 65, 332, 1.1999884171155706), (88, 65, 333, 1.1999985276354186), (88, 23, 333, 1.1996283269211725), (83, 65, 333, 1.1999322680436648), (80, 65, 333, 1.1999474306464533), (71, 65, 333, 1.1998774328475654), (88, 65, 334, 1.1999717642767076), (88, 23, 334, 1.199159857962138), (83, 65, 334, 1.1999883685476394), (81, 59, 334, 1.1999999998835023), (80, 65, 334, 1.1999982513077059), (88, 65, 335, 1.1999203673270442), (88, 23, 335, 1.1996492755839576), (83, 65, 335, 1.1999319109217241), (80, 65, 335, 1.1999461923713528), (71, 65, 335, 1.1998771601080467), (88, 65, 336, 1.199985691652259), (88, 23, 336, 1.1991666536968957), (83, 65, 336, 1.1999869187174956), (80, 65, 336, 1.1999999077748118), (88, 65, 337, 1.199998552364696), (88, 23, 337, 1.1996416927032358), (83, 65, 337, 1.1999337685677185), (80, 65, 337, 1.1999409958141052), (71, 65, 337, 1.1998808989050782), (88, 65, 338, 1.1999921030529295), (88, 23, 338, 1.1991460447263393), (83, 65, 338, 1.1999878769825139), (81, 59, 338, 1.1999999999143065), (88, 65, 339, 1.1999129404960245), (88, 23, 339, 1.1996329830034806), (83, 65, 339, 1.1999355168232329), (80, 65, 339, 1.1999422096856578), (71, 65, 339, 1.1998842488110382), (88, 65, 340, 1.1998429175233776), (88, 23, 340, 1.1991713457287725), (83, 65, 340, 1.1999882841945262), (88, 65, 341, 1.1999236571518044), (88, 23, 341, 1.1996301974646633), (83, 65, 341, 1.1999383987872592), (81, 59, 341, 1.1999999998908342), (80, 65, 341, 1.1999372623879963), (71, 65, 341, 1.1998874056732423), (88, 65, 342, 1.1998544273460303), (88, 23, 342, 1.1991275230908613), (83, 65, 342, 1.1999893244289284), (88, 65, 343, 1.1999986007133732), (88, 23, 343, 1.1996171678517715), (83, 65, 343, 1.1999391323882358), (80, 65, 343, 1.199936189014428), (71, 65, 343, 1.1998893066700433), (88, 65, 344, 1.199849538939965), (88, 23, 344, 1.1991350568263004), (83, 65, 344, 1.1999898980691768), (88, 65, 345, 1.1999893886859792), (88, 23, 345, 1.1996162742602852), (83, 65, 345, 1.1999399030530022), (80, 65, 345, 1.1999354234667907), (71, 65, 345, 1.199890335881895), (88, 65, 346, 1.199844226621324), (88, 23, 346, 1.1991207848409973), (83, 65, 346, 1.1999898143781877), (88, 65, 347, 1.1999895315622897), (88, 23, 347, 1.199613688606286), (83, 65, 347, 1.1999405117511692), (81, 59, 347, 1.1999999998109463), (80, 65, 347, 1.1999370178143847), (71, 65, 347, 1.199891051755105), (88, 65, 348, 1.199846137701843), (88, 23, 348, 1.1991312107510763), (83, 65, 348, 1.1999900959758276), (88, 65, 349, 1.1999895837896666), (88, 23, 349, 1.1996110683170207), (83, 65, 349, 1.1999412902537052), (80, 65, 349, 1.1999378597008676), (71, 65, 349, 1.1998916811772284), (88, 65, 350, 1.1998624847661212), (88, 23, 350, 1.199109443121637), (83, 65, 350, 1.1999924832138424), (81, 59, 350, 1.1999999999608797), (88, 65, 351, 1.1999020947603707), (88, 23, 351, 1.1996070974090949), (83, 65, 351, 1.1999418398158403), (81, 59, 351, 1.1999999999092983), (80, 65, 351, 1.1999373932549091), (71, 65, 351, 1.1998919847828995), (88, 65, 352, 1.1998664404734107), (88, 23, 352, 1.1991147237690414), (83, 65, 352, 1.199993621583576), (81, 59, 352, 1.1999999998868802), (88, 65, 353, 1.199999458275737), (88, 23, 353, 1.199606087683552), (83, 65, 353, 1.1999422205975547), (80, 65, 353, 1.1999368639630306), (71, 65, 353, 1.1998919326132524), (88, 65, 354, 1.199847098945538), (88, 23, 354, 1.1991041042847168), (83, 65, 354, 1.1999932372332647), (88, 65, 355, 1.1999991526203875), (88, 23, 355, 1.1996056813546374), (83, 65, 355, 1.1999428056472572), (80, 65, 355, 1.19993597251423), (71, 65, 355, 1.199891835134152), (88, 65, 356, 1.1998637409247728), (88, 23, 356, 1.198999351745926), (83, 65, 356, 1.199995281412213), (88, 65, 357, 1.1999025356525839), (88, 23, 357, 1.199598488363293), (83, 65, 357, 1.199942593933819), (80, 65, 357, 1.1999358736473147), (71, 65, 357, 1.1998907905008616), (88, 65, 358, 1.1998622901531546), (88, 23, 358, 1.1991059368668735), (83, 65, 358, 1.1999962374471422), (81, 59, 358, 1.1999999999354727), (88, 65, 359, 1.1998997864429721), (88, 23, 359, 1.199600561721129), (83, 65, 359, 1.1999419485792517), (81, 59, 359, 1.19999999989184), (80, 65, 359, 1.1999355698244636), (71, 65, 359, 1.1998901553029635), (88, 65, 360, 1.1998646203467775), (88, 23, 360, 1.1990472123514526), (83, 65, 360, 1.1999949253470794), (81, 59, 360, 1.1999999998986215), (88, 65, 361, 1.1999908337519525), (88, 23, 361, 1.1995844035550525), (83, 65, 361, 1.1999414296078568), (80, 65, 361, 1.1999382448769822), (71, 65, 361, 1.1998896529884964), (88, 65, 362, 1.1998648327443904), (88, 23, 362, 1.1990557231510812), (83, 65, 362, 1.199996681569996), (88, 65, 363, 1.1999876608146607), (88, 23, 363, 1.1995794840968872), (83, 65, 363, 1.1999411440316292), (81, 59, 363, 1.1999999998941924), (80, 65, 363, 1.1999378529601357), (71, 65, 363, 1.199889339040303), (88, 65, 364, 1.1998643944400438), (88, 23, 364, 1.1990469251622677), (83, 65, 364, 1.1999968048990353), (88, 65, 365, 1.1999835249206194), (88, 23, 365, 1.1995750144641386), (83, 65, 365, 1.1999410106291482), (80, 65, 365, 1.1999348951844102), (71, 65, 365, 1.1998885860892174), (88, 65, 366, 1.1998601385641394), (88, 23, 366, 1.1990298613848924), (83, 65, 366, 1.19999807564079), (88, 65, 367, 1.1999644339335833), (88, 23, 367, 1.1995766048314607), (83, 65, 367, 1.1999400805489964), (80, 65, 367, 1.1999369962473099), (71, 65, 367, 1.1998877025493386), (88, 65, 368, 1.1998573372662265), (88, 23, 368, 1.1990466900284449), (83, 65, 368, 1.1999978150882), (88, 65, 369, 1.199984685455664), (88, 23, 369, 1.1995725556976875), (83, 65, 369, 1.1999397521088644), (81, 59, 369, 1.19999999989426), (80, 65, 369, 1.1999368208526837), (71, 65, 369, 1.1998867136432725), (88, 65, 370, 1.199855580987884), (88, 23, 370, 1.199029798136094), (83, 65, 370, 1.199998582765336), (88, 65, 371, 1.1999974657857595), (88, 23, 371, 1.1995683763437255), (83, 65, 371, 1.1999393110534768), (81, 59, 371, 1.1999999999811766), (80, 65, 371, 1.1999357461407791), (71, 65, 371, 1.1998859975183982), (88, 65, 372, 1.1998570449710755), (88, 23, 372, 1.1990347805521993), (83, 65, 372, 1.1999988781675173), (88, 65, 373, 1.1999972644211043), (88, 23, 373, 1.199567261875347), (83, 65, 373, 1.1999390506894778), (80, 65, 373, 1.1999347386007189), (71, 65, 373, 1.1998854465277864), (88, 65, 374, 1.1998567227400847), (88, 23, 374, 1.1990303967156233), (83, 65, 374, 1.1999989579091521), (88, 65, 375, 1.1999942985609071), (88, 23, 375, 1.1995630610707302), (83, 65, 375, 1.1999389481970553), (80, 65, 375, 1.1999346979465326), (71, 65, 375, 1.1998850164445083), (88, 65, 376, 1.1998544519931371), (88, 23, 376, 1.1990219478471553), (83, 65, 376, 1.19999949890522), (81, 59, 376, 1.1999999999755504), (88, 65, 377, 1.1999977668304793), (88, 23, 377, 1.199562470376654), (83, 65, 377, 1.199938626980315), (81, 59, 377, 1.1999999999729536), (80, 65, 377, 1.1999343244429155), (71, 65, 377, 1.1998846054780241), (88, 65, 378, 1.1998559668293847), (88, 23, 378, 1.1990323064708708), (83, 65, 378, 1.1999996653391938), (81, 59, 378, 1.1999999999928435), (88, 65, 379, 1.1999674636908215), (88, 23, 379, 1.1995971321574783), (83, 65, 379, 1.199938266096635), (81, 59, 379, 1.1999999999612365), (80, 65, 379, 1.1999325287589255), (71, 65, 379, 1.1998842163161558), (88, 65, 380, 1.1998792442324884), (88, 23, 380, 1.1989366269021176), (83, 65, 380, 1.1999158570187742), (80, 65, 380, 1.199980037805783), (88, 65, 381, 1.1999362633209187), (88, 23, 381, 1.1995901976956351), (83, 65, 381, 1.1999455752906838), (80, 65, 381, 1.1999432830267673), (71, 65, 381, 1.199901326344696), (88, 65, 382, 1.1999019862417437), (88, 23, 382, 1.199058063311726), (83, 65, 382, 1.1999349056362147), (80, 65, 382, 1.1999933200925572), (88, 65, 383, 1.1998976618400696), (88, 23, 383, 1.1996004478173519), (83, 65, 383, 1.1999443312993612), (80, 65, 383, 1.1999339402786027), (71, 65, 383, 1.1998912628120626), (88, 65, 384, 1.199870044150573), (88, 23, 384, 1.1990483845290762), (83, 65, 384, 1.1999301494082397), (88, 65, 385, 1.199983986630122), (88, 23, 385, 1.199564846801671), (83, 65, 385, 1.1999430796427437), (80, 65, 385, 1.1999345428457302), (71, 65, 385, 1.1998882912488946), (88, 65, 386, 1.1998318484129344), (88, 23, 386, 1.199138915243191), (83, 65, 386, 1.1999191325563994), (80, 65, 386, 1.1999951071692787), (88, 65, 387, 1.19998670056785), (88, 23, 387, 1.1995927641881468), (83, 65, 387, 1.1999423324162666), (80, 65, 387, 1.1999328312139599), (71, 65, 387, 1.199889158092927), (88, 65, 388, 1.199865509493263), (88, 23, 388, 1.1991639201641842), (83, 65, 388, 1.1999277981207686), (80, 65, 388, 1.199996667146615), (88, 65, 389, 1.199865639371652), (88, 23, 389, 1.199610981908617), (83, 65, 389, 1.1999408643996827), (80, 65, 389, 1.1999281073794732), (71, 65, 389, 1.1998832828468777), (88, 65, 390, 1.199846544162065), (88, 23, 390, 1.1990633589380464), (83, 65, 390, 1.1999219307022435), (80, 65, 390, 1.1999877774754548), (88, 65, 391, 1.1999621959140554), (88, 23, 391, 1.1996127422807916), (83, 65, 391, 1.1999405057521175), (80, 65, 391, 1.1999295688909468), (71, 65, 391, 1.1998840065581484), (88, 65, 392, 1.19988543577171), (88, 23, 392, 1.1991199515421096), (83, 65, 392, 1.1999258377955142), (80, 65, 392, 1.1999891003030463), (88, 65, 393, 1.1998131289532108), (88, 23, 393, 1.1996266476716388), (83, 65, 393, 1.199943481716736), (80, 65, 393, 1.199934888526891), (71, 65, 393, 1.1998892514647224), (88, 65, 394, 1.1998933788699802), (88, 23, 394, 1.1990034015600428), (83, 65, 394, 1.199932452536139), (88, 65, 395, 1.1998322188050905), (88, 23, 395, 1.1996993664954125), (83, 65, 395, 1.1999478497468006), (80, 65, 395, 1.1999388404802325), (71, 65, 395, 1.1998962453565503), (88, 65, 396, 1.1999103155394122), (88, 23, 396, 1.1992151972185507), (83, 65, 396, 1.199936814448514), (88, 65, 397, 1.1998511248011035), (88, 23, 397, 1.1996643034418357), (83, 65, 397, 1.1999524235237518), (80, 65, 397, 1.199943138334064), (71, 65, 397, 1.1999037683457727), (88, 65, 398, 1.1999347626942922), (88, 23, 398, 1.1991996141397425), (83, 65, 398, 1.199937352856476), (88, 65, 399, 1.1998301517048093), (88, 23, 399, 1.1996455919061055), (83, 65, 399, 1.1999566648828912), (81, 59, 399, 1.1999999999811384), (80, 65, 399, 1.1999484027506129), (71, 65, 399, 1.1999116711862912), (88, 65, 400, 1.1999358056484806), (88, 23, 400, 1.1990208188990352), (83, 65, 400, 1.1999388144992194), (88, 65, 401, 1.1998297527899349), (88, 23, 401, 1.1996061107837128), (83, 65, 401, 1.1999589240558157), (80, 65, 401, 1.1999499798074702), (71, 65, 401, 1.1999142381072485), (88, 65, 402, 1.1999343494240107), (88, 23, 402, 1.199155446547449), (83, 65, 402, 1.199939479087536), (88, 65, 403, 1.1998232891631158), (88, 23, 403, 1.1996140489657132), (83, 65, 403, 1.1999601161642315), (80, 65, 403, 1.19995152781682), (71, 65, 403, 1.1999146191889036), (88, 65, 404, 1.199932982785977), (88, 23, 404, 1.199012993146949), (83, 65, 404, 1.1999372544392086), (88, 65, 405, 1.199813723399777), (88, 23, 405, 1.1995829439741192), (83, 65, 405, 1.199959572291421), (80, 65, 405, 1.1999504864734276), (71, 65, 405, 1.199912612204875), (88, 65, 406, 1.1999351119422401), (88, 23, 406, 1.1991236744014466), (83, 65, 406, 1.199935855321446), (88, 65, 407, 1.199974249990746), (88, 23, 407, 1.1996017345196428), (83, 65, 407, 1.199960543703411), (80, 65, 407, 1.1999525549979309), (71, 65, 407, 1.1999147620431607), (88, 65, 408, 1.1999277013842466), (88, 23, 408, 1.198927674090788), (83, 65, 408, 1.1999367503362621), (80, 65, 408, 1.1999953391748044), (88, 65, 409, 1.1997958602850591), (88, 23, 409, 1.1995535070356094), (83, 65, 409, 1.1999590083474616), (80, 65, 409, 1.1999474683334204), (71, 65, 409, 1.1999095625756442), (88, 65, 410, 1.1999346660062975), (88, 23, 410, 1.1992022445480357), (83, 65, 410, 1.1999320989909017), (88, 65, 411, 1.199989214331521), (88, 23, 411, 1.1996303464090883), (83, 65, 411, 1.199958986705639), (80, 65, 411, 1.199949642636089), (71, 65, 411, 1.1999097342192537), (88, 65, 412, 1.1999080165925038), (88, 23, 412, 1.1989107672531845), (83, 65, 412, 1.1999319855354298), (80, 65, 412, 1.19999288879168), (88, 65, 413, 1.1997864796144693), (88, 23, 413, 1.1995405764844191), (83, 65, 413, 1.1999565392668126), (80, 65, 413, 1.199943686135312), (71, 65, 413, 1.19990415183768), (88, 65, 414, 1.1999368918468294), (88, 23, 414, 1.199237336033694), (83, 65, 414, 1.1999339004505054), (88, 65, 415, 1.1999404072890478), (88, 23, 415, 1.1996512757133142), (83, 65, 415, 1.1999565813671025), (80, 65, 415, 1.1999453939818157), (71, 65, 415, 1.1999039326751784), (88, 65, 416, 1.1999122775354856), (88, 23, 416, 1.1990711448179758), (83, 65, 416, 1.1999293482323836), (81, 59, 416, 1.199999999914635), (80, 65, 416, 1.1999949914200152), (88, 65, 417, 1.1997988030265827), (88, 23, 417, 1.1995943112874585), (83, 65, 417, 1.1999543767413157), (80, 65, 417, 1.1999415587277515), (71, 65, 417, 1.1999000960347759), (88, 65, 418, 1.199929793926492), (88, 23, 418, 1.199029599755079), (83, 65, 418, 1.1999268997725412), (88, 65, 419, 1.1998753594062161), (88, 23, 419, 1.199555268428329), (83, 65, 419, 1.199954515984396), (80, 65, 419, 1.199944841140287), (71, 65, 419, 1.1999015346596222), (88, 65, 420, 1.199986204789718), (88, 23, 420, 1.1993259897182174), (83, 65, 420, 1.199923355104598), (80, 65, 420, 1.1999815114713792), (88, 65, 421, 1.1998095717368507), (88, 23, 421, 1.1997011473910453), (83, 65, 421, 1.1999519368531482), (80, 65, 421, 1.1999374406533554), (71, 65, 421, 1.199989591190568), (88, 65, 422, 1.1999631038658995), (88, 23, 422, 1.1989180607562813), (83, 65, 422, 1.1999059234018148), (80, 65, 422, 1.1999689483582385), (88, 65, 423, 1.199876445602647), (88, 23, 423, 1.1995310402063377), (83, 65, 423, 1.1999476260971458), (80, 65, 423, 1.199935292142357), (71, 65, 423, 1.1999119125173094), (88, 65, 424, 1.1998757286761264), (88, 23, 424, 1.1991179427335295), (83, 65, 424, 1.1999221115716225), (80, 65, 424, 1.1999865371188045), (88, 65, 425, 1.199772024875594), (88, 23, 425, 1.1996098833125646), (83, 65, 425, 1.1999456469139835), (80, 65, 425, 1.199927827888617), (71, 65, 425, 1.1998941323855903), (88, 65, 426, 1.1998991142841409), (88, 23, 426, 1.199112525416846), (83, 65, 426, 1.199925918018347), (88, 65, 427, 1.199950335803784), (88, 23, 427, 1.1995905095227464), (83, 65, 427, 1.1999472442409373), (80, 65, 427, 1.1999346498730292), (71, 65, 427, 1.1998863017485701), (88, 65, 428, 1.1999475806912008), (88, 23, 428, 1.1990970789441409), (83, 65, 428, 1.1999182426077577), (80, 65, 428, 1.1999843103661936), (88, 65, 429, 1.199880748583793), (88, 23, 429, 1.19957814911475), (83, 65, 429, 1.1999449493549152), (80, 65, 429, 1.1999301261827027), (71, 65, 429, 1.1998839010641253), (88, 65, 430, 1.1998427329768022), (88, 23, 430, 1.1988995707702403), (83, 65, 430, 1.1999114031417863), (81, 59, 430, 1.1999999999959623), (80, 65, 430, 1.1999760505738704), (88, 65, 431, 1.1999277269450024), (88, 23, 431, 1.199509997792308), (83, 65, 431, 1.1999407662761095), (80, 65, 431, 1.1999226545452293), (71, 65, 431, 1.1998883551366668), (88, 65, 432, 1.199815060723563), (88, 23, 432, 1.1993089170004638), (83, 65, 432, 1.1999111157048266), (80, 65, 432, 1.1999729207824692), (88, 65, 433, 1.1998496829723893), (88, 23, 433, 1.199668106190761), (83, 65, 433, 1.1999377592230018), (80, 65, 433, 1.199919244955827), (71, 65, 433, 1.1998803793907327), (88, 65, 434, 1.1999482370339078), (88, 23, 434, 1.199102479017429), (83, 65, 434, 1.1998972055935238), (80, 65, 434, 1.1999597915961013), (88, 65, 435, 1.199899261730631), (88, 23, 435, 1.1995734115455285), (83, 65, 435, 1.1999329310877807), (80, 65, 435, 1.1999143002711279), (71, 65, 435, 1.1998925391605242), (88, 65, 436, 1.199976008737407), (88, 23, 436, 1.1991025755448503), (83, 65, 436, 1.199908368046966), (80, 65, 436, 1.1999602696864662), (88, 65, 437, 1.1998130067842387), (88, 23, 437, 1.199573629191235), (83, 65, 437, 1.199929462344127), (80, 65, 437, 1.199912805634112), (71, 65, 437, 1.1998545133465006), (88, 65, 438, 1.1999055786738746), (88, 23, 438, 1.1991342832411813), (83, 65, 438, 1.199883870624015), (80, 65, 438, 1.1999456148638623), (88, 65, 439, 1.1998240883350237), (88, 23, 439, 1.1995944431231615), (83, 65, 439, 1.199925491804448), (80, 65, 439, 1.1999089856784237), (71, 65, 439, 1.1998852849111108), (88, 65, 440, 1.1999686843882496), (88, 23, 440, 1.199041615093622), (83, 65, 440, 1.1998990294090437), (80, 65, 440, 1.1999579517780765), (88, 65, 441, 1.1997921211415092), (88, 23, 441, 1.1995314570389963), (83, 65, 441, 1.1999261815128173), (80, 65, 441, 1.1999087387211054), (71, 65, 441, 1.199874849646203), (88, 65, 442, 1.1999024308140347), (88, 23, 442, 1.1989686826213843), (83, 65, 442, 1.1998916346528372), (80, 65, 442, 1.1999527123251204), (88, 65, 443, 1.1998941637192686), (88, 23, 443, 1.1995356699592907), (83, 65, 443, 1.1999215838400374), (80, 65, 443, 1.1999020342312927), (71, 65, 443, 1.1998771366403331), (88, 65, 444, 1.199935332001095), (88, 23, 444, 1.1989961902642585), (83, 65, 444, 1.1999444869904872), (88, 65, 445, 1.199884189102663), (88, 23, 445, 1.199577978845083), (83, 65, 445, 1.199932396796164), (80, 65, 445, 1.1999090941531954), (71, 65, 445, 1.1998546349385668), (88, 65, 446, 1.19998656797472), (88, 23, 446, 1.1990562113642866), (83, 65, 446, 1.1999025423859737), (81, 59, 446, 1.1999999999536561), (80, 65, 446, 1.1999586996123779), (88, 65, 447, 1.1998771691830543), (88, 23, 447, 1.19955283403007), (83, 65, 447, 1.1999289540072822), (80, 65, 447, 1.1999100195929346), (71, 65, 447, 1.1998563002898806), (88, 65, 448, 1.1999543993114712), (88, 23, 448, 1.199113467401691), (83, 65, 448, 1.199906604004865), (80, 65, 448, 1.1999669686648897), (88, 65, 449, 1.1998186069080494), (88, 23, 449, 1.1995760344412811), (83, 65, 449, 1.1999251061466738), (80, 65, 449, 1.199903858451115), (71, 65, 449, 1.1998472937740772), (88, 65, 450, 1.1998901513092517), (88, 23, 450, 1.1992757206539535), (83, 65, 450, 1.199897653852077), (80, 65, 450, 1.1999565868658766), (88, 65, 451, 1.1998801907731647), (88, 23, 451, 1.1996623614180555), (83, 65, 451, 1.1999216875259087), (80, 65, 451, 1.1999005775744436), (71, 65, 451, 1.199851376184149), (88, 65, 452, 1.1998911121026334), (88, 23, 452, 1.1990247953544277), (83, 65, 452, 1.199994298564302), (88, 65, 453, 1.1998321037813577), (88, 23, 453, 1.1995829216921894), (83, 65, 453, 1.199925842592905), (80, 65, 453, 1.1998967999198196), (71, 65, 453, 1.1998388958509267), (88, 65, 454, 1.1999738775523086), (88, 23, 454, 1.1990433976808943), (83, 65, 454, 1.199900035536966), (80, 65, 454, 1.1999466895256974), (88, 65, 455, 1.1999553233112517), (88, 23, 455, 1.1996058602774458), (83, 65, 455, 1.1999253681715898), (80, 65, 455, 1.1999074919449706), (71, 65, 455, 1.1998478516488873), (88, 65, 456, 1.1999906756031573), (88, 23, 456, 1.199244660357998), (83, 65, 456, 1.1999128910160837), (80, 65, 456, 1.1999730012985197), (88, 65, 457, 1.1998467379577424), (88, 23, 457, 1.1996372642468691), (83, 65, 457, 1.1999271745303939), (80, 65, 457, 1.199909827871214), (71, 65, 457, 1.1998494674282945), (88, 65, 458, 1.199998617868887), (88, 23, 458, 1.1988633409326832), (83, 65, 458, 1.1998905673878097), (80, 65, 458, 1.1999554895261524), (88, 65, 459, 1.1998877698112929), (88, 23, 459, 1.1995548714056399), (83, 65, 459, 1.199926632251335), (80, 65, 459, 1.1999094956017038), (71, 65, 459, 1.199864462585258), (88, 65, 460, 1.1999658740230734), (88, 23, 460, 1.1992233883105912), (83, 65, 460, 1.1998871394477868), (80, 65, 460, 1.1999526341876485), (88, 65, 461, 1.1998578235658521), (88, 23, 461, 1.199628338356497), (83, 65, 461, 1.1999226011140793), (80, 65, 461, 1.1999030102007449), (71, 65, 461, 1.1998721108250734), (88, 65, 462, 1.199909917483938), (88, 23, 462, 1.1988559564018604), (83, 65, 462, 1.1998982245071415), (80, 65, 462, 1.199959498116356), (88, 65, 463, 1.1998613922267927), (88, 23, 463, 1.199510565007852), (83, 65, 463, 1.1999212519764217), (80, 65, 463, 1.1999003238281982), (71, 65, 463, 1.1998552443460235), (88, 65, 464, 1.1999878675358995), (88, 23, 464, 1.1991515739868521), (83, 65, 464, 1.1998910157864555), (80, 65, 464, 1.1999480161797154), (88, 65, 465, 1.199857742949918), (88, 23, 465, 1.1995961021290669), (83, 65, 465, 1.1999185904442768), (80, 65, 465, 1.1998975616894851), (71, 65, 465, 1.1998565318005119), (88, 65, 466, 1.1999787828956339), (88, 23, 466, 1.1988622384383958), (83, 65, 466, 1.1998909012074244), (80, 65, 466, 1.199948198557398), (88, 65, 467, 1.1998402493200861), (88, 23, 467, 1.1995351632420113), (83, 65, 467, 1.1999161298937855), (80, 65, 467, 1.1998945485230401), (71, 65, 467, 1.199851222621498), (88, 65, 468, 1.1999797231073002), (88, 23, 468, 1.1992162064282739), (83, 65, 468, 1.1998879990671025), (80, 65, 468, 1.1999427093355395), (88, 65, 469, 1.1998607171949511), (88, 23, 469, 1.1996247058165692), (83, 65, 469, 1.1999134437346077), (80, 65, 469, 1.1998918140790364), (71, 65, 469, 1.1998488685455244), (88, 65, 470, 1.1999638806805704), (88, 23, 470, 1.1988853229560426), (83, 65, 470, 1.1998856341186248), (80, 65, 470, 1.1999398154774918), (88, 65, 471, 1.1997901843413328), (88, 23, 471, 1.199514294248937), (83, 65, 471, 1.199910645811511), (80, 65, 471, 1.1998888374808836), (71, 65, 471, 1.1998456633231156), (88, 65, 472, 1.1999583182964448), (88, 23, 472, 1.1993175426653266), (83, 65, 472, 1.1998829680759713), (80, 65, 472, 1.1999356246345867), (88, 65, 473, 1.1997855009566052), (88, 23, 473, 1.1996767068863754), (83, 65, 473, 1.199908335198688), (80, 65, 473, 1.1998864051406237), (71, 65, 473, 1.1998432465420472), (88, 65, 474, 1.1998921417537545), (88, 23, 474, 1.1990405980708454), (83, 65, 474, 1.1998835872732267), (80, 65, 474, 1.1999314793133071), (88, 65, 475, 1.1997645264084782), (88, 23, 475, 1.1995488959496023), (83, 65, 475, 1.19990609593397), (80, 65, 475, 1.1998864521951664), (71, 65, 475, 1.1998231781845135), (88, 65, 476, 1.1999325279972628), (88, 23, 476, 1.199155642374134), (83, 65, 476, 1.1998779349761513), (80, 65, 476, 1.199932660042586), (88, 65, 477, 1.1998147197278026), (88, 23, 477, 1.1996097331936373), (83, 65, 477, 1.1999043786793446), (80, 65, 477, 1.1998838639825926), (71, 65, 477, 1.1998350427303937), (88, 65, 478, 1.1999863175758785), (88, 23, 478, 1.1991321472461618), (83, 65, 478, 1.1998838120221396), (80, 65, 478, 1.1999322091830102), (88, 65, 479, 1.1997807419979627), (88, 23, 479, 1.1995992296889006), (83, 65, 479, 1.1999044214681547), (80, 65, 479, 1.1998858679024667), (71, 65, 479, 1.1998130649603498), (88, 65, 480, 1.1999175448571162), (88, 23, 480, 1.19913897826123), (83, 65, 480, 1.1998741766053311), (80, 65, 480, 1.1999289676774079), (88, 65, 481, 1.1997567408555598), (88, 23, 481, 1.1995958604517343), (83, 65, 481, 1.1999049254099754), (80, 65, 481, 1.1998856001089448), (71, 65, 481, 1.19983297350462), (88, 65, 482, 1.1999524541046658), (88, 23, 482, 1.199212132386544), (83, 65, 482, 1.1998780909786588), (80, 65, 482, 1.1999289446002586), (88, 65, 483, 1.1998360061229425), (88, 23, 483, 1.1996366617656657), (83, 65, 483, 1.199902753635555), (80, 65, 483, 1.1998824322004833), (71, 65, 483, 1.1998312759289975), (88, 65, 484, 1.1999353926736267), (88, 23, 484, 1.199181594814116), (83, 65, 484, 1.1998821433539966), (80, 65, 484, 1.199927772128632), (88, 65, 485, 1.1999022016929706), (88, 23, 485, 1.1996092697752812), (83, 65, 485, 1.1999032230385984), (80, 65, 485, 1.1998841069133614), (71, 65, 485, 1.1998119055912733), (88, 65, 486, 1.199963662720574), (88, 23, 486, 1.1987033570764112), (83, 65, 486, 1.199877484586406), (80, 65, 486, 1.1999279776498923), (88, 65, 487, 1.1997900456024475), (88, 23, 487, 1.1995494269436362), (83, 65, 487, 1.1999046594594454), (80, 65, 487, 1.1998857042455329), (71, 65, 487, 1.1998258766156977), (88, 65, 488, 1.1999408496669994), (88, 23, 488, 1.1994446230153262), (83, 65, 488, 1.1998772478599478), (80, 65, 488, 1.1999286762332548), (88, 65, 489, 1.199785023050384), (88, 23, 489, 1.199741714804103), (83, 65, 489, 1.1999042765580914), (80, 65, 489, 1.19988456327907), (71, 65, 489, 1.199830670063139), (88, 65, 490, 1.1998263965427942), (88, 23, 490, 1.199117373311391), (83, 65, 490, 1.1999247606635042), (80, 65, 490, 1.1999895728631724), (88, 65, 491, 1.1998349162996516), (88, 23, 491, 1.1995838117054887), (83, 65, 491, 1.1999077261578543), (80, 65, 491, 1.1999818080542008), (71, 65, 491, 1.1998093392853078), (88, 65, 492, 1.199938357060562), (88, 23, 492, 1.199140547560826), (83, 65, 492, 1.1998814539142078), (80, 65, 492, 1.1999221451707078), (88, 65, 493, 1.1999717693764325), (88, 23, 493, 1.1996165569939905), (83, 65, 493, 1.1999075412385651), (80, 65, 493, 1.199890791111267), (71, 65, 493, 1.1998155023448507), (88, 65, 494, 1.1999233407371583), (88, 23, 494, 1.198993825512071), (83, 65, 494, 1.199881752781919), (80, 65, 494, 1.199934255164481), (88, 65, 495, 1.1999733440489455), (88, 23, 495, 1.199542205635733), (83, 65, 495, 1.1999133451298505), (80, 65, 495, 1.1998973233150012), (71, 65, 495, 1.1998271612361258), (88, 65, 496, 1.1999623915042603), (88, 23, 496, 1.1990843519570258), (83, 65, 496, 1.1998866797413532), (80, 65, 496, 1.1999383315943208), (88, 65, 497, 1.1998937049632463), (88, 23, 497, 1.1995912904399426), (83, 65, 497, 1.1999166474269038), (80, 65, 497, 1.1999027909156033), (71, 65, 497, 1.1998323182956259), (88, 65, 498, 1.1999682813840291), (88, 23, 498, 1.1990104332938007), (83, 65, 498, 1.1998828552595893), (80, 65, 498, 1.199939084888361), (88, 65, 499, 1.1998905639223365), (88, 23, 499, 1.1995527052801251), (83, 65, 499, 1.199923773411239), (80, 65, 499, 1.1999126541348333), (71, 65, 499, 1.199847489422768), (88, 65, 500, 1.1999328806271943), (88, 23, 500, 1.1989956428945936), (83, 65, 500, 1.1998896645483494), (80, 65, 500, 1.1999472786301453), (88, 65, 501, 1.199873331094227), (88, 23, 501, 1.1995470443526033), (83, 65, 501, 1.1999285677193403), (80, 65, 501, 1.1999184898975173), (71, 65, 501, 1.19985537467242), (88, 65, 502, 1.199943156047442), (88, 23, 502, 1.1990158577670447), (83, 65, 502, 1.1998926624037973), (80, 65, 502, 1.1999525693376463), (88, 65, 503, 1.1998773673581566), (88, 23, 503, 1.199557080991065), (83, 65, 503, 1.1999335864624114), (80, 65, 503, 1.1999257100213454), (71, 65, 503, 1.1998642487804394), (88, 65, 504, 1.199976410650843), (88, 23, 504, 1.1990074197172347), (83, 65, 504, 1.1998964956943352), (80, 65, 504, 1.1999569905547915), (88, 65, 505, 1.199867672584909), (88, 23, 505, 1.1995484879133436), (83, 65, 505, 1.1999395232798429), (80, 65, 505, 1.1999336447755053), (71, 65, 505, 1.1998750620417822), (88, 65, 506, 1.1999939315500425), (88, 23, 506, 1.198971740791934), (83, 65, 506, 1.1998988647832196), (81, 59, 506, 1.1999999999856452), (80, 65, 506, 1.1999650922139318), (88, 65, 507, 1.1998718146603475), (88, 23, 507, 1.1995305494856299), (83, 65, 507, 1.1999441281074843), (80, 65, 507, 1.1999369255504069), (71, 65, 507, 1.1998830767282513), (88, 65, 508, 1.1998980883817327), (88, 23, 508, 1.1989479405239762), (83, 65, 508, 1.1999011001283688), (80, 65, 508, 1.199968598365346), (88, 65, 509, 1.1999454427764928), (88, 23, 509, 1.1995194733688097), (83, 65, 509, 1.1999458440186852), (80, 65, 509, 1.1999353273160835), (71, 65, 509, 1.199895122848583), (88, 65, 510, 1.1998931752034883), (88, 23, 510, 1.1989680489709704), (83, 65, 510, 1.1999091977567193), (80, 65, 510, 1.1999709657536046), (88, 65, 511, 1.1999820862503972), (88, 23, 511, 1.1995305038188895), (83, 65, 511, 1.199947232010294), (80, 65, 511, 1.1999361573894787), (71, 65, 511, 1.1998844066419954), (88, 65, 512, 1.1998923126038594), (88, 23, 512, 1.1989655597662574), (83, 65, 512, 1.1999132002684292), (80, 65, 512, 1.1999704345467157), (88, 65, 513, 1.1997424088053787), (88, 23, 513, 1.1995297477806677), (83, 65, 513, 1.1999485375746108), (80, 65, 513, 1.1999391675984012), (71, 65, 513, 1.1998822203674113), (88, 65, 514, 1.1999961439161497), (88, 23, 514, 1.1989894047348983), (83, 65, 514, 1.1999104557598186), (80, 65, 514, 1.1999659054877085), (88, 65, 515, 1.1997396301424634), (88, 23, 515, 1.199542052828656), (83, 65, 515, 1.1999499120920163), (80, 65, 515, 1.1999441027637523), (71, 65, 515, 1.1998845983108617), (88, 65, 516, 1.1999958890265068), (88, 23, 516, 1.198985720102653), (83, 65, 516, 1.1999077195415568), (80, 65, 516, 1.1999631391589765), (88, 65, 517, 1.1999959949050836), (88, 23, 517, 1.199527164583905), (83, 65, 517, 1.1999514654025047), (80, 65, 517, 1.1999501888023538), (71, 65, 517, 1.1998881616015014), (88, 65, 518, 1.1999844174918233), (88, 23, 518, 1.198934948525412), (83, 65, 518, 1.1999084609559996), (80, 65, 518, 1.1999627597080478), (88, 65, 519, 1.1997113861326467), (88, 23, 519, 1.1995026037817513), (83, 65, 519, 1.1999511229609094), (80, 65, 519, 1.1999516017240408), (71, 65, 519, 1.1998858671770771), (88, 65, 520, 1.1999145108108573), (88, 23, 520, 1.1989519016097894), (83, 65, 520, 1.1999427331809231), (88, 65, 521, 1.1998913476419306), (88, 23, 521, 1.1995192222417312), (83, 65, 521, 1.199955967696543), (80, 65, 521, 1.1999531177459648), (71, 65, 521, 1.1998868338162572), (88, 65, 522, 1.1998902674217067), (88, 23, 522, 1.198916168454116), (83, 65, 522, 1.1999077873038722), (80, 65, 522, 1.1999513144303768), (88, 65, 523, 1.1999563979550916), (88, 23, 523, 1.1995070722797663), (83, 65, 523, 1.19995504755592), (80, 65, 523, 1.1999675488455321), (71, 65, 523, 1.1998913185933835), (88, 65, 524, 1.1999158526991305), (88, 23, 524, 1.1989750278175695), (83, 65, 524, 1.1999106495207545), (80, 65, 524, 1.1999565393209308), (88, 65, 525, 1.1997193173206424), (88, 23, 525, 1.1995267682393724), (83, 65, 525, 1.1999554138541095), (80, 65, 525, 1.1999755129793315), (71, 65, 525, 1.1998918920858697), (88, 65, 526, 1.1999983755096186), (88, 23, 526, 1.1989294630928347), (83, 65, 526, 1.1999097432736825), (80, 65, 526, 1.199954319339553), (88, 65, 527, 1.1997221467489712), (88, 23, 527, 1.1994995734255345), (83, 65, 527, 1.199955051965529), (80, 65, 527, 1.1999807486966576), (71, 65, 527, 1.1998912823002577), (88, 65, 528, 1.19992371504461), (88, 23, 528, 1.198946737031925), (83, 65, 528, 1.199947772132136), (88, 65, 529, 1.1999916491427622), (88, 23, 529, 1.1995164066330772), (83, 65, 529, 1.1999588547344004), (80, 65, 529, 1.199983149767551), (71, 65, 529, 1.199889045997893), (88, 65, 530, 1.1999036832511707), (88, 23, 530, 1.1988752320100176), (83, 65, 530, 1.1999170596532651), (80, 65, 530, 1.1999511702587515), (88, 65, 531, 1.199744156616608), (88, 23, 531, 1.1994976136858657), (83, 65, 531, 1.1999576630293853), (80, 65, 531, 1.1999940768693507), (71, 65, 531, 1.1998915936020107), (88, 65, 532, 1.1999136826248253), (88, 23, 532, 1.1990259949319282), (83, 65, 532, 1.1999102549250642), (80, 65, 532, 1.1999477963267247), (88, 65, 533, 1.1997196328967834), (88, 23, 533, 1.1995465289131118), (83, 65, 533, 1.1999575258325053), (71, 65, 533, 1.1998940936262885), (88, 65, 534, 1.1999276241303425), (88, 23, 534, 1.198884063292121), (83, 65, 534, 1.1999212819634815), (80, 65, 534, 1.1999627379372573), (88, 65, 535, 1.1997390804261894), (88, 23, 535, 1.1994826232312594), (83, 65, 535, 1.1999568985874762), (80, 65, 535, 1.1999996141258542), (71, 65, 535, 1.1998893958059582), (88, 65, 536, 1.1999164951564256), (88, 23, 536, 1.198987713552113), (83, 65, 536, 1.199918342715863), (80, 65, 536, 1.1999551166018665), (88, 65, 537, 1.199748645409365), (88, 23, 537, 1.1995312787714574), (83, 65, 537, 1.1999569464885633), (71, 65, 537, 1.1998901492868745), (88, 65, 538, 1.1999150551069984), (88, 23, 538, 1.1989176274557762), (83, 65, 538, 1.1999193934041126), (80, 65, 538, 1.1999565349300358), (88, 65, 539, 1.1997482818656555), (88, 23, 539, 1.1994984634798609), (83, 65, 539, 1.1999567742582247), (71, 65, 539, 1.199889360015985), (88, 65, 540, 1.199915699462975), (88, 23, 540, 1.1989804776851256), (83, 65, 540, 1.199896671280229), (80, 65, 540, 1.1999315352766555), (88, 65, 541, 1.1997504696461536), (88, 23, 541, 1.1995283638725824), (83, 65, 541, 1.1999541044390356), (71, 65, 541, 1.1998946906947112), (88, 65, 542, 1.199908351259108), (88, 23, 542, 1.1989062656323946), (83, 65, 542, 1.1999554175134608), (88, 65, 543, 1.199742017478661), (88, 23, 543, 1.1995036988698617), (83, 65, 543, 1.1999588123485854), (71, 65, 543, 1.1998843548168059), (88, 65, 544, 1.1999862990162382), (88, 23, 544, 1.1989177299986602), (83, 65, 544, 1.199951500105932), (80, 65, 544, 1.199993542026804), (88, 65, 545, 1.1998947170890437), (88, 23, 545, 1.1995074259486744), (83, 65, 545, 1.1999608410731768), (71, 65, 545, 1.1998872053371599), (88, 65, 546, 1.1998956099257274), (88, 23, 546, 1.1989118730959034), (83, 65, 546, 1.1999171737276781), (80, 65, 546, 1.1999445703746923), (88, 65, 547, 1.1999676412238554), (88, 23, 547, 1.1995098289290884), (83, 65, 547, 1.19995874753314), (71, 65, 547, 1.1998935826316404), (88, 65, 548, 1.1999933324779046), (88, 23, 548, 1.1990011202432487), (83, 65, 548, 1.1999173073300824), (80, 65, 548, 1.1999414427740218), (88, 65, 549, 1.199748463360786), (88, 23, 549, 1.1995426732806413), (83, 65, 549, 1.1999568499209639), (71, 65, 549, 1.1998863110996658), (88, 65, 550, 1.1999709985630933), (88, 23, 550, 1.1989592605672523), (83, 65, 550, 1.1999174000474013), (80, 65, 550, 1.1999436610213658), (88, 65, 551, 1.1997374464530783), (88, 23, 551, 1.1995136254045626), (83, 65, 551, 1.1999562796188434), (71, 65, 551, 1.1998865439094928), (88, 65, 552, 1.1999385996554879), (88, 23, 552, 1.1988445163544124), (83, 65, 552, 1.1999117936631956), (80, 65, 552, 1.1999409767748621), (88, 65, 553, 1.1997579707932977), (88, 23, 553, 1.1994966992583669), (83, 65, 553, 1.1999549240624001), (71, 65, 553, 1.1998879508119176), (88, 65, 554, 1.1998976392891576), (88, 23, 554, 1.1990445694313776), (83, 65, 554, 1.1999519935843248), (80, 65, 554, 1.1999980160714003), (88, 65, 555, 1.199809435358635), (88, 23, 555, 1.1995636272354975), (83, 65, 555, 1.199959557907644), (71, 65, 555, 1.1998834488768622), (88, 65, 556, 1.1999302251534596), (88, 23, 556, 1.198915899641213), (83, 65, 556, 1.1999229466101107), (80, 65, 556, 1.1999385711465562), (88, 65, 557, 1.1997367082046428), (88, 23, 557, 1.1995037222140097), (83, 65, 557, 1.1999580084410546), (71, 65, 557, 1.1998868745688918), (88, 65, 558, 1.1999578943748457), (88, 23, 558, 1.19893082920959), (83, 65, 558, 1.199899793577701), (80, 65, 558, 1.1999202006088623), (88, 65, 559, 1.1997369155888629), (88, 23, 559, 1.1995090881025128), (83, 65, 559, 1.1999544644778752), (71, 65, 559, 1.199898319642846), (88, 65, 560, 1.1999584112229909), (88, 23, 560, 1.1989743254148513), (83, 65, 560, 1.1999134197692327), (80, 65, 560, 1.1999489980068905), (88, 65, 561, 1.199735267500996), (88, 23, 561, 1.1995222360177376), (83, 65, 561, 1.1999534269844379), (71, 65, 561, 1.1998867538136886), (88, 65, 562, 1.1998914013519417), (88, 23, 562, 1.1988182530171692), (83, 65, 562, 1.1999506860520774), (88, 65, 563, 1.1999199158034495), (88, 23, 563, 1.1994971479357923), (83, 65, 563, 1.1999567839279326), (71, 65, 563, 1.1998790342842387), (88, 65, 564, 1.1999812820175124), (88, 23, 564, 1.1990001362523897), (83, 65, 564, 1.199950451293698), (80, 65, 564, 1.1999843542914155), (88, 65, 565, 1.1998706030239115), (88, 23, 565, 1.1995438078424527), (83, 65, 565, 1.199959655242318), (71, 65, 565, 1.1998830755086514), (88, 65, 566, 1.1999212854064523), (88, 23, 566, 1.1989266855236227), (83, 65, 566, 1.1999533870507513), (80, 65, 566, 1.1999908757497806), (88, 65, 567, 1.1999483438161704), (88, 23, 567, 1.1994929898558688), (83, 65, 567, 1.199962987567014), (71, 65, 567, 1.1998862522932017), (88, 65, 568, 1.1999267761636316), (88, 23, 568, 1.1990221561191372), (83, 65, 568, 1.1999202876896418), (80, 65, 568, 1.1999325343685854), (88, 65, 569, 1.1999918392384374), (88, 23, 569, 1.1995558693672335), (83, 65, 569, 1.199959036084022), (71, 65, 569, 1.199886775217112), (88, 65, 570, 1.199984819101835), (88, 23, 570, 1.1989860623344193), (83, 65, 570, 1.1999020663324713), (80, 65, 570, 1.1999220088212634), (88, 65, 571, 1.19973363967728), (88, 23, 571, 1.1995319111680882), (83, 65, 571, 1.1999573115416156), (71, 65, 571, 1.1998957756169746), (88, 65, 572, 1.1999914010196409), (88, 23, 572, 1.1989206014041642), (83, 65, 572, 1.199914451324284), (80, 65, 572, 1.1999527181017375), (71, 65, 572, 1.1999909966002253), (88, 65, 573, 1.1999229434125367), (88, 23, 573, 1.1994980670157063), (83, 65, 573, 1.1999548311692347), (71, 65, 573, 1.1998936082375524), (88, 65, 574, 1.1999413235754661), (88, 23, 574, 1.1988378186795914), (83, 65, 574, 1.199912682764951), (80, 65, 574, 1.1999495348010443), (88, 65, 575, 1.199993151842593), (88, 23, 575, 1.1994889190399483), (83, 65, 575, 1.1999531019449634), (71, 65, 575, 1.199892997354947), (88, 65, 576, 1.1999461452987), (88, 23, 576, 1.1990075627543275), (83, 65, 576, 1.199913047526295), (80, 65, 576, 1.1999441936628212), (88, 65, 577, 1.1999812882522642), (88, 23, 577, 1.1995396251659463), (83, 65, 577, 1.1999518904130688), (71, 65, 577, 1.1998860424677782), (88, 65, 578, 1.1999260167423365), (88, 23, 578, 1.198930697755051), (83, 65, 578, 1.1999069585700448), (80, 65, 578, 1.1999384076897204), (88, 65, 579, 1.1997280835741637), (88, 23, 579, 1.1995095108828968), (83, 65, 579, 1.1999499748848788), (71, 65, 579, 1.1998876821142774), (88, 65, 580, 1.1999713970260606), (88, 23, 580, 1.198933094558304), (83, 65, 580, 1.1999309279758381), (80, 65, 580, 1.1999714495010143), (88, 65, 581, 1.1997909568168414), (88, 23, 581, 1.199505632682269), (83, 65, 581, 1.1999530227787134), (71, 65, 581, 1.1998730725835984), (88, 65, 582, 1.199906188357986), (88, 23, 582, 1.1989327865982475), (83, 65, 582, 1.1999105542850184), (80, 65, 582, 1.1999317596609256), (88, 65, 583, 1.1997144285852086), (88, 23, 583, 1.1995057945568648), (83, 65, 583, 1.199949557086917), (71, 65, 583, 1.1998762821823556), (88, 65, 584, 1.1999530948415769), (88, 23, 584, 1.1989184839334115), (83, 65, 584, 1.1999025926125864), (80, 65, 584, 1.1999317439571853), (88, 65, 585, 1.1997199536178247), (88, 23, 585, 1.1994990709357642), (83, 65, 585, 1.1999464725955933), (71, 65, 585, 1.1998848215998419), (88, 65, 586, 1.1999558842054245), (88, 23, 586, 1.1988952289647594), (83, 65, 586, 1.1999395249477762), (80, 65, 586, 1.1999854156379541), (88, 65, 587, 1.1997889377387028), (88, 23, 587, 1.1994976285386072), (83, 65, 587, 1.1999507123014193), (80, 65, 587, 1.199997982029675), (71, 65, 587, 1.1998640987016), (88, 65, 588, 1.1999105771414504), (88, 23, 588, 1.1989296384694874), (83, 65, 588, 1.1999164111212828), (80, 65, 588, 1.199939595824903), (71, 65, 588, 1.1999986325814025), (88, 65, 589, 1.1999350281282848), (88, 23, 589, 1.1995103615807354), (83, 65, 589, 1.199947842481363), (71, 65, 589, 1.1998689022284126), (88, 65, 590, 1.199950637680344), (88, 23, 590, 1.1988963498787089), (83, 65, 590, 1.199888908510801), (80, 65, 590, 1.199918456001583), (88, 65, 591, 1.199716627749304), (88, 23, 591, 1.1994933664305454), (83, 65, 591, 1.1999434451281377), (80, 65, 591, 1.1999998033756498), (71, 65, 591, 1.199899230550501), (88, 65, 592, 1.1998901469175793), (88, 23, 592, 1.1989197967331051), (83, 65, 592, 1.1999083362660028), (80, 65, 592, 1.199940471327281), (88, 65, 593, 1.1997124440941094), (88, 23, 593, 1.1995030823233537), (83, 65, 593, 1.1999442150192183), (80, 65, 593, 1.199998468411895), (71, 65, 593, 1.1998845290383213), (88, 65, 594, 1.199927593840245), (88, 23, 594, 1.1989199352543343), (83, 65, 594, 1.1999365788407896), (80, 65, 594, 1.199978431289947), (88, 65, 595, 1.1997814064654042), (88, 23, 595, 1.1995027636455797), (83, 65, 595, 1.199948801127862), (80, 65, 595, 1.1999956821660884), (71, 65, 595, 1.1998586272808898), (88, 65, 596, 1.199880793647082), (88, 23, 596, 1.1989311648027818), (83, 65, 596, 1.199910478334761), (80, 65, 596, 1.1999309905410651), (88, 65, 597, 1.1996969542580793), (88, 23, 597, 1.199505853332694), (83, 65, 597, 1.1999453376851128), (80, 65, 597, 1.1999969106090407), (71, 65, 597, 1.199989080595218), (88, 65, 598, 1.1999164344349456), (88, 23, 598, 1.198919823580172), (83, 65, 598, 1.199882982743175), (80, 65, 598, 1.1999067672939159), (88, 65, 599, 1.199716134385152), (88, 23, 599, 1.199501832804802), (83, 65, 599, 1.1999402196093192), (80, 65, 599, 1.1999955610526916), (71, 65, 599, 1.1998968856598378), (88, 65, 600, 1.1999859569261142), (88, 23, 600, 1.1989030201000208), (83, 65, 600, 1.199897839892246), (80, 65, 600, 1.199927417229569), (88, 65, 601, 1.199743379268449), (88, 23, 601, 1.1994902626393447), (83, 65, 601, 1.199941298193862), (80, 65, 601, 1.1999918053642977), (71, 65, 601, 1.1998931410004001), (88, 65, 602, 1.199958089898042), (88, 23, 602, 1.1988762978438239), (83, 65, 602, 1.199940285248263), (80, 65, 602, 1.1999741523951155), (88, 65, 603, 1.1997714042956358), (88, 23, 603, 1.199491752254957), (83, 65, 603, 1.1999448094874474), (80, 65, 603, 1.199985014048576), (71, 65, 603, 1.1998483545345477), (88, 65, 604, 1.1998382730729347), (88, 23, 604, 1.1989609410159021), (83, 65, 604, 1.1999014501328924), (80, 65, 604, 1.1999116347989962), (88, 65, 605, 1.1999580860528205), (88, 23, 605, 1.1995275673697279), (83, 65, 605, 1.1999413891599549), (80, 65, 605, 1.199991764329927), (71, 65, 605, 1.1998600851215717), (88, 65, 606, 1.19993803215785), (88, 23, 606, 1.1989465805852393), (83, 65, 606, 1.1999328434579308), (80, 65, 606, 1.1999710560359673), (88, 65, 607, 1.199759030236432), (88, 23, 607, 1.1995159163346323), (83, 65, 607, 1.19994538318216), (80, 65, 607, 1.1999868825809188), (71, 65, 607, 1.1998496313885283), (88, 65, 608, 1.1998500087828032), (88, 23, 608, 1.1989178156141045), (83, 65, 608, 1.1998962146540928), (80, 65, 608, 1.1999141262742412), (88, 65, 609, 1.1996869786861606), (88, 23, 609, 1.1995031887824519), (83, 65, 609, 1.1999413818603344), (80, 65, 609, 1.1999877437206912), (71, 65, 609, 1.1998757828394526), (88, 65, 610, 1.1999139403607288), (88, 23, 610, 1.198940274397242), (83, 65, 610, 1.1999301136929572), (80, 65, 610, 1.1999680890349387), (88, 65, 611, 1.1996943746287492), (88, 23, 611, 1.1995126970182848), (83, 65, 611, 1.1999437183943311), (80, 65, 611, 1.1999825532098425), (71, 65, 611, 1.199854603103322), (88, 65, 612, 1.1999227368497678), (88, 23, 612, 1.1989352209190607), (83, 65, 612, 1.199936145241137), (80, 65, 612, 1.1999704267717717), (88, 65, 613, 1.1997631402886322), (88, 23, 613, 1.1995118194135566), (83, 65, 613, 1.1999466045908256), (80, 65, 613, 1.1999834273015961), (71, 65, 613, 1.1998482780088935), (88, 65, 614, 1.19984673850623), (88, 23, 614, 1.1989077648542543), (83, 65, 614, 1.1998976815696876), (80, 65, 614, 1.199913166256556), (71, 65, 614, 1.199997095495849), (88, 65, 615, 1.1999401898279671), (88, 23, 615, 1.1995024689804774), (83, 65, 615, 1.1999415603404566), (80, 65, 615, 1.199984850476801), (71, 65, 615, 1.1998749763590213), (88, 65, 616, 1.1999602636623876), (88, 23, 616, 1.1989710702467857), (83, 65, 616, 1.1998950707276488), (80, 65, 616, 1.1999164518844843), (88, 65, 617, 1.199683099655774), (88, 23, 617, 1.1995251476970163), (83, 65, 617, 1.1999393515622985), (80, 65, 617, 1.1999856190247598), (71, 65, 617, 1.1998783461606302), (88, 65, 618, 1.1999017804702186), (88, 23, 618, 1.1988761032303497), (83, 65, 618, 1.1999274629799006), (80, 65, 618, 1.1999630215928172), (88, 65, 619, 1.1996962941514233), (88, 23, 619, 1.1994872088024446), (83, 65, 619, 1.1999434670059939), (80, 65, 619, 1.199981584533376), (71, 65, 619, 1.199858424593248), (88, 65, 620, 1.1999172782737937), (88, 23, 620, 1.1989195019219208), (83, 65, 620, 1.1998963171917938), (80, 65, 620, 1.1999138913697227), (88, 65, 621, 1.1996867811572378), (88, 23, 621, 1.1995013530694738), (83, 65, 621, 1.1999401169754855), (80, 65, 621, 1.1999827907127312), (71, 65, 621, 1.1998804284125453), (88, 65, 622, 1.1999016302004513), (88, 23, 622, 1.19888204232765), (83, 65, 622, 1.199928807315282), (80, 65, 622, 1.1999615288599417), (88, 65, 623, 1.199696124313563), (88, 23, 623, 1.1994911642589037), (83, 65, 623, 1.1999433003765692), (80, 65, 623, 1.199979184626055), (71, 65, 623, 1.1998563590755438), (88, 65, 624, 1.1998796161948695), (88, 23, 624, 1.1989192773680926), (83, 65, 624, 1.1999223000163524), (80, 65, 624, 1.1999462418943285), (88, 65, 625, 1.1999908724178394), (88, 23, 625, 1.1995048459672324), (83, 65, 625, 1.199943602662775), (80, 65, 625, 1.1999792852250586), (71, 65, 625, 1.1998558858844266), (88, 65, 626, 1.1999259939767297), (88, 23, 626, 1.198922366749183), (83, 65, 626, 1.1998926126776146), (80, 65, 626, 1.1999064569873539), (71, 65, 626, 1.1999876490477515), (88, 65, 627, 1.1996573912054715), (88, 23, 627, 1.19950392353133), (83, 65, 627, 1.1999401991597902), (80, 65, 627, 1.1999794166778222), (71, 65, 627, 1.1998789546600788), (88, 65, 628, 1.199940318989329), (88, 23, 628, 1.1989297691434675), (83, 65, 628, 1.1998897842437284), (80, 65, 628, 1.1999088615675995), (88, 65, 629, 1.1996816863989683), (88, 23, 629, 1.1995053562007256), (83, 65, 629, 1.1999376390297967), (80, 65, 629, 1.1999779725070032), (71, 65, 629, 1.1998916333913423), (88, 65, 630, 1.1998960182328753), (88, 23, 630, 1.1988837262110021), (83, 65, 630, 1.1999274480128324), (80, 65, 630, 1.199956449408941), (88, 65, 631, 1.199678608675808), (88, 23, 631, 1.1994921673800034), (83, 65, 631, 1.1999413759737614), (80, 65, 631, 1.1999735594379464), (71, 65, 631, 1.1998596796416712), (88, 65, 632, 1.1998747516197235), (88, 23, 632, 1.1989045423982023), (83, 65, 632, 1.199925885768112), (80, 65, 632, 1.1999831731263846), (88, 65, 633, 1.199680081602054), (88, 23, 633, 1.199499240021168), (83, 65, 633, 1.1999431285880764), (80, 65, 633, 1.199985200479967), (71, 65, 633, 1.1998508781933581), (88, 65, 634, 1.1998909563320945), (88, 23, 634, 1.1989321908486577), (83, 65, 634, 1.1999264077008376), (80, 65, 634, 1.1999799128058206), (88, 65, 635, 1.1996862102952626), (88, 23, 635, 1.1995109117247882), (83, 65, 635, 1.1999433473390821), (80, 65, 635, 1.1999856200662236), (71, 65, 635, 1.1998474189072283), (88, 65, 636, 1.199904780970858), (88, 23, 636, 1.1989727703167061), (83, 65, 636, 1.1999324898320127), (80, 65, 636, 1.1999907648545018), (88, 65, 637, 1.1997070669941021), (88, 23, 637, 1.1995330202321923), (83, 65, 637, 1.1999476168876033), (80, 65, 637, 1.1999868448197792), (71, 65, 637, 1.1998414994703126), (88, 65, 638, 1.1999370063753685), (88, 23, 638, 1.198990371920917), (83, 65, 638, 1.1998963360240142), (80, 65, 638, 1.1999012999710423), (88, 65, 639, 1.199659566249656), (88, 23, 639, 1.1995323095079866), (83, 65, 639, 1.199941684551092), (80, 65, 639, 1.1999823466347324), (71, 65, 639, 1.199844081941565), (88, 65, 640, 1.199925719753597), (88, 23, 640, 1.198864508327771), (83, 65, 640, 1.1998809384639402), (80, 65, 640, 1.199946998582244), (88, 65, 641, 1.1999318835704227), (88, 23, 641, 1.1994836634013324), (83, 65, 641, 1.1999391777523252), (80, 65, 641, 1.1999886911229507), (71, 65, 641, 1.1998899127066918), (88, 65, 642, 1.1999062812417187), (88, 23, 642, 1.1989522726607678), (83, 65, 642, 1.1999247215720543), (80, 65, 642, 1.1999788569569712), (88, 65, 643, 1.1996785146367288), (88, 23, 643, 1.1995189589566202), (83, 65, 643, 1.1999430016914556), (80, 65, 643, 1.1999855869761553), (71, 65, 643, 1.1998628348616192), (88, 65, 644, 1.1999572976134294), (88, 23, 644, 1.1988344166785454), (83, 65, 644, 1.1998873208084153), (80, 65, 644, 1.1999572904698315), (88, 65, 645, 1.1996755179653937), (88, 23, 645, 1.1994869096325627), (83, 65, 645, 1.1999386874560594), (80, 65, 645, 1.1999886295421676), (71, 65, 645, 1.1998883915359124), (88, 65, 646, 1.199874322062299), (88, 23, 646, 1.198971961634356), (83, 65, 646, 1.1999274120067163), (80, 65, 646, 1.1999825170147438), (88, 65, 647, 1.1996789574206261), (88, 23, 647, 1.199526848876049), (83, 65, 647, 1.1999424319501337), (80, 65, 647, 1.1999849138652627), (71, 65, 647, 1.1998578395125385), (88, 65, 648, 1.1998758717310865), (88, 23, 648, 1.1988609748306582), (83, 65, 648, 1.1999229154143765), (80, 65, 648, 1.1999789204303992), (88, 65, 649, 1.1996777072659288), (88, 23, 649, 1.1994782353399243), (83, 65, 649, 1.199943596536497), (80, 65, 649, 1.199986731406739), (71, 65, 649, 1.1998533571698182), (88, 65, 650, 1.199895648932777), (88, 23, 650, 1.1989352849106965), (83, 65, 650, 1.1999275862544039), (80, 65, 650, 1.1999805744136383), (88, 65, 651, 1.199683930696711), (88, 23, 651, 1.199513702374248), (83, 65, 651, 1.1999442300161822), (80, 65, 651, 1.199985294588573), (71, 65, 651, 1.199845467594363), (88, 65, 652, 1.199913200478506), (88, 23, 652, 1.1989628422529532), (83, 65, 652, 1.1998916304954155), (80, 65, 652, 1.1999756016568968), (88, 65, 653, 1.1997172424497606), (88, 23, 653, 1.199519123961133), (83, 65, 653, 1.199941625314059), (80, 65, 653, 1.1999899587096865), (71, 65, 653, 1.1998650035642255), (88, 65, 654, 1.199946547767291), (88, 23, 654, 1.1988799116002447), (83, 65, 654, 1.199923735165211), (80, 65, 654, 1.199976982800475), (88, 65, 655, 1.1997374921064272), (88, 23, 655, 1.1994899220651731), (83, 65, 655, 1.1999445148985783), (80, 65, 655, 1.199984250485923), (71, 65, 655, 1.199853078034822), (88, 65, 656, 1.1999334800276331), (88, 23, 656, 1.1989776207916534), (83, 65, 656, 1.1999307967710877), (88, 65, 657, 1.1998978404454943), (88, 23, 657, 1.1995453335929027), (83, 65, 657, 1.1999454644565455), (80, 65, 657, 1.1999872866835837), (71, 65, 657, 1.1998380459160838), (88, 65, 658, 1.199926520080136), (88, 23, 658, 1.1989757057383212), (83, 65, 658, 1.1998859147510605), (80, 65, 658, 1.1999901383759775), (88, 65, 659, 1.1996560294622542), (88, 23, 659, 1.1995247433154959), (83, 65, 659, 1.199942000506938), (80, 65, 659, 1.1999960802146104), (71, 65, 659, 1.199856653104933), (88, 65, 660, 1.199911466738879), (88, 23, 660, 1.1989083850772813), (83, 65, 660, 1.1998769175712256), (80, 65, 660, 1.1999415582343365), (88, 65, 661, 1.1996685955630262), (88, 23, 661, 1.1994923717829173), (83, 65, 661, 1.1999385728334317), (80, 65, 661, 1.1999899843920505), (71, 65, 661, 1.1998979871891677), (88, 65, 662, 1.1999929104035312), (88, 23, 662, 1.1989354204257083), (83, 65, 662, 1.1999230789061286), (80, 65, 662, 1.1999791481277282), (88, 65, 663, 1.1996645882561912), (88, 23, 663, 1.1995094679152751), (83, 65, 663, 1.1999424027449057), (80, 65, 663, 1.1999846040409763), (71, 65, 663, 1.1998639081165259), (88, 65, 664, 1.1998728207146898), (88, 23, 664, 1.1988421075854971), (83, 65, 664, 1.1999210742311062), (80, 65, 664, 1.1999756649856663), (88, 65, 665, 1.1996609844030182), (88, 23, 665, 1.1994772721708264), (83, 65, 665, 1.1999434155646402), (80, 65, 665, 1.1999854137969947), (71, 65, 665, 1.199856938135988), (88, 65, 666, 1.1999575459200666), (88, 23, 666, 1.1989285418077815), (83, 65, 666, 1.199882394014259), (80, 65, 666, 1.1999470841865383)]
    # 0.8255748815746515'
    data = fdata

    # 1.1262774896482923
    data = [(0, 74, 14, 1.138834111333078), (2, 28, 122, 1.1262774896482923), (7, 83, 127, 1.1986779871503932), (7, 95, 135, 1.199987793619739), (23, 80, 147, 1.197148183620267), (23, 88, 146, 1.1753976162792636), (26, 59, 145, 1.1969954238067515), (26, 81, 101, 1.1993749484774145), (26, 83, 93, 1.1976391165495528), (38, 80, 175, 1.1993506245259267), (59, 80, 146, 1.1978569758424515), (59, 81, 164, 1.199042424893474), (65, 71, 159, 1.1978367365717528), (65, 80, 172, 1.1972071699557534), (65, 81, 160, 1.1998898584867113), (65, 83, 72, 1.19661684664794), (65, 88, 164, 1.1986024308415426), (71, 81, 165, 1.1999336177239344), (71, 83, 151, 1.1947506866791784), (81, 83, 143, 1.1999360473612226), (83, 95, 159, 1.1988830720924462)]

    # 0.7167124801335585
    data=[(0, 4, 23, 1.1733766495941726), (0, 17, 41, 1.1858630934718197), (14, 20, 150, 1.1832870022631763), (17, 36, 126, 1.1973574773086653), (17, 90, 74, 1.1615365809960743), (17, 95, 127, 1.1821459818912645), (24, 59, 127, 1.0508454698984204), (24, 80, 108, 0.7167124801335585), (26, 42, 110, 1.128248548663847), (26, 64, 76, 1.193922799997908), (42, 92, 111, 1.187607765958225), (42, 99, 131, 1.1381474548382597), (44, 64, 73, 1.1252275949253903), (54, 92, 111, 1.193557887294853), (89, 90, 119, 1.1829750489303077), (90, 92, 119, 1.177498791079461), (90, 94, 75, 1.0903134852720797), (90, 99, 134, 1.1974873994827406)]


    # 20ob，半径内障碍物
    # 1.0840440337187116
    data=[(0, 98, 76, 1.0840440337187116), (1, 5, 61, 1.1767475096528954)]

    # 0.28288312371000673 
    fdata = [(80, 34, 73, 1.1798436366327902), (80, 34, 77, 1.1518248364068076), (80, 34, 78, 1.19101509342839), (80, 34, 79, 1.1920938837206694), (80, 34, 80, 1.0849768321334452), (80, 34, 81, 1.0384818183593485), (80, 17, 89, 1.1873948688058682), (80, 17, 90, 1.1976465866060473), (31, 0, 111, 1.1505046226123217), (31, 0, 112, 1.118699318543137), (31, 0, 113, 1.031897541060171), (31, 0, 114, 1.0683060570460334), (31, 0, 115, 1.0644400634557032), (31, 0, 116, 0.7790744711540971), (31, 0, 117, 0.505514111688598), (82, 7, 118, 1.0699207204048382), (31, 0, 118, 0.28288312371000673), (31, 0, 119, 0.32872478667792804), (82, 7, 120, 1.084740076786449), (31, 0, 120, 0.6476745256812766), (82, 7, 121, 1.08899969416442), (71, 7, 121, 1.1063409651516005), (31, 0, 121, 1.0645821605255452), (82, 7, 122, 1.0167005147197463), (71, 7, 122, 1.1752242003581959), (82, 7, 123, 0.9961966634066254), (71, 7, 123, 1.1916403919996326), (82, 7, 124, 1.0839212614087752), (71, 7, 124, 1.174614697541143), (71, 7, 125, 1.1923917711236156), (71, 7, 126, 1.1956508448621388), (71, 7, 127, 1.1978213806593814), (87, 0, 132, 1.1707374961285562), (87, 0, 133, 1.1900866351242345), (46, 0, 141, 1.1988514316251904), (46, 0, 142, 1.1997562087848654), (92, 0, 153, 1.1439961152127889), (92, 0, 154, 1.1535215943157768), (92, 0, 155, 1.1465090389955366), (92, 0, 156, 1.1976007599784342), (92, 0, 157, 1.185777535480411), (92, 0, 158, 1.1881043863448784), (92, 0, 159, 1.1211720996791428), (92, 0, 160, 1.1496737251361022), (70, 7, 247, 1.1469708272905539)]
    data=fdata

    # 1.0644400634557032 
    data=[(0, 31, 114, 1.0644400634557032), (0, 46, 140, 1.1988514316251904), (0, 92, 157, 1.1881043863448784), (7, 71, 126, 1.1978213806593814), (7, 82, 120, 1.08899969416442), (11, 34, 96, 1.1989114959083098), (17, 80, 89, 1.1976465866060473), (34, 60, 85, 1.1064724197689433), (34, 80, 78, 1.1920938837206694), (70, 82, 244, 1.1775758623465251), (80, 96, 79, 1.195618413945242)]
    
    # 障碍物范围加大agent半径
    # 0.8175680175108052 
    fdata = [(1, 0, 68, 0.8175680175108052), (1, 0, 69, 0.8891218016802789), (1, 0, 70, 1.115128660614376), (93, 72, 71, 1.1375169649849206), (4, 0, 72, 1.1877802771683517), (4, 0, 73, 1.1882020254097554), (4, 0, 74, 1.1502179156701884), (4, 0, 75, 1.1327684365608732), (4, 0, 76, 1.1322949074049158), (4, 0, 77, 1.1470408276708228), (4, 0, 78, 1.184747830208681), (80, 21, 88, 1.1954575459750978), (45, 43, 121, 1.1298028472335893), (59, 40, 128, 1.1219213086197644)]
    data = fdata
    # 1.1680942288023868
    data = [(0, 1, 64, 1.1680942288023868), (0, 4, 72, 1.1882020254097554)]

    # 0.8955171504140642
    fdata=[(0, 20, 115, 1.1997049930118409), (0, 26, 87, 1.1651860605936457), (0, 74, 105, 1.19386560465112), (1, 82, 104, 1.1720199422358362), (3, 6, 92, 1.1515815866734744), (3, 18, 109, 1.198850836892829), (6, 18, 90, 1.1915303797010843), (9, 56, 135, 1.1876389942297378), (18, 33, 113, 1.094932031182412), (18, 67, 112, 1.162619026892175), (18, 71, 115, 1.1021188228414107), (20, 74, 116, 1.1816881764270286), (25, 96, 229, 1.1999046858454765), (32, 39, 73, 0.9423005516682416), (33, 50, 160, 1.1999999999149475), (33, 67, 115, 1.18996793056335), (33, 70, 119, 0.9427506559051471), (33, 71, 118, 1.1283546042934272), (38, 42, 86, 1.1756144325041802), (38, 47, 183, 1.1794106773610689), (40, 68, 132, 1.1983720154403057), (40, 82, 103, 1.1440316298528932), (40, 85, 130, 1.1373707319525674), (47, 51, 203, 1.1734145812021346), (47, 94, 165, 1.1992041572748087), (51, 96, 205, 1.1940325380659245), (56, 96, 233, 1.1657485308784328), (57, 85, 131, 1.16237575093102), (58, 96, 224, 1.1334154164308186), (63, 98, 137, 0.8955171504140642), (67, 79, 99, 1.1304071394696982), (77, 83, 151, 1.1475656740180429), (82, 90, 139, 1.191132920318664), (82, 98, 140, 1.0355732619441023), (94, 97, 166, 1.0710763747927898), (96, 97, 213, 1.1925293325622524)]
    data = fdata
    
    # 0.6461503448531394
    data = [(39, 32, 73, 0.9365897536018193), (39, 32, 74, 0.9423005516682416), (39, 32, 75, 0.7779626032576465), (39, 6, 75, 1.1761511496853132), (39, 32, 76, 0.7779464435969088), (39, 6, 76, 1.1359326994470313), (39, 32, 77, 0.9475702342358832), (39, 6, 77, 1.092845713006568), (39, 32, 78, 1.0724771338099028), (42, 38, 81, 1.1720328444035153), (42, 38, 82, 1.0743277324076899), (42, 38, 83, 1.1512717293291201), (42, 38, 84, 1.170389627009473), (26, 0, 84, 1.166840191932449), (42, 38, 85, 1.17514818118888), (26, 0, 85, 1.1249849404478027), (42, 38, 86, 1.1755627038274512), (26, 0, 86, 1.160705637626506), (18, 6, 86, 1.124125880742011), (90, 2, 87, 1.1793375918680875), (42, 38, 87, 1.1756144325041802), (26, 0, 87, 1.159839222002453), (18, 6, 87, 1.095966715988893), (85, 18, 88, 1.1517490885895574), (26, 0, 88, 1.1651860605936457), (18, 6, 88, 1.1940746112762344), (6, 3, 88, 1.0914713432964587), (85, 3, 89, 1.0377068747859852), (18, 6, 89, 1.1914893545771974), (6, 3, 89, 1.1668667806918382), (85, 3, 90, 1.1343608647814456), (18, 6, 90, 1.1915490309594352), (6, 3, 90, 1.1542229694745274), (85, 3, 91, 1.06805717180912), (18, 6, 91, 1.1915303797010843), (6, 3, 91, 1.1294116418455011), (85, 3, 92, 1.0769263696061375), (18, 6, 92, 1.0650955488619605), (6, 3, 92, 1.122097742963224), (18, 6, 93, 1.0502925300502577), (70, 32, 94, 1.1866594740112373), (18, 6, 94, 1.1057298202777917), (79, 67, 95, 1.1381233123595125), (18, 6, 95, 1.1763561077325393), (79, 67, 96, 1.0648086131844694), (79, 67, 97, 0.991565492433235), (79, 67, 98, 1.127922944472231), (82, 40, 99, 1.0625632626753951), (79, 67, 99, 1.1303874090120618), (21, 18, 99, 1.1498594027799638), (82, 40, 100, 1.0872450701654075), (79, 67, 100, 1.1304071394696982), (21, 18, 100, 1.1757213883255353), (2, 1, 100, 1.1835223085988373), (82, 40, 101, 1.1357016367678945), (82, 1, 101, 1.1462862783901167), (79, 67, 101, 1.00660674566895), (21, 18, 101, 1.1617503838344525), (2, 1, 101, 1.1510395830305071), (82, 40, 102, 1.1404005121678795), (82, 1, 102, 1.1443374861311517), (79, 67, 102, 1.0472267886971198), (74, 0, 102, 1.1530725040785303), (2, 1, 102, 1.1969826876013896), (82, 40, 103, 1.1424866264332438), (74, 0, 103, 1.1634684772988748), (2, 1, 103, 1.1402188774554773), (90, 2, 104, 1.1681817280310969), (82, 40, 104, 1.1440316298528932), (82, 1, 104, 1.176127732870515), (74, 0, 104, 1.1788289410870274), (90, 2, 105, 1.156038336853785), (82, 40, 105, 1.1184835414786707), (82, 1, 105, 1.1720199422358362), (74, 0, 105, 1.190956397551862), (90, 2, 106, 1.1372293839442016), (82, 1, 106, 1.1901201793044702), (74, 0, 106, 1.19386560465112), (74, 0, 107, 1.1837949681904607), (82, 1, 108, 1.1504699465178858), (67, 18, 108, 1.1212460221824554), (82, 1, 109, 1.1660759518884845), (67, 18, 109, 0.9418314118914699), (71, 18, 110, 1.173504657184308), (67, 18, 110, 1.045051588524578), (2, 1, 110, 1.1993361160684186), (82, 1, 111, 1.1837356943008408), (74, 0, 111, 1.1631815691339764), (71, 18, 111, 1.0030145208227759), (67, 18, 111, 1.1141551973517918), (71, 18, 112, 1.1199573953420254), (67, 33, 112, 1.0782493164431282), (67, 18, 112, 1.1572623640455941), (71, 70, 113, 1.1476694352981263), (71, 18, 113, 1.1768065989720107), (67, 33, 113, 1.1946074987971378), (67, 18, 113, 1.162619026892175), (71, 70, 114, 1.1868583231296803), (71, 33, 114, 1.1917899523669628), (71, 33, 115, 1.0923581223411465), (67, 33, 115, 1.1825492490417315), (71, 33, 116, 1.0990163003532962), (70, 33, 116, 1.0903388915973538), (67, 33, 116, 1.18996793056335), (20, 0, 116, 1.1997049930118409), (71, 33, 117, 1.1366996644401737), (70, 33, 117, 0.8945979865553697), (20, 0, 117, 1.1216677644510054), (85, 83, 118, 1.1893772356444638), (71, 33, 118, 1.1288287392741136), (70, 33, 118, 0.970779146936833), (20, 0, 118, 1.0086594235349815), (85, 83, 119, 1.1731412334854436), (71, 33, 119, 1.1283546042934272), (70, 33, 119, 0.9404104537611372), (20, 0, 119, 0.8981759667482407), (85, 83, 120, 1.1962538259169688), (71, 33, 120, 1.1005144598945396), (70, 33, 120, 0.9427506559051471), (20, 0, 120, 0.7945540329883721), (71, 33, 121, 1.1140747038338592), (70, 33, 121, 0.8294792605664877), (20, 0, 121, 0.706022169827418), (71, 33, 122, 1.1540967395576889), (70, 33, 122, 0.9846180954228614), (20, 0, 122, 0.6487244871757345), (20, 0, 123, 0.6461503448531394), (20, 0, 124, 0.7442167288089161), (20, 0, 125, 0.9493882863293331), (20, 0, 126, 1.190077343633742), (85, 57, 127, 1.1566240028078858), (85, 40, 127, 1.0887781440075335), (85, 57, 128, 1.1750962306856394), (85, 40, 128, 1.110137582787971), (53, 47, 128, 1.1536115689354016), (85, 57, 129, 1.1890607111365463), (85, 40, 129, 1.1109733740745753), (85, 57, 130, 1.1835524266392272), (85, 40, 130, 1.1363795941540078), (85, 57, 131, 1.1633923338534555), (85, 40, 131, 1.1373707319525674), (85, 57, 132, 1.16237575093102), (85, 40, 132, 1.0740241527065446), (74, 0, 132, 1.1967631221312045), (68, 40, 132, 1.1963501567474635), (56, 9, 132, 1.1504472635244922), (98, 63, 133, 1.1893710903150267), (85, 57, 133, 1.1083040817428367), (74, 0, 133, 1.1888629524054861), (68, 40, 133, 1.1983720154403057), (98, 63, 134, 1.0150410270841805), (74, 0, 134, 1.1710296997942387), (56, 9, 134, 1.184741818245036), (98, 63, 135, 0.8823435106314561), (74, 0, 135, 1.1936217093176704), (40, 33, 135, 1.1522639071229523), (98, 82, 136, 1.0473771498170097), (98, 63, 136, 0.9009702182916425), (74, 0, 136, 1.1931036448209675), (56, 9, 136, 1.1876389942297378), (98, 82, 137, 1.0659082510001865), (98, 63, 137, 0.8956702073375489), (90, 82, 137, 1.1937404187456822), (74, 0, 137, 1.156715191154912), (56, 9, 137, 1.110365064586701), (98, 82, 138, 1.0715205248580397), (98, 63, 138, 0.8955171504140642), (90, 82, 138, 1.1869028909397699), (74, 0, 138, 1.1489580816541678), (56, 9, 138, 1.185394451849046), (98, 82, 139, 1.0906870518233363), (98, 63, 139, 0.7150190806262935), (90, 82, 139, 1.1858189230938596), (74, 0, 139, 1.152019083780089), (98, 82, 140, 1.0351351187543847), (98, 63, 140, 0.7630744829935657), (90, 82, 140, 1.191132920318664), (74, 0, 140, 1.1558389953430162), (98, 82, 141, 1.0355732619441023), (98, 63, 141, 1.0139827293304753), (74, 0, 141, 1.1596196826754777), (98, 82, 142, 0.9566450875643441), (74, 0, 142, 1.1638125471647607), (53, 25, 142, 1.1535772057517482), (98, 82, 143, 0.9817340612729598), (77, 71, 143, 1.1783838500154993), (74, 0, 143, 1.168666361089435), (53, 25, 143, 1.1095147328377402), (98, 82, 144, 1.109975561297256), (77, 71, 144, 1.1475233936960434), (74, 0, 144, 1.1748333515286091), (53, 25, 144, 1.1851180260983354), (74, 0, 145, 1.1832510073191191), (74, 0, 146, 1.195472875996048), (83, 77, 147, 1.1306747543761777), (83, 77, 148, 1.083389997289688), (83, 77, 149, 1.1515003115814932), (83, 77, 150, 1.1475939375374777), (83, 77, 151, 1.1475652613318463), (83, 77, 152, 1.1475656740180429), (83, 77, 153, 1.1605021225260412), (50, 33, 161, 1.1999999999149475), (97, 94, 162, 1.1372670257942474), (97, 94, 163, 0.9425262104322583), (77, 57, 163, 1.0853010508640075), (97, 94, 164, 1.0136032506101007), (97, 94, 165, 1.0029155568777575), (94, 47, 165, 1.198842881622234), (97, 94, 166, 1.0001279161673924), (94, 58, 166, 1.1738059741350628), (94, 47, 166, 1.1992041572748087), (66, 57, 166, 1.179296935406829), (51, 47, 194, 1.088229673833478), (58, 51, 199, 1.0786118562917604), (58, 51, 200, 1.1365428259322543), (51, 47, 201, 1.163816090124542), (51, 47, 203, 1.163864906743755), (51, 47, 204, 1.1734145812021346), (97, 96, 210, 1.1862681004078603), (97, 96, 211, 1.0798090874621522), (97, 96, 212, 1.1893831563359674), (97, 96, 213, 1.1925251378171646), (97, 96, 214, 1.1925293325622524), (51, 25, 216, 1.1918266196052836), (51, 25, 217, 0.9443576367998219), (51, 25, 218, 1.0173393561715953), (51, 25, 219, 1.1470407528107367), (96, 58, 221, 1.1995753534408449), (96, 58, 222, 1.0955900345226521), (96, 58, 225, 1.1334154164308186), (96, 58, 226, 1.0913098230130074), (96, 58, 227, 1.1551148736401768), (96, 25, 227, 1.1890012133878791), (96, 25, 229, 1.1919199739797675), (96, 25, 230, 1.1999046858454765), (96, 81, 243, 1.1236618097383235)]

    # j改为 i
    # 0.8451520752322724
    fdata = [(44, 0, 71, 1.1190220785592704), (44, 0, 72, 1.100951456370143), (44, 0, 73, 1.1793246300994529), (44, 0, 74, 1.1458816697076124), (44, 0, 75, 1.1307679660863794), (44, 0, 76, 1.189913593547433), (44, 0, 77, 1.1505622791444565), (44, 0, 78, 1.1538178889328234), (30, 27, 78, 1.199718829134996), (44, 0, 79, 1.1524111023730563), (30, 27, 79, 1.0797230016595805), (30, 27, 80, 1.1649741464810899), (30, 27, 81, 1.1827682501975272), (30, 27, 82, 1.187785309955391), (30, 27, 83, 1.1882525007967095), (30, 27, 84, 1.1639337318649479), (94, 82, 97, 1.1334538925242281), (50, 5, 97, 1.137139893933782), (94, 82, 98, 1.1598573324285266), (94, 82, 99, 1.1654148729109313), (94, 82, 100, 1.1703815769187114), (94, 82, 101, 1.1786365661697082), (94, 46, 101, 1.1997567498562682), (94, 82, 102, 1.1817437951927856), (94, 82, 103, 1.1817633259893767), (94, 82, 104, 1.1817632342921571), (85, 30, 104, 1.1307812911685644), (36, 22, 104, 1.1575156986667148), (36, 22, 105, 1.1824600631983737), (36, 22, 106, 0.9277335621114968), (69, 36, 107, 1.1776703139858036), (36, 30, 107, 1.1777055238620642), (36, 22, 107, 1.0316231649055951), (36, 30, 108, 1.1932602444004952), (36, 22, 108, 1.0318969691708757), (85, 30, 109, 1.1842981374546022), (69, 12, 109, 1.159803266022168), (36, 30, 109, 1.1942412099028386), (36, 22, 109, 0.8852636421723501), (85, 30, 110, 1.1890254412162982), (69, 12, 110, 1.0532054267304494), (36, 30, 110, 1.1341819424135153), (36, 22, 110, 0.8451520752322724), (69, 12, 111, 1.0755287228513009), (36, 30, 111, 1.0959749324599297), (36, 22, 111, 0.9115278467071363), (69, 12, 112, 1.1179174903122966), (36, 30, 112, 1.0966967885355234), (36, 22, 112, 1.0820041685251496), (69, 12, 113, 1.1229300861818021), (36, 30, 113, 1.1795898461451337), (69, 36, 114, 1.1899980477416774), (69, 12, 114, 1.122949615421259), (69, 36, 115, 1.1004742076294944), (69, 12, 115, 1.122949310367536), (69, 12, 116, 1.1451917634147912), (36, 22, 116, 1.1689309086214899), (30, 15, 117, 1.1148946142790457), (30, 15, 118, 1.1346112842344382), (30, 15, 119, 1.1979448986724317), (57, 22, 120, 1.1565836661701867), (57, 22, 121, 1.0105754522897172), (57, 22, 122, 0.8812621307595289), (57, 22, 123, 0.9526903800493768), (57, 22, 124, 1.0556203520235248), (57, 22, 125, 1.0557055660616297), (57, 22, 126, 0.9356365697987399), (57, 22, 127, 1.0117216188318858), (46, 36, 127, 1.1484034056862473), (57, 15, 128, 1.195680362128217), (57, 15, 129, 1.03570208434499), (93, 72, 132, 1.1203216042206245), (93, 72, 133, 1.140367434789911), (93, 72, 134, 1.1616509758633633), (93, 72, 135, 1.178382599088097), (93, 72, 136, 1.189046398888172), (93, 72, 137, 1.194626545247978), (93, 72, 138, 1.1972104816055615), (98, 57, 188, 1.1613554320375832)]  
    data = fdata
    
    # 1.0318969691708757
    data =  [(0, 44, 77, 1.1538178889328234), (12, 69, 114, 1.122949310367536), (22, 36, 107, 1.0318969691708757), (22, 57, 124, 1.0557055660616297), (27, 30, 82, 1.1882525007967095), (30, 36, 108, 1.1942412099028386), (30, 85, 109, 1.1890254412162982), (46, 94, 100, 1.1997567498562682), (72, 93, 137, 1.1972104816055615), (82, 94, 103, 1.1817632342921571)]         
    
    # j改为 i 一个障碍物
    # 0.6193071164940586
    fdata=[(29, 13, 58, 1.1861377036521497), (30, 13, 59, 0.9417063894071848), (29, 13, 59, 1.1958785850765326), (30, 13, 60, 1.1019113040355193), (30, 13, 61, 1.098479388168175), (30, 13, 62, 1.0946963237217224), (29, 20, 62, 1.1708376100988467), (30, 13, 63, 1.0931075933041787), (29, 20, 63, 1.1479261550388038), (72, 22, 64, 1.182420491928968), (30, 13, 64, 1.093055674542321), (29, 20, 64, 1.1093207012405724), (30, 13, 65, 1.038755882931288), (72, 22, 66, 1.1416807974305443), (30, 13, 66, 1.1351934624682034), (51, 21, 68, 1.0461803386093365), (51, 21, 69, 1.1954157870219968), (30, 20, 69, 1.0581777263329437), (95, 30, 71, 1.1550659320514274), (95, 30, 72, 1.1820215776927434), (89, 13, 72, 0.979954528951752), (95, 30, 73, 0.9420932327107839), (89, 13, 73, 1.061931369905593), (95, 30, 74, 1.0740360428435245), (89, 13, 74, 1.0987483007778382), (20, 13, 74, 1.1879228421347094), (95, 30, 75, 1.0541591312220742), (89, 13, 75, 1.1383523878821078), (85, 30, 75, 1.124891389935356), (20, 13, 75, 1.1910840040849522), (95, 30, 76, 1.0570593331785292), (89, 20, 76, 1.1757294184774598), (89, 13, 76, 1.1378744752349752), (95, 30, 77, 0.8495648781473661), (89, 20, 77, 1.1898277274777813), (89, 13, 77, 0.8980834439217075), (64, 51, 77, 1.1017024977659855), (64, 13, 77, 1.152760523762488), (95, 30, 78, 0.7005676590643264), (89, 20, 78, 1.1590182216807665), (89, 13, 78, 0.8832780826726896), (64, 13, 78, 1.1279031662091727), (95, 30, 79, 0.6193071164940586), (89, 20, 79, 1.18985460614828), (89, 13, 79, 1.1213976834999542), (64, 13, 79, 1.178136403446272), (95, 30, 80, 0.6693016169023074), (64, 13, 80, 1.0875715249742848), (95, 30, 81, 0.8455953093446613), (93, 64, 81, 1.1319723385705311), (51, 13, 81, 1.149386993823086), (95, 30, 82, 1.1259127964904416), (85, 30, 83, 1.127403152369875), (85, 30, 84, 1.1331998055980697), (57, 13, 84, 1.081149381620228), (96, 4, 85, 1.191629247575488), (85, 30, 85, 1.134220097173112), (57, 13, 85, 1.1676118981732526), (56, 43, 85, 1.1830818622145538), (57, 13, 86, 0.9897971748479676), (58, 43, 87, 1.196830079076953), (57, 51, 87, 1.154145048211741), (57, 13, 87, 0.9770535784754655), (57, 51, 88, 1.1239806782295492), (57, 51, 89, 1.1657940700940204), (57, 51, 90, 1.1854510895397454), (57, 51, 91, 1.199548994079025), (95, 23, 92, 1.095372873856085), (57, 51, 92, 1.199531448390224), (95, 23, 93, 1.0559367432930888), (85, 25, 93, 1.196390483138668), (57, 51, 93, 1.1926460029065014), (95, 23, 94, 1.0940789539581817), (85, 64, 94, 1.1596093782484436), (51, 26, 94, 1.1621103038530582), (95, 23, 95, 1.091255079759472), (85, 64, 95, 1.165539862619304), (51, 26, 95, 1.0056628389192361), (95, 23, 96, 1.0912146565108065), (85, 64, 96, 1.1065270057771415), (57, 26, 96, 1.1831382512778745), (51, 26, 96, 0.8932829803852794), (7, 0, 96, 1.1979133686912213), (95, 23, 97, 1.0933170498794245), (85, 64, 97, 1.1628037365512254), (64, 26, 97, 1.1914962204511188), (57, 26, 97, 1.0382302359385447), (51, 26, 97, 0.950054728019843), (64, 26, 98, 1.1936046077873468), (64, 8, 98, 1.188158116890648), (57, 26, 98, 1.125786861659387), (51, 26, 98, 0.9488640062298708), (64, 8, 99, 1.0522419726114343), (57, 26, 99, 1.059545463531852), (51, 26, 99, 0.9007724450105289), (64, 8, 100, 0.9617413809990117), (57, 26, 100, 1.096261232035901), (57, 23, 100, 1.1845869280819608), (64, 8, 101, 0.8802194165131231), (57, 26, 101, 1.1060240797031957), (90, 43, 102, 1.07553982643812), (64, 51, 102, 1.1149071025283896), (64, 8, 102, 0.9079008846414193), (57, 26, 102, 1.0627056165256226), (90, 43, 103, 1.1210725258323038), (81, 58, 103, 1.1985664719968665), (64, 8, 103, 0.9072837605651533), (57, 26, 103, 1.1111391768123826), (90, 43, 104, 1.1723608074854839), (81, 58, 104, 1.1994671577230223), (64, 51, 104, 1.1786027984152145), (64, 8, 104, 0.7453977633559077), (25, 23, 104, 1.1843799215837714), (90, 43, 105, 1.1888022416101096), (64, 51, 105, 1.1884188979541073), (64, 8, 105, 0.9004588301494549), (25, 23, 105, 1.0375703514405614), (90, 43, 106, 1.1947634475131226), (25, 23, 106, 1.113145054067631), (25, 23, 107, 1.0520052022782778), (25, 23, 108, 1.09082464784142), (25, 23, 109, 1.0910528323617203), (43, 2, 112, 1.1454715597218286), (28, 0, 117, 1.1828455350107787), (28, 0, 118, 1.1547070502890104), (64, 41, 119, 1.1578814730316829), (28, 0, 119, 1.1992811483737875), (64, 41, 120, 1.1477877408126809), (28, 0, 120, 1.1846729610736602), (26, 1, 125, 0.998325087262047), (26, 1, 126, 0.9645423918204078), (26, 1, 127, 0.9179967627444426), (47, 1, 128, 1.1363061710432272), (26, 1, 128, 0.9439079936532504), (47, 1, 129, 1.0699080234315272), (26, 1, 129, 0.9429623059688544), (47, 1, 130, 1.0623473898391076), (26, 1, 130, 0.7430970241554475), (94, 1, 131, 1.1803222615047075), (85, 47, 131, 1.1283731803455028), (85, 1, 131, 1.1036767096071014), (47, 1, 131, 1.0651920050787396), (26, 1, 131, 0.9364748489310806), (85, 47, 132, 1.1275189615379226), (47, 1, 132, 0.8389532970822243), (85, 47, 133, 1.0486598631695028), (47, 1, 133, 0.8534655874265997), (85, 47, 134, 1.0356858221922725), (47, 1, 134, 1.1117314158367189), (85, 47, 135, 1.182068551362698), (94, 8, 137, 1.1691693684672093), (85, 1, 137, 1.1616186333629164), (94, 8, 138, 1.053024988474318), (85, 1, 138, 1.1971303514332257), (94, 8, 139, 1.1082746152054492), (85, 1, 139, 1.1847048439103256), (94, 8, 140, 1.1079324703769895), (85, 1, 140, 1.1685778603140933), (94, 8, 141, 0.8988097226049844), (94, 8, 142, 1.008415019391812), (47, 8, 142, 1.0192080728553192), (47, 8, 143, 1.1146456697259233), (47, 8, 144, 1.163067958506227), (5, 1, 144, 1.1735262450350894), (47, 8, 145, 1.1975310699564639), (5, 1, 145, 1.0340307680750587), (47, 8, 146, 1.1972993729617873), (5, 1, 146, 0.8517827646608587), (47, 8, 147, 1.109339146539383), (5, 1, 147, 0.8699156480196146), (47, 8, 148, 1.1739974972958018), (41, 34, 148, 1.1364534904234525), (5, 1, 148, 0.869892576123753), (41, 34, 149, 1.1883283096595738), (5, 1, 149, 0.7660358692695216), (41, 34, 150, 1.1943630344142189), (5, 1, 150, 1.1173225937407487), (41, 34, 151, 1.197297095509344), (41, 34, 152, 1.1984849381426945), (41, 34, 153, 1.1992077748560634), (41, 34, 154, 1.1995976691586374), (41, 34, 155, 1.199800971028484), (44, 8, 158, 1.1991167280415924), (94, 41, 160, 1.1113800307181445), (94, 41, 161, 1.16460077365735), (41, 1, 161, 1.1674207092362416), (94, 41, 162, 1.1689523700868203), (80, 8, 162, 1.1753686643837025), (41, 1, 162, 1.127346568280357), (80, 8, 163, 1.189061146920759), (41, 1, 163, 1.114825103912332), (94, 41, 164, 1.167453967737556), (41, 1, 164, 1.1141078101269601), (94, 41, 165, 1.0872266530971275), (41, 1, 165, 1.0936831841130645), (34, 3, 168, 1.1910965721035638), (63, 0, 170, 1.1706033193755647), (41, 9, 170, 1.1688756930220998), (63, 0, 171, 1.1721490545596498), (63, 0, 172, 1.1496084907745152), (63, 0, 174, 1.189246155870183), (63, 0, 175, 1.1905898255097942), (63, 0, 176, 1.1965292566174899), (94, 9, 180, 1.124958159944003), (72, 0, 183, 1.1921045845334852), (72, 0, 184, 1.1927272302181726), (72, 0, 185, 1.1456973704218667), (73, 9, 188, 1.1538207592668321), (73, 9, 189, 1.1284824855808726), (73, 9, 190, 1.1986542104027187), (73, 70, 196, 1.1788507399561428), (18, 10, 197, 1.1999999998299529), (11, 0, 257, 1.0525782790299045), (11, 0, 258, 1.0454933309245906), (11, 0, 259, 1.07013178098038), (11, 0, 260, 1.0712466818201813), (50, 0, 316, 1.1486146256603802), (50, 0, 317, 1.1726847490866223), (50, 0, 318, 1.1874425645862998), (50, 0, 321, 1.1922738135670026), (50, 0, 322, 1.195173315086242)] 
    data = fdata
    
    # 0.869892576123753
    data=[(0, 7, 95, 1.1979133686912213), (0, 11, 259, 1.0712466818201813), (0, 28, 118, 1.1992811483737875), (0, 50, 321, 1.195173315086242), (0, 63, 174, 1.1905898255097942), (0, 72, 183, 1.1927272302181726), (1, 5, 147, 0.869892576123753), (1, 26, 128, 0.9429623059688544), (1, 41, 163, 1.1141078101269601), (1, 47, 130, 1.0651920050787396), (2, 45, 130, 1.1779448866986875), (3, 34, 173, 1.1774442153046836), (3, 73, 182, 1.168133069115263), (3, 94, 189, 1.1969090754155642), (5, 93, 131, 1.1916892843265445), (8, 47, 145, 1.1972993729617873), (8, 64, 102, 0.9072837605651533), (8, 94, 139, 1.1079324703769895), (10, 18, 196, 1.1999999998299529), (13, 20, 74, 1.1910840040849522), (13, 26, 101, 1.146394070211286), (13, 29, 58, 1.1958785850765326), (13, 30, 63, 1.093055674542321), (13, 57, 88, 1.013360644158149), (13, 89, 75, 1.1378744752349752), (13, 93, 100, 1.1264620141933048), (14, 26, 92, 1.117542118969606), (14, 30, 71, 1.1580821492034887), (14, 64, 72, 1.1094808945309014), (14, 85, 90, 1.1573052767359606), (23, 25, 108, 1.0910528323617203), (23, 95, 95, 1.0912146565108065), (26, 51, 97, 0.9488640062298708), (26, 57, 100, 1.1060240797031957), (26, 64, 97, 1.1936046077873468), (29, 95, 68, 1.1703355131179634), (30, 85, 83, 1.1331998055980697), (30, 95, 75, 1.0570593331785292), (34, 41, 154, 1.199800971028484), (41, 94, 161, 1.1689523700868203), (45, 58, 103, 1.194062975807634), (47, 85, 131, 1.1275189615379226), (51, 57, 91, 1.199531448390224), (64, 85, 94, 1.165539862619304)]

    # m2 =3
    # 0.7305780885127487
    fdata=[(47, 0, 24, 1.195353454176876), (47, 0, 26, 1.0041435765951903), (47, 0, 27, 1.0473069991262849), (47, 0, 28, 1.0476115409208224), (47, 0, 29, 1.002779025524009), (47, 0, 30, 1.151950095387033), (14, 4, 85, 1.1890163121531203), (98, 30, 89, 1.156924224145074), (71, 51, 93, 1.1652915974421854), (71, 51, 94, 1.1559764215127433), (82, 45, 97, 1.1972361311682025), (82, 0, 109, 1.1077643159412878), (82, 0, 110, 0.8312833801830549), (82, 0, 111, 1.0056063132819846), (82, 0, 112, 0.892395974283168), (82, 0, 113, 0.8874982054553939), (82, 0, 114, 0.7305780885127487), (82, 0, 115, 0.9976546570154484), (55, 0, 124, 1.1581793623837529), (55, 0, 125, 1.1921925088235705), (55, 0, 126, 1.1917606033994947), (55, 0, 127, 1.1230382804403154), (55, 0, 128, 1.1099048358547083), (55, 0, 129, 1.1195092820527774), (55, 0, 130, 1.1347583355863538), (80, 72, 131, 1.1999439074129792), (55, 0, 131, 1.154957335099767), (55, 0, 132, 1.1835844705015577), (82, 53, 148, 1.0505324330901218), (82, 53, 149, 0.9641741723984635), (82, 53, 150, 1.0299784594287116), (82, 53, 151, 1.02236367418717), (82, 53, 152, 1.019610944173293), (82, 53, 153, 1.0195236666992362), (82, 53, 154, 0.9273972008784862), (82, 53, 155, 1.0262676742167804)]
    data = fdata
    # 0.8874982054553939
    data=[(0, 47, 27, 1.0476115409208224), (0, 55, 125, 1.1917606033994947), (0, 82, 112, 0.8874982054553939), (53, 82, 152, 1.0195236666992362)]

    # 只要半径内ob， 一个ob
    # 0.9210729112686505    
    fdata=[(0, 31, 117, 1.1529312518211026), (0, 65, 18, 0.9716784264194331), (2, 19, 57, 0.9907201766710728), (2, 22, 54, 1.0850913370350832), (2, 98, 58, 1.1597548351813296), (8, 10, 188, 1.1773144840421605), (8, 42, 178, 1.1481085612710544), (8, 92, 160, 1.0978076061606827), (10, 82, 110, 1.0991082520696895), (14, 56, 188, 1.1958059705089614), (19, 89, 234, 1.1457867818441365), (22, 45, 57, 1.1506339981287452), (22, 62, 230, 1.0446774047058665), (22, 89, 228, 1.158900457395305), (28, 35, 229, 1.100406687717043), (28, 47, 110, 1.1944705694022548), (28, 52, 254, 1.1506051086420834), (30, 42, 189, 1.0258050569843413), (30, 89, 197, 1.191727172838558), (35, 85, 225, 1.1294455661527585), (38, 49, 115, 1.083669618952194), (39, 48, 117, 1.1773612162442995), (39, 77, 127, 1.1916101003086246), (42, 92, 122, 1.1660372639326848), (47, 48, 111, 0.9210729112686505), (47, 85, 103, 1.1730331990838243), (53, 92, 156, 1.107711521404843), (53, 95, 145, 1.1781326353142998), (56, 59, 184, 0.9843524493637854), (77, 80, 111, 1.108682271553155), (77, 82, 121, 1.1553339619112764), (77, 85, 122, 1.003176944376011), (78, 98, 77, 1.109119283132315)]
    data=fdata

    # 0.5531882828506687 
    data=[(65, 0, 19, 0.9716784264194331), (65, 0, 20, 0.7225986757820307), (65, 0, 21, 0.6292382994440643), (65, 0, 22, 0.7460812160224665), (65, 0, 23, 1.0199560115287463), (76, 73, 45, 1.0963020370866545), (22, 2, 51, 1.1499731081179387), (22, 2, 52, 0.9348182273011698), (22, 2, 53, 1.1199328055204592), (22, 2, 54, 1.0872084772742787), (19, 2, 54, 1.1409339726111472), (98, 2, 55, 1.1811407361148951), (45, 22, 55, 1.1918299700435409), (22, 2, 55, 1.0850913370350832), (19, 2, 55, 1.10341254143725), (22, 2, 56, 0.6098581749347586), (19, 2, 56, 1.0943401230707623), (22, 19, 57, 1.160194063327076), (22, 2, 57, 0.5531882828506687), (19, 2, 57, 0.9860229344228006), (98, 2, 58, 1.1581772540924142), (45, 19, 58, 1.1415406278179365), (36, 2, 58, 1.159835673116397), (22, 2, 58, 0.7520976634677397), (19, 17, 58, 1.1969433926549757), (19, 2, 58, 0.9907201766710728), (98, 2, 59, 1.1597548351813296), (45, 19, 59, 1.1823673969514856), (36, 2, 59, 1.08610620773299), (22, 2, 59, 1.1304533504036227), (19, 2, 59, 0.6889222337966358), (45, 19, 60, 1.183701969015875), (36, 2, 60, 1.058180477718528), (19, 2, 60, 0.7330049096057528), (45, 19, 61, 1.13727741255238), (36, 2, 61, 1.1692595655286742), (19, 2, 61, 1.1114515919040213), (36, 2, 62, 1.191287490732443), (98, 17, 66, 1.1298202839470843), (98, 17, 67, 1.0578386400025446), (98, 17, 68, 1.15674787124461), (92, 75, 72, 1.1499016794736026), (92, 75, 73, 1.1815581067040892), (98, 78, 74, 1.1294981185598796), (98, 78, 75, 1.119340731886193), (98, 78, 76, 1.0969195742630407), (98, 78, 77, 1.1094259055886384), (98, 78, 78, 1.109119283132315), (98, 78, 79, 1.053950703011013), (51, 35, 80, 1.1586923270900173), (85, 47, 97, 1.153769841474135), (85, 47, 98, 1.1654059587846994), (31, 6, 98, 1.1961648284011879), (85, 47, 99, 1.1695692084456621), (85, 47, 100, 1.1720205746557881), (85, 47, 101, 1.1728791987091944), (85, 47, 102, 1.1729834782787871), (85, 47, 103, 1.1730028908463108), (85, 47, 104, 1.1730331990838243), (92, 20, 107, 1.159986524254704), (88, 54, 107, 1.1472777214810745), (82, 10, 107, 1.170599315771237), (80, 77, 107, 1.1841936308544063), (82, 10, 108, 0.903661112631392), (80, 77, 108, 0.9943635881167118), (48, 47, 108, 0.898162236404098), (82, 10, 109, 1.154615964177584), (80, 77, 109, 1.1059617368172743), (48, 47, 109, 0.9494133553963615), (82, 10, 110, 1.0990051762889217), (80, 77, 110, 1.1085041210840292), (48, 47, 110, 0.9082732166707459), (47, 28, 110, 1.1901245352211203), (82, 61, 111, 1.1741333036161967), (82, 10, 111, 1.0991082520696895), (80, 77, 111, 1.1086824359947198), (49, 38, 111, 1.0439773891773236), (48, 47, 111, 0.9212412509457534), (47, 28, 111, 1.1944705694022548), (82, 10, 112, 0.8434150154441222), (80, 77, 112, 1.108682271553155), (49, 38, 112, 0.8379119076980434), (48, 47, 112, 0.9210729112686505), (82, 10, 113, 0.678828079732653), (80, 77, 113, 1.1651831657685756), (49, 38, 113, 0.9358115435568571), (48, 47, 113, 0.7467739137939604), (82, 10, 114, 0.7003733428029341), (49, 38, 114, 0.9228731169921962), (48, 47, 114, 0.6970579551866196), (82, 10, 115, 0.9115456477465631), (49, 38, 115, 0.9164315431158521), (48, 47, 115, 0.7788987373029823), (49, 38, 116, 1.083669618952194), (48, 47, 116, 0.9828416510038238), (48, 39, 116, 1.1993739548647173), (47, 28, 116, 1.1668307195225098), (31, 0, 116, 1.1141160599057054), (82, 77, 117, 1.1737056277773348), (49, 38, 117, 0.9863796022963203), (48, 39, 117, 1.1699280075975869), (47, 28, 117, 1.1726872993878754), (31, 0, 117, 1.1475507549358503), (82, 77, 118, 0.9740002370320762), (49, 38, 118, 1.0220381551244435), (48, 39, 118, 1.1773612162442995), (31, 0, 118, 1.1529312518211026), (85, 77, 119, 1.037501376219602), (82, 77, 119, 1.0284061235978477), (49, 38, 119, 1.1893101430682604), (95, 20, 120, 1.154965082517872), (92, 42, 120, 1.1398019025920647), (85, 77, 120, 1.189288311505111), (82, 77, 120, 1.1216776130375359), (92, 42, 121, 1.1556101402963046), (85, 77, 121, 0.9879803098134685), (82, 77, 121, 1.1485360153706377), (92, 42, 122, 1.1582707637291343), (85, 77, 122, 1.0026480980454604), (82, 77, 122, 1.1553339619112764), (61, 28, 122, 1.1633910536036), (92, 42, 123, 1.1660372639326848), (92, 20, 123, 1.1992191134832113), (85, 77, 123, 1.003176944376011), (82, 77, 123, 1.1645406080061083), (92, 42, 124, 0.9486833404426939), (92, 20, 124, 1.1699462615202956), (85, 77, 124, 1.0016031185396301), (92, 42, 125, 0.83629513559219), (92, 20, 125, 1.188403916609361), (82, 39, 125, 1.1639040451743994), (77, 39, 125, 1.0285625977675852), (92, 42, 126, 0.8302518454167269), (82, 39, 126, 1.1812750639090257), (77, 39, 126, 1.1707619325936454), (48, 39, 126, 1.1981681540727933), (92, 42, 127, 0.9306606127179662), (77, 39, 127, 1.1866798290073377), (48, 39, 127, 1.1996674404976202), (95, 92, 128, 1.1954571791931916), (92, 42, 128, 1.1347756989081272), (77, 39, 128, 1.1916101003086246), (92, 38, 140, 1.1141192167792848), (95, 53, 141, 1.0394041969399554), (92, 38, 141, 1.078249291464696), (95, 53, 142, 1.118681359113048), (92, 38, 142, 1.0989978790885044), (95, 53, 143, 1.1302951336503861), (95, 53, 144, 1.1594387756458069), (95, 53, 145, 1.1780461601380507), (95, 53, 146, 1.1781326353142998), (92, 53, 153, 0.9979219346726448), (92, 53, 154, 1.030073027391856), (92, 53, 155, 1.015113015691083), (92, 53, 156, 1.1071250663839993), (92, 8, 156, 1.1418383845642692), (92, 53, 157, 1.107711521404843), (92, 8, 157, 1.1333781583384896), (92, 53, 158, 0.8204494652656616), (92, 8, 158, 1.1075855299679542), (92, 53, 159, 0.6666207264524705), (92, 8, 159, 1.1023162773306583), (92, 53, 160, 0.6757103482278802), (92, 8, 160, 1.1008241480282803), (92, 53, 161, 0.8467007692872721), (92, 8, 161, 1.0978076061606827), (91, 57, 161, 1.1658330634757266), (92, 53, 162, 1.1467762798393408), (92, 8, 162, 0.9924371576901028), (91, 57, 162, 1.1294649108692203), (53, 42, 162, 1.1940889292366992), (92, 8, 163, 1.0559775716925435), (92, 42, 171, 1.1520785885391092), (92, 42, 172, 1.0993217791042333), (92, 42, 173, 1.1578730249820741), (42, 8, 173, 1.179861290542948), (92, 42, 174, 1.181815982398667), (42, 8, 174, 1.103503744816785), (92, 42, 175, 1.193307262117471), (92, 10, 175, 1.156786523541349), (42, 8, 175, 1.0819187281417377), (92, 42, 176, 1.1878682458376488), (42, 8, 176, 1.1901812926117719), (92, 42, 177, 1.1898587492434392), (42, 8, 177, 1.1367294123667382), (92, 42, 178, 1.1615895892614017), (42, 8, 178, 1.148181220498775), (42, 8, 179, 1.1481085612710544), (42, 8, 180, 1.1646197809552536), (59, 56, 181, 1.0637460578632116), (59, 56, 182, 1.0564859190570686), (59, 56, 183, 0.9785299899819089), (10, 8, 183, 1.1788641043911714), (92, 47, 184, 1.1629740615908721), (59, 56, 184, 0.9851807222861662), (10, 8, 184, 1.1214567652453062), (92, 47, 185, 1.191682907616919), (59, 56, 185, 0.9843524493637854), (56, 14, 185, 1.1771024021556031), (56, 6, 185, 1.1865904237493938), (42, 30, 185, 1.1688200787019047), (10, 8, 185, 1.051123506560267), (59, 56, 186, 0.7743983815634286), (56, 14, 186, 1.1891246063446994), (56, 6, 186, 1.139141044821705), (42, 30, 186, 1.0305742295034153), (10, 8, 186, 1.0192164095391014), (59, 56, 187, 1.0049444119557396), (56, 14, 187, 1.1885573550149828), (42, 30, 187, 0.9302914429075564), (10, 8, 187, 1.1372503854004679), (56, 14, 188, 1.1918662824240387), (42, 30, 188, 1.166280290183971), (10, 8, 188, 1.1698558032931936), (56, 14, 189, 1.1958059705089614), (56, 6, 189, 1.0728016759839587), (10, 8, 189, 1.1773144840421605), (56, 14, 190, 1.144304968106452), (56, 35, 198, 1.1983178254484181), (89, 62, 214, 1.1391068805213775), (89, 62, 215, 1.014334829087566), (85, 35, 222, 1.1178429731122137), (85, 35, 223, 1.08713884764623), (85, 35, 224, 1.1401401366599957), (85, 35, 225, 1.1297206086771463), (35, 28, 225, 1.1282920161355232), (85, 35, 226, 1.1294455661527585), (85, 14, 226, 1.1879604852607502), (35, 28, 226, 1.1288496650158446), (85, 35, 227, 0.9007743723510585), (85, 14, 227, 1.1548282520602362), (35, 28, 227, 1.0139036496972607), (85, 35, 228, 0.9892614486065461), (85, 14, 228, 1.1243691761013712), (35, 28, 228, 1.10335939467515), (35, 28, 229, 1.1005605597461492), (89, 19, 230, 1.1370640888523775), (35, 28, 230, 1.100406687717043), (89, 19, 231, 0.9587646180055126), (35, 28, 231, 1.0000611435513953), (89, 19, 232, 0.9551688671964074), (35, 28, 232, 1.0686854956665461), (89, 19, 233, 1.1259642236201775), (89, 19, 234, 1.1457781939884493), (89, 19, 235, 1.1457867818441365), (89, 19, 236, 1.1072123544246404), (89, 19, 237, 1.1684737822669342), (52, 28, 251, 1.0908959874554491), (52, 28, 252, 0.8760248795094215), (52, 28, 253, 1.1440741032532376), (52, 28, 254, 1.151246768806483), (52, 28, 255, 1.1506051086420834), (52, 28, 256, 0.9558445244077178), (52, 14, 256, 1.169532478637966), (52, 28, 257, 1.1572298249569684), (52, 14, 257, 1.1921237062741226), (47, 1, 303, 1.0630060321582444), (47, 1, 304, 1.1702506631736245)]

    # 只要半径内ob， 3个ob
    # 1.1871785501038643 
    fdata= [(97, 25, 119, 1.1871785501038643), (97, 25, 120, 1.1943286268417574)]
    data =fdata
    # 1.1740473006257601
    data=[(25, 97, 119, 1.1943286268417574), (80, 97, 118, 1.1740473006257601)]
    
    # agent之间，只要半径内 1ob
    # 0.61523958537585
    fdata= [(86, 49, 110, 1.1502491936658117), (84, 2, 110, 1.0717735561892034), (86, 23, 116, 1.1723004181051455), (66, 4, 116, 1.0570781048146451), (86, 23, 117, 1.1212407101984572), (66, 4, 117, 0.9792906216218829), (86, 23, 118, 1.1686034073887366), (66, 4, 118, 1.1088192713428102), (86, 23, 119, 1.1747750319291361), (86, 5, 119, 1.173064636878764), (66, 4, 119, 1.065408177367428), (86, 23, 120, 1.174957184565347), (86, 5, 120, 1.179552341947829), (66, 4, 120, 1.0657479078198462), (86, 23, 121, 1.1332379010095455), (86, 5, 121, 1.1840683825762008), (66, 4, 121, 0.8733709886914425), (95, 21, 122, 1.108931486748587), (86, 23, 122, 1.1883401556258146), (66, 4, 122, 0.9305358102060506), (23, 5, 122, 1.1931678018989715), (95, 21, 123, 0.9264026483557094), (94, 86, 123, 1.1019835169150136), (38, 5, 123, 1.1721243046330687), (23, 5, 123, 0.98396252038147), (95, 21, 124, 1.1205310927434369), (94, 86, 124, 0.9824082942796536), (73, 66, 124, 1.1470023040103912), (66, 21, 124, 1.181609791382539), (38, 5, 124, 1.1324448909546998), (23, 5, 124, 1.097275604600337), (95, 21, 125, 1.1302888824034734), (94, 86, 125, 0.9548429528867207), (94, 38, 125, 1.1877764665057184), (73, 66, 125, 1.1505452342074416), (66, 21, 125, 1.0782651051361725), (38, 5, 125, 1.1379723301990583), (23, 5, 125, 1.1743720834908444), (95, 21, 126, 1.133770470364964), (94, 86, 126, 1.1488413433033884), (94, 38, 126, 1.152035125764852), (94, 5, 126, 1.1978535116900735), (66, 21, 126, 1.17002680338441), (38, 5, 126, 1.1492580256052938), (23, 5, 126, 1.1756147096986669), (95, 21, 127, 1.1344179639545973), (94, 86, 127, 1.1529529324373007), (94, 38, 127, 1.1748383156334368), (66, 21, 127, 1.1654383238310695), (38, 5, 127, 1.1544488258349337), (25, 23, 127, 1.1641777191704723), (23, 5, 127, 1.158904732915539), (94, 86, 128, 1.1520068681930793), (94, 38, 128, 1.17686172486218), (94, 23, 128, 1.1730138704067157), (68, 1, 128, 1.1595799829046247), (66, 21, 128, 1.1630414083899452), (38, 5, 128, 1.0803195801803709), (94, 86, 129, 1.0899735888149928), (94, 38, 129, 1.0662521571456351), (94, 23, 129, 1.09866213101824), (66, 21, 129, 1.1629952146140625), (38, 5, 129, 1.1189280123397238), (94, 86, 130, 1.1438457871914816), (94, 38, 130, 1.0618700885921413), (94, 23, 130, 1.1383806105096632), (68, 1, 130, 0.954335967178914), (66, 21, 130, 1.0681746804059573), (38, 4, 130, 1.1474709260637315), (94, 38, 131, 1.162865678201946), (94, 23, 131, 1.1428837487919903), (68, 1, 131, 1.0207964105561804), (66, 21, 131, 1.1364596415204178), (38, 4, 131, 1.14773824858624), (94, 23, 132, 0.8847160722062883), (68, 1, 132, 1.01797285962575), (50, 48, 132, 1.0555950719907532), (38, 4, 132, 1.1176364226836495), (94, 23, 133, 0.7756516836072935), (68, 1, 133, 1.0178675824291772), (38, 4, 133, 1.1437078756027403), (25, 5, 133, 1.198401034447957), (94, 23, 134, 0.8503276106147447), (68, 1, 134, 1.0164117907720622), (38, 4, 134, 1.174087512484234), (25, 23, 134, 1.199534225612674), (25, 5, 134, 1.173543893933215), (94, 86, 135, 1.1921050365522594), (94, 23, 135, 1.0904992395869295), (38, 4, 135, 1.1841959046861503), (25, 23, 135, 1.1530864407485317), (94, 86, 136, 1.1960515259549975), (38, 4, 136, 1.1842017537190475), (25, 5, 136, 1.1930149930310268), (49, 5, 137, 1.1998192165684174), (43, 25, 137, 1.1626417546769403), (38, 4, 137, 1.1853833948914358), (25, 5, 137, 1.190906323532705), (49, 5, 138, 1.1761808218203478), (43, 25, 138, 1.1652303865098796), (49, 5, 139, 1.1786228912113235), (43, 25, 139, 1.161172082136698), (49, 5, 140, 1.1968825432539547), (43, 25, 140, 1.099554732760408), (43, 39, 141, 1.1941559674104838), (43, 25, 141, 1.137662115175643), (38, 25, 141, 1.1876642232716625), (71, 43, 142, 1.0530378275201795), (43, 39, 142, 1.1970225533251035), (38, 25, 142, 0.9558943674630632), (71, 43, 143, 1.1767882335085083), (38, 25, 143, 1.1481935062646897), (71, 43, 144, 1.1947181240555225), (38, 25, 144, 1.1489618333888973), (71, 43, 145, 1.183789672208074), (43, 38, 145, 1.187242303175876), (39, 38, 145, 1.1302568200351966), (38, 25, 145, 1.145799093854525), (71, 43, 146, 1.191336283446197), (43, 38, 146, 1.0733023557499415), (39, 38, 146, 1.0534361507468781), (73, 25, 147, 1.1334388118983556), (71, 43, 147, 1.1934581881027095), (43, 38, 147, 1.1922697545182566), (39, 38, 147, 1.0428049099715566), (73, 39, 148, 1.1752255201561579), (73, 25, 148, 1.0521363144542648), (71, 43, 148, 1.1935459880082735), (43, 38, 148, 1.193651457963264), (39, 38, 148, 1.0459905112969337), (73, 39, 149, 0.967138596467478), (73, 25, 149, 1.09780471014203), (71, 43, 149, 1.1847915471186328), (71, 38, 149, 1.0830016505230189), (43, 38, 149, 1.1935576767005402), (39, 38, 149, 1.1299093012059762), (73, 39, 150, 1.0153288664158204), (71, 38, 150, 1.0346950628828377), (43, 38, 150, 1.1906159975931188), (39, 38, 150, 1.1356678513742382), (73, 39, 151, 0.8972391136154934), (71, 38, 151, 0.969192303271441), (43, 38, 151, 1.1758091720016397), (39, 38, 151, 1.1860052460074795), (73, 39, 152, 0.8987515141763068), (71, 38, 152, 1.006253055908968), (39, 21, 152, 1.1581891739170993), (73, 39, 153, 0.9035389245332881), (73, 5, 153, 1.1837351659961555), (71, 49, 153, 1.1781406490288011), (71, 38, 153, 1.0133419236136925), (43, 38, 153, 1.1980922730587633), (39, 21, 153, 1.1960301846498114), (73, 39, 154, 0.61523958537585), (71, 49, 154, 1.1391850770831602), (71, 38, 154, 0.8935959655489653), (39, 21, 154, 1.0218510522686715), (73, 39, 155, 0.707687983374316), (39, 21, 155, 1.070467306714501), (73, 39, 156, 1.1085622119724836), (73, 5, 156, 1.1397901061888238), (39, 21, 156, 1.1761842667722655), (73, 5, 157, 1.1552514335777662), (39, 21, 157, 1.177465334429543), (38, 10, 157, 1.1685915208299582), (43, 21, 158, 1.062967364700909), (38, 10, 158, 1.0230438197998153), (43, 21, 159, 0.966621382576117), (38, 10, 159, 1.0042207123993232), (73, 5, 160, 1.1898399265529462), (43, 21, 160, 0.8018539068426888), (38, 10, 160, 0.9819230150241265), (73, 5, 161, 1.1971045841108439), (43, 21, 161, 1.176911654628532), (38, 10, 161, 1.0827483073987412), (73, 5, 162, 1.1968229807159667), (43, 21, 162, 1.181089420826827), (38, 21, 162, 1.0758071073097928), (38, 10, 162, 1.0828431136279915), (73, 5, 163, 1.1983737744176561), (43, 21, 163, 0.7606937041989432), (38, 21, 163, 1.0365787649716116), (38, 10, 163, 1.0553448209060114), (73, 5, 164, 1.1992158525030783), (43, 21, 164, 0.6407473295884013), (38, 21, 164, 1.0800643518551691), (38, 10, 164, 1.144202025186107), (43, 21, 165, 0.6898836468136296), (38, 21, 165, 1.0788011334436178), (43, 21, 166, 0.8938932495109333), (38, 21, 166, 0.9529286944946619), (43, 21, 167, 1.069318711257609), (38, 21, 167, 0.92176276077951), (73, 21, 168, 1.1960561097630555), (38, 21, 168, 1.0063483513841076), (73, 21, 169, 1.194689268006848), (38, 21, 169, 1.1992469880937129), (26, 25, 171, 1.137562922472577), (26, 25, 172, 1.1399088466112044), (26, 25, 173, 1.1826727893719626), (82, 10, 174, 1.1565125633697704), (26, 25, 174, 1.1722893528757676), (82, 10, 175, 0.8735484158926117), (26, 25, 175, 1.17190235855108), (82, 10, 176, 1.0082593931293675), (26, 25, 176, 0.9894842835429549), (82, 10, 177, 1.1554870530635946), (26, 25, 177, 1.1617554288551415), (82, 10, 178, 1.1716903494313862), (82, 10, 179, 1.1748197704479952), (82, 10, 180, 1.1722000518807145), (53, 1, 196, 1.0903045327584204), (53, 1, 197, 1.124363784939214), (53, 1, 198, 1.1699781971772587), (53, 1, 199, 1.1920892478815288), (53, 1, 200, 1.19822174035022), (31, 1, 200, 1.1976286743935542), (31, 1, 201, 1.1940943971452966), (95, 1, 206, 1.1316832186261987), (95, 1, 207, 1.1307710877552426), (95, 1, 208, 1.1400564673140399)]
    data=fdata
    # 0.8791888771278974  
    data=[(0, 15, 12, 0.8791888771278974), (0, 62, 79, 1.1948257299766942), (0, 84, 63, 1.153468363515471), (1, 53, 199, 1.19822174035022), (1, 68, 132, 1.0178675824291772), (1, 95, 207, 1.1400564673140399), (4, 38, 135, 1.1842017537190475), (4, 66, 119, 1.0657479078198462), (5, 23, 125, 1.1756147096986669), (5, 25, 136, 1.190906323532705), (5, 38, 126, 1.1544488258349337), (5, 73, 163, 1.1992158525030783), (5, 86, 120, 1.1840683825762008), (10, 38, 161, 1.0828431136279915), (10, 82, 178, 1.1748197704479952), (21, 38, 164, 1.0788011334436178), (21, 39, 156, 1.177465334429543), (21, 43, 161, 1.181089420826827), (21, 66, 128, 1.1629952146140625), (21, 73, 168, 1.194689268006848), (21, 95, 126, 1.1344179639545973), (23, 25, 135, 1.0297987025757325), (23, 29, 92, 1.197515942233589), (23, 73, 143, 1.0801677083648518), (23, 77, 94, 1.0211105011607116), (23, 78, 100, 1.130070695189656), (23, 86, 119, 1.174957184565347), (23, 94, 130, 1.1428837487919903), (25, 26, 174, 1.17190235855108), (25, 38, 143, 1.1489618333888973), (25, 43, 138, 1.161172082136698), (29, 49, 93, 1.1993455108488738), (31, 84, 104, 1.1961939930802987), (38, 39, 149, 1.1356678513742382), (38, 43, 149, 1.1906159975931188), (38, 71, 152, 1.0133419236136925), (38, 73, 156, 1.042475107016209), (38, 94, 127, 1.17686172486218), (39, 43, 141, 1.1970225533251035), (39, 73, 152, 0.9035389245332881), (39, 77, 104, 1.198383661744925), (41, 48, 105, 1.1591057611711781), (43, 71, 147, 1.1935459880082735), (53, 73, 108, 1.1052791683689165), (58, 84, 96, 1.1418748551961238), (73, 82, 169, 1.148405885315749), (86, 94, 127, 1.1520068681930793)]

    # 3个ob
    # 0.9219815605505202
    fdata= [(79, 14, 35, 1.0414780741890914), (82, 63, 88, 1.098941280151197), (82, 63, 89, 1.1995412520936197), (82, 63, 90, 1.1315624036855623), (82, 48, 90, 1.1856208610667156), (47, 35, 90, 1.1921361068455572), (82, 63, 91, 1.1561019510553396), (82, 48, 91, 1.1511019687338944), (96, 80, 92, 1.1565148448038707), (82, 63, 92, 1.1569159310641337), (82, 48, 92, 1.1540914535964295), (96, 80, 93, 1.0169738944449), (82, 63, 93, 0.9863415607117447), (82, 48, 93, 1.158644326318465), (96, 80, 94, 1.021722656174796), (82, 63, 94, 0.9219815605505202), (82, 48, 94, 1.169707359969141), (96, 80, 95, 1.1746615286686541), (82, 63, 95, 0.9639875984179954), (55, 38, 95, 1.1967057908570529), (82, 63, 96, 1.1106262941470848), (63, 47, 96, 1.1575054197096677), (57, 38, 96, 1.1943221282123822), (55, 38, 96, 1.1961477017363653), (63, 47, 97, 1.1652079219115274), (57, 38, 97, 1.1864174062312234), (55, 38, 97, 1.1254795863437235), (55, 38, 98, 1.1336369516637), (55, 38, 99, 1.1434164417971509), (57, 38, 100, 1.1992038765423856), (55, 38, 100, 1.1464515201408751), (57, 38, 101, 1.1981158748811378), (55, 38, 101, 1.1471847482702926), (57, 45, 102, 1.1957017990213925), (57, 38, 102, 1.173896623400071), (57, 32, 102, 1.1995748856039967), (85, 16, 103, 1.191983705521891), (57, 45, 103, 1.1942987915099168), (85, 70, 104, 1.1129944168776207), (85, 16, 104, 1.0950641476723701), (57, 45, 104, 1.1381447438935348), (85, 70, 105, 0.9771655809976414), (85, 16, 105, 1.1490004745362905), (85, 70, 106, 1.0543017926197304), (85, 16, 106, 1.0554494672576895), (85, 70, 107, 1.0466835228824685), (85, 16, 107, 1.0691236918231186), (85, 70, 108, 1.0463386342885226), (85, 16, 108, 1.068317531293716), (45, 38, 108, 1.1773708154651128), (85, 70, 109, 1.0461471537575269), (85, 16, 109, 0.9366629919429909), (85, 70, 110, 0.9268614731795517), (85, 16, 110, 0.9457280532339145), (85, 70, 111, 0.9801151443827006), (85, 16, 111, 1.0616825652641368), (85, 70, 112, 1.1959484830110987), (74, 22, 112, 1.16869126345572), (83, 69, 113, 1.1992369203487072), (83, 69, 114, 1.1826268768777481), (70, 2, 122, 1.1066409953367384), (70, 2, 123, 1.1479161985456585), (70, 2, 124, 1.145877793864418), (70, 2, 125, 1.1536110937566617), (48, 22, 125, 1.1359925486527707), (70, 2, 126, 1.1719277252849727), (70, 2, 127, 1.17406905375274), (70, 2, 128, 1.1741813427997077), (96, 51, 129, 1.1768706730744138), (45, 1, 135, 1.1752826757249524), (45, 1, 136, 1.1746085027418516), (45, 1, 137, 1.1922357147659652), (45, 1, 138, 1.175182023919113), (87, 0, 144, 1.186681674297201), (87, 0, 145, 1.147176172324342), (87, 0, 146, 1.1628574812001324), (87, 0, 147, 1.1714953477788281), (87, 0, 148, 1.1809435002418684), (87, 0, 149, 1.179022531135941), (87, 0, 150, 1.1109263430352976), (87, 0, 151, 1.121071697852986)]
    data =fdata

    # 1.0461471537575269
    data = [(0, 87, 148, 1.179022531135941), (2, 70, 127, 1.1741813427997077), (8, 30, 37, 1.1423874289297404), (16, 85, 107, 1.068317531293716), (32, 57, 101, 1.1995748856039967), (38, 55, 100, 1.1471847482702926), (38, 57, 100, 1.1981158748811378), (47, 63, 96, 1.1652079219115274), (48, 82, 92, 1.158644326318465), (56, 83, 111, 1.1735086838512638), (63, 82, 91, 1.1569159310641337), (69, 83, 112, 1.1992369203487072), (70, 85, 108, 1.0461471537575269), (80, 96, 91, 1.1565148448038707)]
    
    
    # ob加入半径 * 2 ， 1 ob
    # 0.8869893796489141
    fdata=[(75, 63, 120, 1.1972119340478191), (72, 26, 130, 1.1586892765254098), (39, 20, 137, 1.177604081110779), (81, 75, 139, 1.0849581186221435), (81, 75, 140, 0.9355813831957204), (81, 75, 141, 0.9880299330483262), (81, 75, 142, 0.9816869999095762), (81, 75, 143, 0.9773989565065114), (81, 75, 144, 0.9772412506219537), (81, 75, 145, 0.9215437168265485), (81, 75, 146, 1.0720764924684902), (95, 37, 147, 1.131138956235799), (95, 37, 148, 1.1802806715001728), (61, 39, 152, 1.1865514087180946), (70, 53, 153, 1.0012009059365454), (92, 37, 154, 1.1918804127630334), (70, 53, 154, 1.1252522565997662), (70, 53, 155, 1.0309676550151392), (70, 53, 156, 1.0409205293456927), (70, 53, 157, 1.0403025483124477), (60, 39, 157, 1.1758100811177887), (70, 53, 158, 0.8869893796489141), (81, 53, 159, 1.1658938136696602), (70, 53, 159, 1.113252151480951), (81, 53, 160, 1.1729562380155156), (23, 13, 168, 1.171824608383145), (23, 13, 169, 1.09099238892874), (95, 23, 170, 1.1555266112861093), (23, 13, 170, 1.1643604904193257), (23, 13, 171, 1.1687521706662398), (23, 13, 172, 1.1692256462984267), (95, 62, 195, 1.1837278165101948), (82, 23, 198, 1.1558945477134475), (82, 23, 199, 1.1910072779940857), (82, 23, 200, 1.191168064743403), (82, 23, 201, 1.1912272956956418), (82, 23, 202, 1.1912231634893273), (82, 23, 203, 1.1912199888331128), (82, 23, 204, 1.1912196746531218), (82, 23, 205, 1.1343762323619184), (82, 23, 206, 1.1766716012455913), (18, 15, 257, 1.143478099627195), (18, 15, 258, 1.131979549137667), (77, 15, 266, 1.1676794599925866), (77, 15, 267, 1.1335096209847928)] 
    data=fdata
    
    # 加入 最少一个障碍物约束
    # 0.959484116269048  
    data=[(0, 27, 16, 0.959484116269048), (1, 15, 45, 1.0551142321042133), (5, 63, 59, 1.1120369234915335), (10, 30, 46, 1.1373951420237474), (13, 23, 171, 1.1692256462984267), (15, 80, 70, 1.1426731270725103), (23, 82, 203, 1.1912196746531218), (25, 63, 115, 1.1718879502405186), (25, 81, 147, 1.1675572360757658), (43, 44, 108, 1.1843730288470729), (44, 99, 98, 1.1794617493704205), (53, 70, 156, 1.0403025483124477), (53, 81, 159, 1.1729562380155156), (75, 81, 143, 0.9772412506219537)]
    
    # 0.6048829108465662
    fdata=[(71, 0, 38, 1.1817489541256034), (71, 0, 39, 1.1342943061964061), (71, 0, 40, 1.1991656956085261), (27, 0, 51, 1.1799378793125987), (27, 0, 52, 1.1282031247089872), (27, 0, 53, 1.1426887324111883), (27, 0, 54, 1.1422711211263987), (27, 0, 55, 1.1721532228219294), (82, 56, 67, 1.1328567347380216), (82, 56, 68, 1.0518063646813447), (50, 34, 69, 1.05823931612961), (50, 34, 70, 1.0914602017462005), (50, 34, 71, 1.08785143356522), (39, 20, 71, 1.1575295843039903), (50, 34, 72, 1.0812912435483992), (59, 50, 73, 1.1990840302486119), (50, 34, 73, 1.0880154101992032), (39, 20, 73, 1.167928810940439), (50, 34, 74, 0.9101713910493845), (39, 20, 74, 1.1787474897179577), (50, 34, 75, 0.9592096834330521), (44, 32, 117, 1.1372770885341208), (44, 32, 118, 0.8404070895370686), (44, 32, 119, 0.8956924266986217), (89, 32, 120, 1.0854063761647428), (44, 32, 120, 0.8802459251091291), (89, 32, 121, 1.1934595945447162), (44, 32, 121, 0.880098587689787), (89, 32, 122, 1.1877377003059342), (44, 32, 122, 0.6048829108465662), (93, 44, 123, 1.0886941418361105), (44, 32, 123, 0.6442369025875001), (89, 32, 124, 1.1976624851871542), (76, 44, 124, 1.1504385740076413), (44, 32, 124, 0.9755370089442017), (89, 32, 125, 1.1796806949981637), (89, 44, 127, 1.1723624138019177), (89, 44, 128, 1.186072607177346), (42, 21, 128, 1.16662008498526), (77, 35, 129, 1.1033311885174486), (42, 21, 129, 1.1584045419920728), (77, 35, 130, 1.193859269422075), (42, 35, 130, 1.138567180931873), (42, 21, 130, 1.189159699381964), (77, 35, 131, 1.1869199829630552), (42, 35, 131, 1.1460080355225075), (77, 35, 132, 1.1841834046739486), (42, 35, 132, 1.1511299528571266), (35, 21, 132, 1.1958913053919358), (77, 35, 133, 1.1838361152004107), (42, 35, 133, 1.1523117359154802), (42, 21, 133, 1.1918017806341337), (35, 21, 133, 1.1794244064758004), (77, 35, 134, 1.183768901868168), (42, 35, 134, 1.148339525052178), (42, 21, 134, 1.1785363160107116), (35, 21, 134, 1.1643857659410335), (77, 35, 135, 1.1231766804493637), (42, 35, 135, 1.1472432462394957), (42, 21, 135, 1.1775335534078564), (35, 21, 135, 1.160973350607148), (42, 35, 136, 1.1413167195790073), (42, 21, 136, 1.179231331213377), (35, 21, 136, 1.160686668553902), (35, 21, 137, 1.145607627274375), (32, 1, 156, 1.0698350655493534), (32, 1, 157, 1.034806382822926), (32, 1, 158, 0.954098785788824), (92, 32, 159, 1.1761260012904386), (32, 1, 159, 1.1845490834247858), (92, 32, 160, 1.1977867067079), (32, 1, 160, 1.1814017253685631), (92, 32, 161, 1.1974413065764), (32, 1, 161, 0.8778424336770234), (92, 32, 162, 1.1817092418114694), (32, 1, 162, 0.7815422762575126), (32, 1, 163, 0.9497903526902509), (32, 1, 164, 0.984278803790696), (32, 1, 165, 1.1039950570476058)]
    data=fdata
    
    
    # 0.880098587689787
    data=[(0, 27, 53, 1.1422711211263987), (1, 32, 159, 1.1814017253685631), (20, 39, 73, 1.1787474897179577), (21, 35, 135, 1.160686668553902), (21, 42, 134, 1.1775335534078564), (27, 67, 41, 1.1806364280549333), (32, 44, 120, 0.880098587689787), (32, 89, 123, 1.1976624851871542), (32, 92, 160, 1.1974413065764), (34, 50, 72, 1.0880154101992032), (35, 42, 134, 1.1472432462394957), (35, 77, 133, 1.183768901868168), (44, 76, 130, 0.9814412256858042), (44, 93, 131, 1.1928294761681537), (50, 59, 72, 1.1990840302486119), (50, 98, 98, 1.1993014338206796), (63, 86, 77, 1.1434527557954774)]
    
    # -1
    #  0.9675571599285329
    # -1
    # 0.824166272881022
    
    # 再测一次
    # 1ob
    # 0.42229998303124866 
    # 0.844445662434103
    
    # 20ob
    # 0.8990436368984871
    # 1.013921610549037
    
    min_value = min(row[3] for row in data)
    print(min_value)
    exit()

if __name__ == "__main__":
    # for i in range(1, 5):
    #     print('********************************   ',
    #           i, ' *********************************')
    #     Local_File_Test(i)
    test_statistics_n_round_test()

    pass
