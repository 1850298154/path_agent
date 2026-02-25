import numpy as np
import zrand as zr
import SET


first = True
inter_r_multi=2

# def inCollisionRadius(i_p, j_p):

#     # 速度最大值没有输入， 不能直接用
#     # 应该求一下当前速度
#     # theorem_4 = (2 * (SET.ini_v[i][0]
#     #                   + SET.ini_v[i][1]
#     #                   ) * SET.K
#     #              + SET.r_min
#     #              + 2*SET.epsilon)
#     # theorem_4 = (2 * SET.Vmax * SET.K * SET.h
#     theorem_4 = (2 * SET.Vmax * SET.K
#                  + SET.r_min
#                  + 2*SET.epsilon)
#     # theorem_4 /= 2
#     # theorem_4 /= 8  # 程序无解cvxpy， 崩溃
#     global first
#     if first:
#         print(
#             f'(2 * SET.Vmax{SET.Vmax} * SET.K{SET.K} * SET.h{SET.h} + SET.r_min{SET.r_min}+ 2*SET.epsilon{SET.epsilon})  = ')
#         print('******************************************  theorem_4 = ', theorem_4)
#         first = False
#     # print(type(i_p))
#     # print(type(j_p))
#     # print(i_p)
#     # print(j_p)
#     if (np.linalg.norm(i_p - j_p) < theorem_4):
#         return True
#     else:
#         return False


def sort_radius(index, i_p, obstacle_list):
    dis_j_list = []
    # del obstacle_list[index] # 千万不能直接del， 后续会出 bug， mosek error， 因为其他地方还用到了这个
    obstacle_list = [obstacle_list[i]
                     for i in range(len(obstacle_list))
                     if i != index]
    # # print('len(obstacle_list) ---------- ', len(obstacle_list))
    # for j in range(0, len(obstacle_list)):  # j is the index of other agent

    #     # if(j == index): # 都 del obstacle_list[index] , 就不用判断，否则会多去掉一个，万一这个agent[j]是最近的一个呢？
    #     #     continue

    #     P_j = obstacle_list[j]  # P_j: 表示第j个agent的预测轨迹。
    #     # if(inCollisionRadius(i_p=i_p, j_p=P_j[0])):
    #     # 23-08-22 测试觉得，停不下来，所以用预测的下一步进行判断。
    #     if(inCollisionRadius(i_p=i_p, j_p=P_j[1])):
    #         dis_j_list.append(obstacle_list[j])
    #     else:
    #         continue
    dis_j_list=obstacle_list
    sort_dis_j_list = []
    if len(dis_j_list) > 0:
        # print(type(dis_j_list))
        # print(dis_j_list)
        # print(type(dis_j_list[0]))  # 越界
        # print(dis_j_list[0])
        # print(type(dis_j_list[0][0]))
        # print(dis_j_list[0][0])
        sort_dis_j_list = sorted(
            dis_j_list, key=lambda x: np.linalg.norm(i_p - x[0]))

    return sort_dis_j_list
    # return sort_dis_j_list[:zr.m]


cnt_inCollision_list = []


# this code is appropirate for both 2D and 3D
def Get_inter_cons(agent, share_data):

    P = agent.pre_traj
    target = agent.target
    K = agent.K
    D = agent.D
    eta = agent.eta
    r_min = agent.r_min
    index = agent.index
    term_overlap = agent.term_overlap
    p = agent.p
    rho_0 = agent.rho_0
    epsilon = agent.epsilon
    W_list = agent.W

    cons_A = np.zeros((1, D*K))
    cons_B = np.array([-1.0])
    cons_C = np.zeros((1, 1))
    RHO = np.array([0.0])

    pre_traj_list = share_data['pre_traj']
    priority_list = share_data['priority']

    # l_max = 2*agent.Vmax*agent.h*agent.K + agent.r_min + 2*agent.epsilon
    # # l_max = 200
    # print('l_max')
    # print(l_max)
    # len_agent = len(pre_traj_list)
    # print(len_agent)
    # ignore = []

    # # l_max = 2*agent.Vmax*agent.h*agent.K + agent.r_min + 2*agent.epsilon
    # l_max = 2*agent.Vmax * agent.K + agent.r_min + 2*agent.epsilon
    # # print('l_max')
    # # print(l_max)
    # # len_agent = len(pre_traj_list)
    # # print(len_agent)
    # # ignore_j_list = []
    # # all_r_constrain = []

    # pre_traj_list = sort_radius(index, P[1], pre_traj_list)
    sorted_obstacles = sort_radius(index, P[0], pre_traj_list)
    i=0 #如果没有障碍物 ， for 循环就不会产生i变量
    for i in range(len(sorted_obstacles)):
        if np.linalg.norm(sorted_obstacles[i][0]-P[0]) > inter_r_multi*agent.r_max:
            break
    pre_traj_list=sorted_obstacles[:min(max(i,2),zr.m)]
    # if SET.Num <= 2:
    #     pre_traj_list=sorted_obstacles[:min(max(i,1),zr.m)]
    # elif SET.Num >= 3: # 最少俩个邻居， 加上自己 一共 3个agent
    #     pre_traj_list=sorted_obstacles[:min(max(i,2),zr.m)]
    cnt_inCollision = len(pre_traj_list)
    # for j in range(0, len_agent):  # j is the index of other agent
    for j in range(0, len(pre_traj_list)):  # j is the index of other agent

        # if j == index:  # 在sort_radius中 del index了
        #     continue

        P_j = pre_traj_list[j]
        # # if(inCollisionRadius(i_p=P[1], j_p=P_j[1])):
        # if(inCollisionRadius(i_p=P[0], j_p=P_j[0])):
        #     cnt_inCollision += 1
        # else:
        #     continue

        # r_constrain = np.linalg.norm(p-P_j[0])
        # # print(r_constrain)
        # # all_r_constrain.append(r_constrain)
        # # the agent out of rang is negelected
        # # if r_constrain > l_max:
        # if r_constrain > agent.r_max:
        #     # print(j)
        #     # ignore_j_list.append(j)
        #     continue

        for t in range(1, len(P)):
            p = P[t]
            l = len(P_j)
            if(t >= l):
                p_j = P_j[-1]
            else:
                p_j = P_j[t]

            if(t == len(P)-1):
                a, b, rho = MBVC_WB(p, p_j, target, r_min,
                                    eta, rho_0, term_overlap)

                if priority_list[j] > priority_list[index]:
                    rho = 10*rho_0/epsilon**2
                elif priority_list[j] < priority_list[index]:
                    rho = 0.01*rho_0/epsilon**2
                else:
                    if j < index:
                        W_j = W_list[j+1]
                    else:
                        W_j = W_list[j]
                    rho /= W_j/2/epsilon

            else:
                a, b = MBVC(p, p_j, r_min)

            # add constraints
            cons_a = []
            for i in range(0, len(P)-1):
                if(i == t-1):
                    cons_a = np.append(cons_a, a)
                else:
                    cons_a = np.append(cons_a, np.zeros(D))

            cons_A = np.row_stack((cons_A, cons_a))
            cons_B = np.append(cons_B, b)
            cons_C = np.row_stack((cons_C, np.zeros(cons_C.shape[1])))

            if(t == len(P)-1):
                cons_c = np.zeros((len(cons_B), 1))
                cons_c[-1] = 1.0
                cons_C = np.column_stack((cons_C, cons_c))

                RHO = np.append(RHO, rho)
    # print('agent.index', end='  :    ')
    # print(agent.index)

    # print('ignore_j_list')
    # print(len(ignore_j_list),end=' ')
    # print(ignore_j_list)

    # print('all_r_constrain')
    # print(len(all_r_constrain),end=' ')
    # print(all_r_constrain)

    cnt_inCollision_list.append(cnt_inCollision)
    if agent.index == SET.Num - 1:
        print('--------------------------  %d 个智能体 --------------------------' %
              SET.Num)
            #   agent.index+1)
        # print('cnt_inCollision_list                 ', cnt_inCollision_list)
        # print('cnt_inCollisionRadius_list      ', cnt_inCollision_list)
        print('每个智能体加入的邻居智能体作为约束的个数          ', cnt_inCollision_list)
        print('每个智能体加入的邻居智能体作为约束的个数的平均值   ',
              sum([i/SET.Num for i in cnt_inCollision_list]))# zyt 验收取消输出
        
        print('--------------------------  %d 个智能体 --------------------------' %
              SET.Num)
            #   agent.index+1)
        cnt_inCollision_list.clear()

    return [cons_A, cons_B, cons_C, RHO]


def MBVC(agent_i, agent_j, r_min):

    p = (agent_i+agent_j)/2
    a = (agent_i-agent_j)/np.linalg.norm(agent_i-agent_j)

    b = a @ p + r_min/2

    return a, b


def MBVC_WB(agent_i, agent_j, target, r_min, eta, rho_0, term_overlap):

    p = (agent_i+agent_j)/2
    a = (agent_i-agent_j)/np.linalg.norm(agent_i-agent_j)

    b = a @ p + r_min/2
    n_j = np.zeros(2)
    n_target = np.zeros(2)

    for i in range(2):
        n_j[i] = (agent_j-agent_i)[i]
        n_target[i] = (target-agent_i)[i]

    if np.linalg.norm(n_j) > 1e-10 and np.linalg.norm(n_target) > 1e-10:

        n_j = n_j/np.linalg.norm(n_j)

        n_target = n_target/np.linalg.norm(n_target)

        if(term_overlap):
            rho_i_j = rho_0 * \
                2.71828**(eta*(n_j[1]*n_target[0]-n_j[0]*n_target[1]))
        else:
            rho_i_j = rho_0

    else:
        rho_i_j = rho_0

    return a, b, rho_i_j
