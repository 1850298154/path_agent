import zstatistics as zs
import zyaml as zy
import zException as ze
import SET
from uav import *
import numpy as np
import multiprocessing as mp
# from thread import *  # Commented: thread module deleted in P0 cleanup
# from group_corridor import *
# from obstacle_corridor import *  # Commented: module deleted in P0 cleanup
# from inter_avoid import *  # Commented: module deleted in P0 cleanup
# from connection import *  # Commented: module deleted in P0 cleanup
import cvxpy as cp
from trajectory import *
from plot import *
from geometry import *
from others import *
import time
from plot import *
import copy
from scipy import linalg as lg
from cvxopt import matrix, solvers
from cvxopt.solvers import options


fallback_agent_index_list = []  # 递归判断，是否之前的状态需要回退
glo_agent_list = []
stop_multi = 2

def check_one_step(agent_list, obstacle_list, share_data):
    reverse = fallback_agent_index_list[::-1]  # 原来从小号到大号，现在从大到小
    # for max_collide_agent_index in reverse:
    if len(reverse) == 0:
        return
    # for max_collide_agent_index in range(reverse[0], -1, -1):
    # 后面没有规划的agent ， 比前面规划的的agent  少一个 pre_traj
    for max_collide_agent_index in range(zy.parameters['Num']-1, -1, -1):
        agent1_traj_list = agent_list[max_collide_agent_index].pre_traj_list
        # for j, before_agent_index in enumerate(fallback_agent_index_list):
        # for before_agent_index in range(max_collide_agent_index, -1, -1):
        for before_agent_index in range(max_collide_agent_index-1, -1, -1):
            if before_agent_index in fallback_agent_index_list:
                continue  # 这里当前step ， 是一次规划。 如果前面已经出错了， 那么同前一步一定会定住不动，无需倒退。
            agent2_traj_list = agent_list[before_agent_index].pre_traj_list

            # i_min_index=-1
            # j_min_index=-1
            # min_dis = float('inf')
            need_stop_dist=stop_multi*agent_list[max_collide_agent_index].r_min
            exist_traj_collide = False
            for i in range(len(agent1_traj_list[-1])):
                for j in range(len(agent2_traj_list[-1])):
                    agent1_step_p = agent1_traj_list[-1][i]
                    agent2_step_p = agent2_traj_list[-1][j]
                    distance = np.linalg.norm(agent1_step_p
                                              - agent2_step_p)
                    if distance < need_stop_dist:
                        exist_traj_collide = True
                        break
                if distance <need_stop_dist:
                # if distance < agent_list[max_collide_agent_index].r_min:
                    exist_traj_collide = True
                    break
            exist_traj_collide = False
            for i in range(len(agent1_traj_list[-1])):
                # 最后一轮step的预测，再取出预测中的第一个位置
                agent1_step_p = agent1_traj_list[-1][i]
                agent2_step_p = agent2_traj_list[-1][i]
                distance = np.linalg.norm(agent1_step_p
                                          - agent2_step_p)
                # if distance < agent_list[max_collide_agent_index].r_min:
                if distance < agent_list[max_collide_agent_index].r_min:
                    exist_traj_collide = True
                    break

            # agent1_step_p = agent1_traj_list[-1][0]  # 最后一轮step的预测，再取出预测中的第一个位置
            # agent2_step_p = agent2_traj_list[-1][0]
            # distance = np.linalg.norm(agent1_step_p
            #                           - agent2_step_p)
            # if distance < agent_list[max_collide_agent_index].r_min:
            if exist_traj_collide == True:
                print('i j step dis')
                four_tup = (max_collide_agent_index,
                            before_agent_index,
                            len(agent2_traj_list),
                            distance)
                print(four_tup)
                zs.number_list.append(four_tup[:3])
                zs.info2_list.append(four_tup)
                # # 记录
                # plot_pre_traj([agent_list[max_collide_agent_index],
                #                agent_list[before_agent_index]],
                #               obstacle_list, True, ze.e4_LFB_episode)
                # ze.e4_LFB_episode += 1

                # 做一些弥补措施在这里
                if len(agent2_traj_list) > 2:
                    # # agent1 已经定下来过了， 不然不会被加入 fallback_agent_index_list 后到这里
                    # agent1_traj_list[-1] = agent1_traj_list[-2]
                    # agent2_traj_list[-1] = agent2_traj_list[-2]
                    
                    # agent_list[before_agent_index].
                    
                    # agent = agent_list[before_agent_index]
                    # agent.cache = run_convex_program_fall_back(agent)
                    # agent = agent_list[max_collide_agent_index]
                    # agent.cache = run_convex_program_fall_back(agent)
                    
                    # agent.post_processing(share_data)
                    pass
                # if not REALFLY:
                #     # post processing
                #     agent.post_processing(share_data)
                # else:
                #     interval = next_interval

                #     agent.post_processing(share_data, interval)

                # # 记录
                # plot_pre_traj([agent_list[max_collide_agent_index],
                #                agent_list[before_agent_index]],
                #               obstacle_list, True, ze.e4_LFB_episode)
                # ze.e4_LFB_episode += 1


# this is the main loop
def run_one_step(agent_list, obstacle_list):
    global glo_agent_list
    glo_agent_list = agent_list
    start = time.time()

    ####### input trajectory to crazyfiles ######

    if SET.REALFLY:
        input_traj_thread = MyThread(begin_trajectory, args=[
                                     [agent_list, SET.h*SET.interval]])
        input_traj_thread.start()

    ######## communicate data ########

    share_data = get_share_data(agent_list)

    ######## the main caculation #######

    # the calculation for each agent

    items = []
    for i in range(SET.Num):
        # items.append([copy.deepcopy(agent_list[i]), copy.deepcopy(share_data), copy.deepcopy(obstacle_list),
        #               SET.REALFLY, SET.next_interval])

        # 为了后面的 elif SET.compute_model == 'norm':
        items.append([(agent_list[i]), (share_data), (obstacle_list),
                      SET.REALFLY, SET.next_interval])
        # items.append([(agent_list[i]), (share_data),  copy.deepcopy(obstacle_list),
        #               SET.REALFLY, SET.next_interval])
    if SET.compute_model == 'thread':

        pool = []
        for i in range(SET.Num):
            thread = MyThread(run_one_agent, args=[items[i]])
            pool.append(thread)
            thread.start()

        agent_list = []
        for thread in pool:
            thread.join()
            agent_list.append(thread.get_result())

    elif SET.compute_model == 'norm':

        # agent_list = [run_one_agent(item) for item in items]
        # agent_list = [print(
        #     '----------------------------',
        #     cnt, '\nlen(item[2])', len(item[2])
        # ),
        #     run_one_agent(item)
        #     for cnt, item in enumerate(items)
        # ]
        agent_list = []
        for cnt, item in enumerate(items):
            # print('---------------------------- ',
            #       cnt, '\nlen(item[2])', len(item[2])
            #       )
            # print('---------------------------- ',
            #       cnt, 'len(item[2])  obnum  = ', len(item[2])
            #       ) # 障碍物list长度 # zyt 验收取消输出

            agent_list.append(run_one_agent(item))
        # check_one_step(agent_list, obstacle_list)
        # global glo_agent_list # 加在前面了
        # check_one_step(glo_agent_list, obstacle_list, share_data=share_data)

    elif SET.compute_model == 'process':

        pool = mp.Pool(SET.core_num)
        agent_list = pool.map(run_one_agent, items)
        pool.close()
        pool.join()

    else:

        raise ValueError('Please choose a compute model')

    if SET.REALFLY:

        time_interval = time.time()-start

        print('computation time is %s' % (time_interval))

        input_traj_thread.join()

    return agent_list


def run_one_agent(item):
    import datetime
    start = datetime.datetime.now()

    agent = item[0]
    share_data = item[1]
    obstacle_list = item[2]
    REALFLY = item[3]
    next_interval = item[4]

    # get inter robot avoidance constraints
    agent.inter_cons_A, agent.inter_cons_B, agent.inter_cons_C, agent.Rho_ij = Get_inter_cons(
        agent, share_data)

    # # get obstacle collision avoidance constraints
    # agent.ob_cons_A, agent.ob_cons_B, agent.ob_corridor_list, agent.segment_list = Get_ob_cons(
    #     agent, obstacle_list)  # 多段线段（自定义异常）、SVM （cvxopt异常）
    Get_ob_cons_err = False
    try:
        # get obstacle collision avoidance constraints
        agent.ob_cons_A, agent.ob_cons_B, agent.ob_corridor_list, agent.segment_list = Get_ob_cons(
            agent, obstacle_list)  # 多段线段（自定义异常）、SVM （cvxopt异常）
    # except Exception as e:
    except ze.Seg_Except as e:
        print('**********************************************')
        print('**             agent.deadlock               **')
        print(f'    agent   {agent.index}   meet deadlock 1  ')
        print("An exception has occurred  :  ", str(e))
        import traceback
        # 获取详细的异常信息，包括位置
        traceback_details = traceback.format_exc()
        print(traceback_details)
        print('**********************************************')
        agent.deadlock = True
        agent.deadlock_info.append(len(agent.pre_traj_list))
        agent.cache = run_convex_program_fall_back(agent)
        Get_ob_cons_err = True
        fallback_agent_index_list.append(agent.index)
        # plot_pre_traj([agent], obstacle_list, True, ze.e1_Seg_episode)
        # ze.e1_Seg_episode += 1
    except Exception as e:
        print('**********************************************')
        print('**             agent.deadlock               **')
        print(f'    agent   {agent.index}   meet deadlock 2  ')
        print("An exception has occurred  :  ", str(e))
        import traceback
        # 获取详细的异常信息，包括位置
        traceback_details = traceback.format_exc()
        print(traceback_details)
        print('**********************************************')
        agent.deadlock = True
        agent.deadlock_info.append(len(agent.pre_traj_list))
        agent.cache = run_convex_program_fall_back(agent)
        Get_ob_cons_err = True
        fallback_agent_index_list.append(agent.index)
        # plot_pre_traj([agent], obstacle_list, True, ze.e2_SVM_episode)
        # ze.e2_SVM_episode += 1
    if not Get_ob_cons_err:
        # print('agent.ob_corridor_list')
        # print(type(agent.ob_corridor_list))
        # print(agent.ob_corridor_list, flush=True)
        # print(*agent.ob_corridor_list, flush=True)

        # # running convex program
        # agent.cache = run_convex_program(agent)
        try:
            # running convex program
            agent.cache = run_convex_program(agent)
        except Exception as e:
            print('**********************************************')
            print('**             agent.deadlock               **')
            print(f'    agent   {agent.index}   meet deadlock 3  ')
            print("An exception has occurred  :  ", str(e))
            import traceback
            # 获取详细的异常信息，包括位置
            traceback_details = traceback.format_exc()
            print(traceback_details)
            print('**********************************************')
            agent.deadlock = True
            agent.deadlock_info.append(len(agent.pre_traj_list))
            agent.cache = run_convex_program_fall_back(agent)
            fallback_agent_index_list.append(agent.index)
            # plot_pre_traj([agent], obstacle_list, True, ze.e3_MPC_episode)
            # ze.e3_MPC_episode += 1

    if check_hitting_obstacles(agent):
        agent.cache = run_convex_program_fall_back(agent)
    # global glo_agent_list
    # check_one_step(glo_agent_list, obstacle_list, agent.index)

    if not REALFLY:
        # post processing
        agent.post_processing(share_data)
    else:
        interval = next_interval

        agent.post_processing(share_data, interval)

    end = datetime.datetime.now()
    # from datetime import timedelta

    # 创建一个示例的timedelta对象
    td = end-start

    # 将timedelta对象转换为总秒数
    total_seconds = td.total_seconds()

    # 将总秒数转换为浮点数
    total_seconds_float = float(total_seconds)
    agent.plan_time_list.append(total_seconds_float)
    return agent


def check_hitting_obstacles(agent):
    # get predeterminted trajectory and the terminal point
    # P = agent.Phi @ agent.cache[0]
    # P = P.reshape((agent.K, agent.D))
    # agent.terminal_p = P[-1].copy()
    agent1_step_p = agent.cache[0][0:agent.D]
    import zrand as zr
    is_ex_collision,ob_i,exp_smin = zr.tri_check_agents_external_collision(
        agent1_step_p[0],
        agent1_step_p[1],
        # obstacles, # 膨胀后的障碍物 
        zr.obstacles, # 膨胀后的障碍物 
        # extend_obstacles, # 膨胀后的障碍物 
        # zr.radius,
        zy.parameters['radius'],
    )
    if is_ex_collision:
        print('check_hitting_obstacles')
        print('is_ex_collision,ob_i,exp_smin')
        print(is_ex_collision,ob_i,exp_smin)
        return True
    else:
        return False

def run_convex_program_fall_back(agent):
    # region
    type = agent.type

    ####### functional constraints related #######
    inter_cons_A = agent.inter_cons_A
    inter_cons_B = agent.inter_cons_B
    inter_cons_C = agent.inter_cons_C
    ob_cons_A = agent.ob_cons_A
    ob_cons_B = agent.ob_cons_B
    Rho_ij = agent.Rho_ij
    epsilon = agent.epsilon
    buffer = agent.buffer

    ####### dynamic related #######

    state = agent.state

    K = agent.K
    D = agent.D

    VA = agent.VA
    VB = agent.VB
    VC = agent.VC

    # Theta_u = agent.Theta_u
    # Theta_v = agent.Theta_v
    # Theta_p = agent.Theta_p

    # Umax = agent.Umax
    # Vmax = agent.Vmax

    # Xi = agent.Xi
    # Phi = agent.Phi
    # Xi_K = agent.Xi_K

    # G_p = agent.G_p
    # W = agent.W.copy()

    # ######## objective function related #######

    # if np.linalg.norm(agent.terminal_p-agent.target) > 2.0:
    #     Q_tar = 10.0*2.0/np.linalg.norm(agent.terminal_p-agent.target)
    # else:
    #     Q_tar = 10.0

    # if type == "Free-transitor" or type == "Obstacle-transitor" or type == "Searcher":

    #     cost_index = agent.cost_index

    #     # get the needed weight coefficient matrix
    #     Sigma = np.zeros([D * K, D * K])

    #     for i in range(max([cost_index-1, 0]), K):
    #         for j in range(D):
    #             Sigma[D*i+j][D*i+j] = Q_tar

    #     Delta_P = 1.0*agent.Delta_P

    #     Q = VB.T @  Phi.T @ Sigma @  Phi @  VB + VB.T @  Phi.T @ Delta_P @  Phi @  VB
    #     p = 2 * VB.T @  Phi.T @ Sigma @ (Phi @  (VA @ state + VC) - G_p) +\
    #         2 * VB.T @  Phi.T @ Delta_P @ Phi @  (VA @ state + VC)

    # elif type == "Connector" or type == "Follower":

    #     P_neighbor = agent.P_neighbor.copy()

    #     Q = 5*VB.T @  Phi.T @  Phi @  VB
    #     p = 10*VB.T @  Phi.T @ (Phi @  (VA @ state + VC) - P_neighbor)

    # else:

    #     raise Exception("Please choose the type of robot")

    # ##############################
    # ####### convex program #######
    # ##############################

    # len_U = D*K
    # len_E = inter_cons_C.shape[1]
    # len_ob_B = len(ob_cons_B)

    # l = len_U+len_E+len_ob_B

    # M_log = np.diag(Rho_ij)

    # P = lg.block_diag(Q, M_log, 200000*np.eye(len_ob_B))
    # q = np.block([p, -2*M_log @ np.ones(len_E)*epsilon, np.zeros(len_ob_B)])

    # # inter avoidance constraints
    # len_cons_B = len(inter_cons_B)

    # G_1 = np.zeros((len_cons_B, l))
    # G_1[0:len_cons_B, 0:len_U] = -inter_cons_A @  Phi @ VB
    # G_1[0:len_cons_B, len_U:len_U+len_E] = inter_cons_C

    # h_1 = inter_cons_A @  Phi @ (VA @ state + VC) - inter_cons_B

    # # E lower bound constraint
    # G_2 = np.zeros((len_E, l))
    # G_2[0:len_E, len_U:len_U+len_E] = -np.eye(len_E)

    # h_2 = np.zeros(len_E)

    # # E upper bound constraint
    # G_3 = np.zeros((len_E, l))
    # G_3[0:len_E, len_U:len_U+len_E] = np.eye(len_E)

    # h_3 = np.ones(len_E)*epsilon

##############################################################################
    # # obstacle aviodance constriants
    # len_ob_cons_B = len(ob_cons_B)
    # # print('len_ob_cons_B')
    # # print(len_ob_cons_B)

    # G_4 = np.zeros((len_ob_B, l))
    # G_4[0:len_ob_cons_B, 0:len_U] = - ob_cons_A @  Phi @ VB
    # G_4[0:len_ob_cons_B, len_U+len_E:len_U+len_E+len_ob_B] = -np.eye(len_ob_B)

    # h_4 = ob_cons_A @  Phi @ (VA @ state + VC) - ob_cons_B - buffer

    # # ob_B up nound constraints

    # G_5 = np.zeros((len_ob_cons_B, l))
    # G_5[0:len_ob_cons_B, len_U+len_E:len_U+len_E+len_ob_B] = np.eye(len_ob_B)

    # h_5 = np.ones(len_ob_B)*buffer

    # l_nonnegative_orthant = len(h_1) + len(h_2) + \
    #     len(h_3) + len(h_4) + len(h_5)
##############################################################################

    # # terminal constraint
    # A = np.zeros((D, l))
    # A[0:D, 0:len_U] = Xi_K @ VB

    # b = np.zeros(D) - Xi_K @ (VA @ state + VC)

    # # acceleration constraints
    # G_cone_1 = np.zeros(((D+1)*K, l))
    # h_cone_1 = np.zeros((D+1)*K)
    # for k in range(K):
    #     e_k = np.zeros((D, D*K))
    #     e_k[0:D, D*k:D*k+D] = np.eye(D)
    #     G_cone_1[(D+1)*k+1:(D+1)*(k+1), 0:len_U] = e_k
    #     h_cone_1[(D+1)*k] = Umax

    # # velocity constraints
    # G_cone_2 = np.zeros(((D+1)*K, l))
    # h_cone_2 = np.zeros((D+1)*K)
    # for k in range(K):
    #     e_k = np.zeros((D, D*K))
    #     e_k[0:D, D*k:D*k+D] = np.eye(D)
    #     G_cone_2[(D+1)*k+1:(D+1)*(k+1), 0:len_U] = e_k @ Xi @ VB
    #     h_cone_2[(D+1)*k] = Vmax
    #     h_cone_2[(D+1)*k+1:(D+1)*(k+1)] = - e_k @ Xi @ (VA @ state + VC)

    # # G = matrix(
    # #     np.block([[G_1], [G_2], [G_3], [G_4], [G_5], [G_cone_1], [G_cone_2]]))
    # # print('G_4.shape')
    # # print(G_4.shape)
    # G = matrix(
    #     np.block([[G_1], [G_2], [G_3],
    #               [G_4], [G_5],
    #               [G_cone_1], [G_cone_2]
    #               ]))
    # h = matrix(np.block([h_1, h_2, h_3, h_4, h_5, h_cone_1, h_cone_2]))
    # A = matrix(A)
    # b = matrix(b)
    # P = 2*matrix(P)
    # q = matrix(q)

    # dims = {'l': l_nonnegative_orthant, 'q': [
    #     D+1 for i in range(K+K)], 's': []}

    # options.update({'show_progress': False})
    # endregion

    # res = solvers.coneqp(P=P, q=q, G=G, h=h, A=A, b=b, dims=dims)

    # U = (np.array(res['x'][0:len_U]))[:, 0]
    # E = (np.array(res['x'][len_U:l]))[:, 0]
    nstate = np.array(state)
    nstate[2:4] = 0.0
    return [
        # VA @ state + VB @ U + VC,
        VA @ nstate,
        # U.reshape((K, D)),
        np.zeros((K, D)),
        # E[1:inter_cons_C.shape[1]]
        epsilon * np.ones(inter_cons_C.shape[1]-1)
    ]


# ======================================================================
# run convex program of each agents
def run_convex_program(agent):
    # region
    type = agent.type

    ####### functional constraints related #######
    inter_cons_A = agent.inter_cons_A
    inter_cons_B = agent.inter_cons_B
    inter_cons_C = agent.inter_cons_C
    ob_cons_A = agent.ob_cons_A
    ob_cons_B = agent.ob_cons_B
    Rho_ij = agent.Rho_ij
    epsilon = agent.epsilon
    buffer = agent.buffer

    ####### dynamic related #######

    state = agent.state

    K = agent.K
    D = agent.D

    VA = agent.VA
    VB = agent.VB
    VC = agent.VC

    Theta_u = agent.Theta_u
    Theta_v = agent.Theta_v
    Theta_p = agent.Theta_p

    Umax = agent.Umax
    Vmax = agent.Vmax

    Xi = agent.Xi
    Phi = agent.Phi
    Xi_K = agent.Xi_K

    G_p = agent.G_p
    W = agent.W.copy()

    ######## objective function related #######

    if np.linalg.norm(agent.terminal_p-agent.target) > 2.0:
        Q_tar = 10.0*2.0/np.linalg.norm(agent.terminal_p-agent.target)
    else:
        Q_tar = 10.0

    if type == "Free-transitor" or type == "Obstacle-transitor" or type == "Searcher":

        cost_index = agent.cost_index

        # get the needed weight coefficient matrix
        Sigma = np.zeros([D * K, D * K])

        for i in range(max([cost_index-1, 0]), K):
            for j in range(D):
                Sigma[D*i+j][D*i+j] = Q_tar

        Delta_P = 1.0*agent.Delta_P

        Q = VB.T @  Phi.T @ Sigma @  Phi @  VB + VB.T @  Phi.T @ Delta_P @  Phi @  VB
        p = 2 * VB.T @  Phi.T @ Sigma @ (Phi @  (VA @ state + VC) - G_p) +\
            2 * VB.T @  Phi.T @ Delta_P @ Phi @  (VA @ state + VC)

    elif type == "Connector" or type == "Follower":

        P_neighbor = agent.P_neighbor.copy()

        Q = 5*VB.T @  Phi.T @  Phi @  VB
        p = 10*VB.T @  Phi.T @ (Phi @  (VA @ state + VC) - P_neighbor)

    else:

        raise Exception("Please choose the type of robot")

    ##############################
    ####### convex program #######
    ##############################

    len_U = D*K
    len_E = inter_cons_C.shape[1]
    len_ob_B = len(ob_cons_B)

    l = len_U+len_E+len_ob_B

    M_log = np.diag(Rho_ij)

    P = lg.block_diag(Q, M_log, 200000*np.eye(len_ob_B))
    q = np.block([p, -2*M_log @ np.ones(len_E)*epsilon, np.zeros(len_ob_B)])

    # inter avoidance constraints
    len_cons_B = len(inter_cons_B)

    G_1 = np.zeros((len_cons_B, l))
    G_1[0:len_cons_B, 0:len_U] = -inter_cons_A @  Phi @ VB
    G_1[0:len_cons_B, len_U:len_U+len_E] = inter_cons_C

    h_1 = inter_cons_A @  Phi @ (VA @ state + VC) - inter_cons_B

    # E lower bound constraint
    G_2 = np.zeros((len_E, l))
    G_2[0:len_E, len_U:len_U+len_E] = -np.eye(len_E)

    h_2 = np.zeros(len_E)

    # E upper bound constraint
    G_3 = np.zeros((len_E, l))
    G_3[0:len_E, len_U:len_U+len_E] = np.eye(len_E)

    h_3 = np.ones(len_E)*epsilon

##############################################################################
    # obstacle aviodance constriants
    len_ob_cons_B = len(ob_cons_B)
    # print('len_ob_cons_B')
    # print(len_ob_cons_B)

    G_4 = np.zeros((len_ob_B, l))
    G_4[0:len_ob_cons_B, 0:len_U] = - ob_cons_A @  Phi @ VB
    G_4[0:len_ob_cons_B, len_U+len_E:len_U+len_E+len_ob_B] = -np.eye(len_ob_B)

    h_4 = ob_cons_A @  Phi @ (VA @ state + VC) - ob_cons_B - buffer

    # ob_B up nound constraints

    G_5 = np.zeros((len_ob_cons_B, l))
    G_5[0:len_ob_cons_B, len_U+len_E:len_U+len_E+len_ob_B] = np.eye(len_ob_B)

    h_5 = np.ones(len_ob_B)*buffer

    l_nonnegative_orthant = len(h_1) + len(h_2) + \
        len(h_3) + len(h_4) + len(h_5)
##############################################################################

    # terminal constraint
    A = np.zeros((D, l))
    A[0:D, 0:len_U] = Xi_K @ VB

    b = np.zeros(D) - Xi_K @ (VA @ state + VC)

    # acceleration constraints
    G_cone_1 = np.zeros(((D+1)*K, l))
    h_cone_1 = np.zeros((D+1)*K)
    for k in range(K):
        e_k = np.zeros((D, D*K))
        e_k[0:D, D*k:D*k+D] = np.eye(D)
        G_cone_1[(D+1)*k+1:(D+1)*(k+1), 0:len_U] = e_k
        h_cone_1[(D+1)*k] = Umax

    # velocity constraints
    G_cone_2 = np.zeros(((D+1)*K, l))
    h_cone_2 = np.zeros((D+1)*K)
    for k in range(K):
        e_k = np.zeros((D, D*K))
        e_k[0:D, D*k:D*k+D] = np.eye(D)
        G_cone_2[(D+1)*k+1:(D+1)*(k+1), 0:len_U] = e_k @ Xi @ VB
        h_cone_2[(D+1)*k] = Vmax
        h_cone_2[(D+1)*k+1:(D+1)*(k+1)] = - e_k @ Xi @ (VA @ state + VC)

    # G = matrix(
    #     np.block([[G_1], [G_2], [G_3], [G_4], [G_5], [G_cone_1], [G_cone_2]]))
    # print('G_4.shape')
    # print(G_4.shape)
    G = matrix(
        np.block([[G_1], [G_2], [G_3],
                  [G_4], [G_5],
                  [G_cone_1], [G_cone_2]
                  ]))
    h = matrix(np.block([h_1, h_2, h_3, h_4, h_5, h_cone_1, h_cone_2]))
    A = matrix(A)
    b = matrix(b)
    P = 2*matrix(P)
    q = matrix(q)

    dims = {'l': l_nonnegative_orthant, 'q': [
        D+1 for i in range(K+K)], 's': []}

    options.update({'show_progress': False})
    # endregion

    res = solvers.coneqp(P=P, q=q, G=G, h=h, A=A, b=b, dims=dims)
    # print('res ---------------------')
    # print(res)
    # print('status : ',res['status'])
    # if res['status'] != 'optimal':
    #     print('False : ', res['status'])
    #     # input('input ---------------------')
    #     print('input ---------------------')
    # print('res ---------------------')
    

    U = (np.array(res['x'][0:len_U]))[:, 0]
    E = (np.array(res['x'][len_U:l]))[:, 0]

    return [VA @ state + VB @ U + VC, U.reshape((K, D)), E[1:inter_cons_C.shape[1]]]
