import SET
import bug
# import zbug as zb
import zrand as zr
import random
# from turtle import distance
import numpy as np
from scipy import linalg as lg
from geometry import *
# from path_planning import *
# import path_planning as planner
import copy
import time
import zyaml as zy


class uav2D():

    # initialization
    def __init__(self, index, ini_x, target, type, ini_K=11):
        #    zyt
        # If it stops, but does not reach the end, it deadlocks
        self.deadlock = False
        self.deadlock_info = []
        self.plan_time_list = []
        self.old_path_list = []
        self.pullback=0
        self.pullback_list=[]
        self.tractive_point_list=[]
        self.tractive_point_list_list=[]
        self.step_tractive = []

        ##############################
        ####### key cofficient #######
        ##############################

        # the index of this agent
        self.index = index

        # the type of this UAV:
        self.type = type

        # the length of horizon
        self.K = ini_K

        # the dimension of uav
        self.D = 2

        self.h = SET.h

        self.buffer = SET.buffer

        self.REALFLY = SET.REALFLY

        self.obstacle_list = copy.deepcopy(SET.obstacle_list)

        #####################
        ####### state #######
        #####################

        # initial position
        self.ini_p = ini_x.copy()

        ini_v = np.zeros(2)

        # current position
        self.p = ini_x.copy()

        # current velocity
        self.v = ini_v.copy()

        # input
        self.u = np.zeros(self.D)

        # current state including position and velocity
        self.state = np.append(self.p, self.v)

        # maximum acc
        # self.Umax = 2.0
        self.Umax = zy.parameters['Umax']

        # maximum velocity
        # self.Vmax = 3.0
        self.Vmax = zy.parameters['Vmax']

        # self.r_a=0.3
        # self.r_a=0
        self.r_a = SET.ExtendWidth # agent的半径

        # the redius of a robot (pay attention: it is diameter!)  
        self.r_min = 2*self.r_a  # np.sqrt(4*self.r_a**2+(self.h*self.Vmax)**2) # agent的直径
        self.physical_radius = zy.parameters['physical_radius']
        self.physical_r_min = 2*self.physical_radius

        # self.r_max = self.Vmax*self.h*self.K
        self.r_max = self.Vmax*self.h*self.K+self.r_min*4

        # dynamic matrix
        self.get_dynamic()

        self.get_coef_matrix()

        ##########################
        ####### Trajectory #######
        ##########################

        # target position
        self.target = target

        # terminal position
        self.terminal_p = self.p.copy()

        if type == 'Obstacle-transitor' or type == 'Searcher':
            print(' bug.path_plan(', flush=True)
            print('bug_planner_time : ', end=' ')

            # get path
            # self.path = planner.path_plan(
            #     self.terminal_p, self.target)  # zyt:path
            # self.path = bug.path_plan(
            #     self.terminal_p, self.target, bug.obstacle_adapter(zr.obstacles))  # 封装
            lower_bound = zy.parameters['bug.inflated_size']  # 大于两个agent半径
            upper_bound = (zy.parameters['zr.more_inflated_size'] -
                           zy.parameters['radius'])
            # lower_bound = upper_bound

            # 最后agent别跑出去了
            self.path = bug.path_plan(
                self.index,
                self.terminal_p, self.target,
                bug.obstacle_adapter(zr.obstacles),

                inflated_size=random.uniform(
                    lower_bound, upper_bound
                )
            )  # 封装
            # self.path = zb.path_plan(
            #     self.terminal_p, self.target, zb.obstacle_adapter(zr.obstacles))  # 封装
            self.path = np.array(self.path)
            
            # self.path=np.array([self.terminal_p,self.target])
            # self.path=np.array([np.array([250,125]),self.target])
            # self.path = np.linspace(self.terminal_p, self.target, num=30, endpoint=True, retstep=False, dtype=None)
            # self.path = np.linspace(np.array([250,125]), self.target, num=30, endpoint=True, retstep=False, dtype=None)
            
            print('uav.__init__()')
            print('path', self.index)
            print(*self.path, sep='\n')
            # print(type(self.path)) # numpy.ndarray #TypeError: 'str' object is not callable
            # np.savetxt("data/path"+str(self.index),self.path)
            # self.path=np.loadtxt("data/path"+str(self.index))

        # a coefficient related to the objective
        self.cost_index = ini_K

        # the predetermined trajectory
        self.pre_traj = np.zeros((self.K+1, self.D))

        for i in range(self.K+1):
            self.pre_traj[i] = self.p.copy()

        # tractive position
        self.tractive_point = None

        # the tractive position list for objective
        self.G_p = None

        # get tractive point for obstace transitor
        self.get_tractive_point()

        # the list of all time's
        self.pre_traj_list = []

        # the list of all past position
        self.position = self.p.copy()

        #######################
        ####### Dealock #######
        #######################

        self.rho_0 = 10.0

        # warning band width
        self.epsilon = 0.20

        self.term_overlap = False

        self.term_last_p = self.p.copy()

        self.E = 0.0

        self.term_overlap_again = False

        self.term_last_p = self.p.copy()

        self.term_index = 0

        self.eta = 1.0

        self.E = 0.4*self.epsilon*np.ones(SET.Num-1)

        self.W = 0.4*self.epsilon*np.ones(SET.Num)

        self.ever_reach_target = False

        self.priority = 2

        self.contest = False

        self.distance = np.linalg.norm(self.p-self.target)

        self.no_supremacy = True

        self.min_distance = False

        #########################
        ####### crazyfile #######
        #########################

        if self.REALFLY:

            # the height of crazyfile is constantly 1m
            self.height = SET.height

            # the yaw of crazyfile is constantly 0.0 deg
            self.yaw = 0.0

            # the input trajectory
            self.input_traj = self.get_input_traj()

        ###############################
        ####### data collection #######
        ###############################

        self.data = np.block([[np.array([self.index, self.D])],
                              [self.ini_p], [self.target], [-9999999*np.ones(self.D)]])

    def post_processing(self, share_data, interval=1):

        # data collection
        U_list = self.cache[1]

        # get new input
        self.u = U_list[0]

        # get predeterminted trajectory and the terminal point
        P = self.Phi @ self.cache[0]
        P = P.reshape((self.K, self.D))
        self.terminal_p = P[-1].copy()

        self.data = np.block([[self.data], [self.tractive_point], [self.p], [self.v],
                              [-7777777*np.ones(self.D)], [P], [-9999999*np.ones(self.D)]])
        self.step_tractive.append(self.tractive_point)

        # here, in our code, PT including the current position in next time's replanning
        self.pre_traj = np.block([[P], [self.terminal_p]])

        if not self.REALFLY:
            # 在仿真状态中认为生成轨迹中的第一个位置就是下一时刻的位置
            # get new state
            self.p[0:self.D] = self.cache[0][0:self.D].copy()
            self.v[0:self.D] = self.cache[0][self.D:2*self.D].copy()
            self.state = np.append(self.p, self.v)

            # get position list
            self.position = np.block([[self.position], [self.p]])
        else:
            ####### the realfly #######

            # get the input trajectory
            self.input_traj = self.get_input_traj()

            # 在真实运行过程中，会认为本次生成轨迹之后再过 (interval-1)*h 时间后才是下次计算时初始状态的时刻
            self.p[0:self.D] = self.cache[0][2*self.D *
                                             (interval-1)+0:2*self.D*(interval-1)+self.D].copy()
            self.v[0:self.D] = self.cache[0][2*self.D *
                                             (interval-1)+self.D:2*self.D*(interval-1)+2*self.D].copy()
            self.state = np.append(self.p, self.v)

            # get new state and predetermined tarjectory for crazyfile
            self.get_pre_traj(interval)

            # get position list
            for i in range(1, interval+1):
                p = self.cache[0][2*self.D *
                                  (i-1)+0:2*self.D*(i-1)+self.D].copy()
                self.position = np.block([[self.position], [p]])

        # get predtermined trajectory list of each time step
        self.pre_traj_list += [self.pre_traj]

        # get new cost_index
        for i in range(self.K, -1, -1):
            if(np.linalg.norm(self.pre_traj[i]-self.target) > 0.01):
                break
        self.cost_index = i  # 要求全部到达终点，要求K个严格<=0.01

        if i < self.K:
            self.ever_reach_target = True

        # print('self.W')
        # print(type(self.W))
        # print(self.W)
        # print(flush=True)
        # print('self.W.shape')
        # print(self.W.shape)   #(100,)
        # print(flush=True)

        # print('self.E')
        # print(type(self.E))
        # print(self.E)
        # print(flush=True)
        # print('self.E.shape')
        # print(self.E.shape)   # (99,)
        # print(flush=True)

        # print('self.epsilon')
        # print(type(self.epsilon))
        # print(self.epsilon)
        # print(flush=True)
        # # print('self.epsilon.shape')
        # # print(self.epsilon.shape)   # 浮点数
        # # print(flush=True)

        # print('np.block([self.epsilon, self.E])')
        # print(type(np.block([self.epsilon, self.E])))
        # print(np.block([self.epsilon, self.E]))
        # print(np.block([self.epsilon, self.E]).shape)  # (100,)

        # epsilon_100s = np.block([self.epsilon, self.E]) # 原来代码没了，自己回去慢慢找哈
        # zyt:长度为100,大小为epsilon浮点数
        epsilon_100s = np.repeat(self.epsilon, SET.Num)
        self.W = 0.2*epsilon_100s+0.8*self.W

        for i in range(len(self.W)):
            if self.W[i] < 0.005:
                self.W[i] = 0.005

        self.distance = np.linalg.norm(self.target-self.terminal_p)

        # deadlcok resolution
        self.deadlock_resolution(share_data)

        # get tractive position
        self.get_tractive_point()

        return None

    # transform the predetermined trajectory to the one that can be parsed by crazyfile
    def get_input_traj(self):

        P = self.pre_traj

        P = np.block([[self.state[0:2]], [P], [P[-1]]])
        self.or_pre_traj = P.copy()

        t = self.h*np.arange(len(P))
        W = np.ones(len(P))
        polyx = np.polyfit(t, P[:, 0], 7, w=W)
        polyy = np.polyfit(t, P[:, 1], 7, w=W)

        polyz = np.zeros(8)
        polyz[7] = self.height
        polyyaw = np.zeros(8)
        polyyaw[7] = self.yaw

        t = self.h*(len(P)-1)

        # 这个地方进行了进一部分进行了进一步的修改
        input_traj = [polyx, polyy, polyz, polyyaw]

        return input_traj

    # 在crazyfile情况下获得下时刻的状态估计，并且获得估计的预设轨迹，值得一提的是这里的interval是下次计算所需要的时间
    def get_pre_traj(self, interval):

        l = len(self.pre_traj)

        self.pre_traj[0:l-(interval-1)] = self.pre_traj[interval-1:l].copy()

        for i in range(interval-1):
            self.pre_traj[l-1-i] = self.terminal_p.copy()

        return None

    def deadlock_resolution(self, share_data):

# 1.	首先，将self.pre_traj中最后一项赋值给term_p变量。
#       post_processing 对刚求出来的最后一个step的 K个预测点
        term_p = self.pre_traj[-1].copy()

# 2.	将self.pre_traj倒数第二项赋值给term_second_p变量。
        term_second_p = self.pre_traj[-2].copy()
        # term_thrid_p=self.pre_traj[-3].copy()

# 3.	判断是否存在缓存数据self.cache[2]，若存在，则将其赋值给self.E变量。
        self.E = self.cache[2]

# 4.	如果self.term_overlap为True（即之前发生了重叠现象），则进行以下判断：
        if self.term_overlap:
# •	判断当前位置term_p与上次位置self.term_last_p的欧氏距离是否小于0.005，以及与倒数第二个位置term_second_p的欧氏距离是否小于0.01，以及与目标位置self.target的欧氏距离是否大于0.02。
            condition_a = np.linalg.norm(term_p-self.term_last_p) < 0.001
            condition_b = np.linalg.norm(term_p-term_second_p) < 0.005
            condition_c = np.linalg.norm(term_p-self.target) > 0.02
# •	如果满足以上条件，则将self.term_overlap设为True。
            if condition_a and condition_b and condition_c:
                self.term_overlap_again = True
# 5.	如果self.term_overlap为False（即之前未发生重叠现象），则进行以下判断：
        else:
# •	判断当前位置term_p与上次位置self.term_last_p的欧氏距离是否小于0.005，以及与倒数第二个位置term_second_p的欧氏距离是否小于0.01，以及与目标位置self.target的欧氏距离是否大于0.02。
            condition_a = np.linalg.norm(term_p-self.term_last_p) < 0.005
            condition_b = np.linalg.norm(term_p-term_second_p) < 0.01
            condition_c = np.linalg.norm(term_p-self.target) > 0.02
# •	如果满足以上条件，则将self.term_overlap设为True。
            if condition_a and condition_b and condition_c:
                self.term_overlap = True
                # print(str(self.index)+" begins terminal overlap mode")

# 6.	判断是否满足特定条件flag：
        flag = False
# •	如果self.E的类型是NumPy数组，且对每个元素计算self.epsilon - self.E的差值后的绝对值都小于1e-3，则将flag设为True。
        if(type(self.E) is np.ndarray):
            if (self.epsilon-self.E < 1e-3).all():
                flag = True
# •	如果self.E的类型不是NumPy数组，且self.epsilon - self.E的差值小于1e-3，则将flag设为True。
        elif self.epsilon-self.E < 1e-3:
            flag = True
# 7.	如果flag为True，则进行以下操作：
        if flag:
# •	将self.term_overlap和self.term_overlap_again都设为False。
            self.term_overlap = False
            self.term_overlap_again = False
# •	将self.eta设为1.0。
            self.eta = 1.0
# •	将self.contest设为False。
            self.contest = False
# •	如果self.priority为3，则将其设为2。
            if self.priority == 3:
                self.priority = 2
# 8.	将self.term_index加1。
        self.term_index += 1
# 9.	如果self.term_overlap_again为True，并且self.eta小于4.0，则进行以下操作：
        if self.term_overlap_again and self.eta < 4.0:  # and self.term_index > 3:
# •	将self.term_overlap_again设为False。
            self.term_overlap_again = False
# •	将self.eta增加0.3。
            self.eta += 0.3
# •	将self.term_index设为0。
            self.term_index = 0
# 10.	将term_p复制给self.term_last_p。
        self.term_last_p = term_p.copy()

        # print(str(self.index)+"'s terminal overlap is: "+str(self.term_overlap)+" and "+str(self.E))
        
# 11.	获取共享数据share_data中的优先级列表priority_list。
        priority_list = share_data['priority']
# 12.	判断是否存在优先级为3的任务，如果存在，则将no_supremacy设为False；否则，将其设为True。
        no_supremacy = True
        for priority in priority_list:
            if priority == 3:
                no_supremacy = False
                break
# 13.	获取共享数据share_data中的距离列表distance_list和竞争状态列表contest_list。
        distance_list = share_data['distance']
        contest_list = share_data['contest']
        min_distance = True
        distance = distance_list[self.index]
# 14.	判断当前机器人的距离是否是最小距离，以及是否存在其他机器人正在竞争。
        for dis, contest in zip(distance_list, contest_list):
            if dis > distance and contest:
                min_distance = False
                break
# 15.	如果该机器人曾经到达过目标位置，将其优先级设为1；否则，根据一定条件判断将其优先级设为3。
        if self.ever_reach_target:
            self.priority = 1
        else:
            if self.eta > 3.7:

                if self.contest and no_supremacy and min_distance:
                    self.priority = 3
                self.contest = True
# 16.	返回None。
        return None

    # get the list of tractive point which is used for tracting the agent to the tractive point

    def get_tractive_point_list(self):

        G_p = self.tractive_point
        # print('self.tractive_point')
        # print(self.tractive_point)
        for i in range(1, self.K):
            G_p = np.append(G_p, self.tractive_point)
        self.G_p = G_p

        return None

    # get the tractive point

    def get_tractive_point(self):

        if self.type == "Free-transitor":

            self.tractive_point = self.target

        elif self.type == "Obstacle-transitor" or self.type == "Searcher":

            obstacle_list = self.obstacle_list

            # if the path is None, i.e, the search based planning doesn't a feasible path, the tractive point will be chosen as the terminal point of predetermined trajectory
            if self.path is None:

                self.tractive_point = self.terminal_p.copy()  # 这个地方以后可能还需要一定的修改

            else:

                if self.type == "Obstacle-transitor":
                    # print('if self.type == "Obstacle-transitor":')
                    # 牵引点首先选择 ompl上的第一个中继站。
                    # 如果没有碰撞，可以选择更后面的中继站
                    self.tractive_point = self.terminal_p.copy()
                    # target: [51.038549   11.37081417]
                    # terminal_p: [10.60122968 46.76207209]
                    # path: [[ 9.73163309 47.61235655]
                    #  [21.59120754 34.96797529]
                    #  [42.19843009 15.86651765]
                    #  [51.038549   11.37081417]]
                    # cost_index: 8
                    # pre_traj: [[ 9.84444568 47.50204944]
                    #  [ 9.95161763 47.39725767]
                    #  [10.09357335 47.25845443]
                    #  [10.26467246 47.09115533]
                    #  [10.40098752 46.95786741]
                    #  [10.50251878 46.85859093]
                    #  [10.56926616 46.79332582]
                    #  [10.60122968 46.76207209]
                    #  [10.60122968 46.76207209]]
                    # tractive_point: [42.19843009 15.86651765]

                    # if a collision-free path exists, then we can find the tractive point
                    for i in range(len(self.path)-1, -1, -1):
                        # print('i = ',i)
                        if not detect_line_collision(obstacle_list, line(self.path[i], self.terminal_p)):

                            # 就是ompl求出来的路径
                            # 只要不碰撞，就可以选择最后一个ompl上的中继站走
                            self.tractive_point = self.path[i].copy()
                            # print('self.tractive_point = self.path[i].copy()')
                            # print('self.tractive_point = ',self.tractive_point )
                            # print('self.terminal_p     = ',self.terminal_p)
                            # dis =  np.linalg.norm(self.terminal_p-self.tractive_point)
                            # print('dis                 = ',dis)
                            
                            if i >= self.pullback: # 虽然找到了不碰撞的路径点
                                self.pullback = i # 更新，只能前，不能后退
                            else: # 如果要后退， 直接重规划
                                print('如果要后退， 直接重规划')
                                self.pullback_list.append(self.pullback)
                                self.pullback=0
                                self.tractive_point_list_list.append(self.tractive_point_list)
                                self.tractive_point_list=[]
                                print('self.index, self.pullback_list, i')
                                print(self.index, self.pullback_list, i)
                                # self.get_new_target(self.target) # 迷路了 调用； 还有一种情况，路径卡主障碍物，出现回退的情况
                            self.tractive_point_list.append((i, self.tractive_point))
                            break
                    else:
                        # print('i = ',i)
                        # print(
                        #     'break'
                        # )
                        # print('调用replan吧')
                        print('如果迷路了，直接重规划')
                        self.pullback_list.append(self.pullback)
                        self.pullback = 0
                        self.tractive_point_list_list.append(self.tractive_point_list)
                        self.tractive_point_list=[]
                        print('self.index, self.pullback_list')
                        print(self.index, self.pullback_list)
                        self.tractive_point_list.append((i, self.tractive_point))

                        # if not detect_line_collision(obstacle_list, line(self.terminal_p, self.terminal_p)): # 确定不在障碍物里面
                        #     self.get_new_target(self.target) # 迷路了 调用； 还有一种情况，路径卡主障碍物，出现回退的情况

                elif self.type == "Searcher":

                    for i in range(len(self.path)):

                        if np.linalg.norm(self.path[i] - self.tractive_point) < 1e-2:

                            p_start = i

                    for i in range(len(self.path)-1, p_start-1, -1):

                        vertex_list = [self.terminal_p]

                        vertex_list += [self.path[k]
                                        for k in range(i, p_start-1, -1)]

                        # print(vertex_list)

                        poly = polygon(vertex_list)

                        if not detect_polygon_collision(obstacle_list, poly):

                            self.tractive_point = self.path[i].copy()

                            break

        else:

            self.tractive_point = self.terminal_p.copy()

        # No matter whether path exists or doesn't, there needs a tractive list for convex programming
        self.get_tractive_point_list()

        return None

    # change the target position
    def get_new_target(self, target):

        self.target = target.copy()
        print('get_new_target')
        print(' bug.path_plan(', flush=True)

        # replanning the path
        # self.path = planner.path_plan(self.terminal_p, self.target)
        lower_bound = zy.parameters['bug.inflated_size']  # 大于两个agent半径
        upper_bound = (zy.parameters['zr.more_inflated_size'] -
                        zy.parameters['radius'])
        self.old_path_list.append(self.path)
        # 最后agent别跑出去了
        self.path = bug.path_plan(
            self.index,
            # self.terminal_p, self.target, # 视野末端 进入 障碍物了
            self.p, self.target,
            bug.obstacle_adapter(zr.obstacles),

            inflated_size=random.uniform(
                lower_bound, upper_bound
            )
        )  # 封装
        # self.path = bug.path_plan(
        #     self.terminal_p, self.target, bug.obstacle_adapter(zr.obstacles))  # 封装
        # self.path = zb.path_plan(
        #     self.terminal_p, self.target, zb.obstacle_adapter(zr.obstacles))  # 封装
        self.path = np.array(self.path)

        print('uav.get_new_target()')
        print('path', self.index)
        print(*self.path, sep='\n', end='----------------\n\n')
        # get the new tractive point
        self.get_tractive_point()
        

        # # get new cost_index
        # for i in range(self.K, -1, -1):
        #     if(np.linalg.norm(self.pre_traj[i]-self.target) > 0.01):
        #         break
        # self.cost_index = i
        # get new cost_index
        for i in range(self.K, -1, -1):
            # if(np.linalg.norm(self.pre_traj[i]-self.target) > zy.parameters['radius']):
            # if(np.linalg.norm(self.pre_traj[i]-self.target) > 1e-2):
            # if( np.linalg.norm( self.pre_traj[i]-self.target ) > 0.1 * self.radius ):
            if( np.linalg.norm( self.pre_traj[i]-self.target ) > 0.01 * self.physical_radius  ):
                
                break  # 小于等于半径，算到终点。 必须要脚踩目标，或这边触及目标。 现在必须非常小
        self.cost_index = i

        return None

    def get_nei_objective(self, share_data):

        pre_traj_list = share_data['pre_traj']

        # P_neighbor includes all horizon's position
        P_neighbor = np.zeros(self.K*self.D)

        P1 = pre_traj_list[self.neighbor[0]][1:self.K+1].reshape(1, -1)[0]
        P2 = pre_traj_list[self.neighbor[1]][1:self.K+1].reshape(1, -1)[0]
        P_neighbor += 1.5/2*P1+0.5/2*P2

        return P_neighbor


#######################################################
#                                                     #
#                                                     #
#######################################################
#                                                     #
#                                                     #
#######################################################

    def get_coef_matrix(self):

        D = self.D
        K = self.K

        # position matrix
        # get all position matrix
        global Phi
        Phi = np.column_stack((np.eye(D), np.zeros((D, D))))
        phi = Phi
        for i in range(1, K):
            Phi = lg.block_diag(Phi, phi)
        self.Phi = Phi

        # get K position matrix
        global Phi_K
        Phi_K = np.zeros((D, K*D))
        for i in range(0, D):
            Phi_K[i][K*D-D+i] = 1.0
        self.Phi_K = Phi_K @ Phi

        # velocity matrix
        global Xi
        Xi = np.column_stack((np.zeros((D, D)), np.eye(D)))
        xi = Xi
        for i in range(1, K):
            Xi = lg.block_diag(Xi, xi)
        self.Xi = Xi

        # get K velocity matrix
        global Xi_K
        Xi_K = np.zeros((D, K*D))
        for i in range(0, D):
            Xi_K[i][K*D-D+i] = 1.0
        self.Xi_K = Xi_K @ Xi

        # gamma this matrix is used for the maximium input control constraint
        theta_u = np.array([1.0, 1.0])
        Theta_u = theta_u
        for i in range(1, K):
            Theta_u = lg.block_diag(Theta_u, theta_u)
        self.Theta_u = Theta_u

        self.Theta_v = Theta_u.copy()

        self.Theta_p = Theta_u.copy()

        # control input change cost

        Delta = np.eye(K*D)
        for i in range(D):
            Delta[i][i] = 0
        for i in range(D, K*D):
            Delta[i][i-D] = -1

        self.Delta = Delta.T @ Delta

        Delta_P = np.zeros((K*D, K*D))
        for i in range(1, K):
            for j in range(D):
                Delta_P[i*D+j][i*D+j] = i/K
                Delta_P[i*D+j][i*D-D+j] = -i/K

        self.Delta_P = Delta_P.T @ Delta_P

        return None

    def get_dynamic(self):

        K = self.K
        h = self.h

        # system dynamic in continous time
        A = np.array([[0, 0, 1, 0], [0, 0, 0, 1], [0, 0, 0, 0], [0, 0, 0, 0]])
        B = np.array([[0, 0], [0, 0], [1, 0], [0, 1]])

        m = A.shape[0]

        # system dynamic
        A = np.dot(np.linalg.inv(np.eye(m)-h/2*A), (np.eye(m)+h/2*A))
        B = np.dot(np.linalg.inv(np.eye(m)-h/2*A)*h, B)

        VA = A
        for i in range(2, K+1):
            C = np.eye(m)
            for j in range(1, i+1):
                C = np.dot(C, A)
            VA = np.block([[VA], [C]])
        self.VA = VA

        VB = B
        for i in range(1, K):
            VB = np.block([[np.dot(np.zeros((m, m)), B)], [VB]])
        for i in range(1, K):
            C = np.dot(matrixPow(A, i-K+1), B)
            for j in range(i-K+2, i+1):
                C = np.block([[C], [np.dot(matrixPow(A, j), B)]])
            VB = np.block([[C, VB]])
        self.VB = VB

        self.VC = np.zeros(m*K)

        return None


# the power of matrix
def matrixPow(Matrix, n):
    if(type(Matrix) == list):
        Matrix = np.array(Matrix)
    if(n == 1):
        return Matrix
    elif(n == 0):
        return np.eye(Matrix.shape[0])
    elif(n < 0):
        return np.zeros(Matrix.shape)
    else:
        return np.matmul(Matrix, matrixPow(Matrix, n-1))
