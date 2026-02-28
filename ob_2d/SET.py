# forest_SET.py
# 修改版，放100个agent
import numpy as np
# from geometry import *
import geometry
import zyaml as zy
import zrand as zr


def initialize_set():

    ###################
    #  personal set   #
    ###################

    global Num     # the number of agents
    # Num=8
    Num = zr.Num

    # maximum acc
    global Umax  # zyt2
    Umax = 2.0
    Umax = zy.parameters['Umax']

    # maximum velocity
    global Vmax  # zyt2
    Vmax = 3.0
    Vmax = zy.parameters['Vmax']

    global type_list
    # type_list=["Obstacle-transitor","Obstacle-transitor","Obstacle-transitor","Obstacle-transitor",\
    #            "Obstacle-transitor","Obstacle-transitor","Obstacle-transitor","Obstacle-transitor"]

    type_list = [
        "Obstacle-transitor"
        for _ in range(Num)
    ]

    global K       # the length of horizon
    # K = 2
    K = 8
    # K = 9
    global h       # time step
    h = 0.15
    global episodes      # the maximum times for replanning
    # episodes=70
    episodes = zr.episodes
    # the mode that running the convex programming, 'norm' , 'process' and 'thread'
    global compute_model
    compute_model = 'norm'
    # if the mode is 'process', then, choosing the number of core apllied for this computing
    global core_num
    core_num = 4
    core_num = 1
    global map_range
    # map_range={'x':[-0.5,10.5],'y':[-0.5,10.5]}
    map_range = {'x': [-zr.radius, zr.set_xlim+zr.radius],
                 'y': [-zr.radius, zr.set_ylim+zr.radius]}

    ## parameter for real fly ##

    global REALFLY       # if REALFLY=True, the real fly experiment for using crayfiles
    REALFLY = False
    global height        # the flying height for 2d-test when using crazyfiles
    height = 0.5
    global interval      # For real fly, it is an integer a little larger than computation_time/h
    interval = 1
    global next_interval
    next_interval = 1

    ## about the fig ##

    # if show = True, then the fig will be save in file: savefig.
    global show
    show = True            # Pay attentiion that, in realfly test, show must be chosen as 'False' since it will lead to the time delay

    global format  # the format of the printed fig
    format = '.jpg'

    global plot_range    # if save figure, determine the plot range of this figure
    plot_range = {'x': [map_range['x'][0]-0.5, map_range['x'][1] +
                        0.5], 'y': [map_range['y'][0]-0.5, map_range['y'][1]+0.5]}
    ratio = (plot_range['y'][1]-plot_range['y'][0]) / \
        (plot_range['x'][1]-plot_range['x'][0])
    plot_range['size'] = (20, ratio*20)

    global buffer        # the buffer when adopting parting plane linear constraints
    # buffer = 0.03        # "obstacle-transitor":0.02
    buffer = zr.buffer        # "obstacle-transitor":0.02

    # The extended width when considering the radius of agents, pay attention: it is radius
    global ExtendWidth
    # ExtendWidth = 0.3
    ExtendWidth = zr.radius
    # ExtendWidth = 0

    global r_min  # zyt2 直径
    # r_min = 2*ExtendWidth
    r_min = 2*zr.radius
    global epsilon
    epsilon = 0.20  # zyt2
    # epsilon = 2.20  # zyt2

    global resolution
    # resolution=0.1
    resolution = 1
    # resolution = 0.1

    global ini_x   # intial position
    # ini_x=[np.array([0.0,0.0]),np.array([10.0,0.0]),np.array([0.0,10.0]),np.array([10.0,10.0]),\
    #        np.array([0.0,5.0]),np.array([5.0,0.0]),np.array([10.0,5.0]),np.array([5.0,10.0]) ]
    ini_x = [
        np.array([agent[0], agent[1]])
        for agent in zr.agents_starts
    ]
    # ini_x = [
    #     # np.array([0, 0])
    #     np.array([0, 125])
    #     for agent in zr.agents_starts
    # ]

    # target position: is a variable in some condition
    global target
    # target=[np.array([10.0,10.0]),np.array([0.0,10.0]),np.array([10.0,0.0]),np.array([0.0,0.0]),\
    #         np.array([10.0,5.0]),np.array([5.0,10.0]),np.array([0.0,5.0]),np.array([5.0,0.0])]
    target = [
        np.array([agent[0], agent[1]])
        for agent in zr.agents_ends
    ]
    # target = [
    #     # np.array([250, 250])
    #     np.array([125, 250])
    #     for agent in zr.agents_ends
    # ]

    # the enviromrnt doen't consider the diameter of robots
    global ini_obstacle_list

    # ini_obstacle_list=[
    #     rectangle(np.array([1.0,1.5]),0.5,0.5,0.0),
    #     rectangle(np.array([4.0,1.0]),1.0,1.0,0.0),
    #     rectangle(np.array([8.0,1.5]),0.5,0.5,0.0),
    #     rectangle(np.array([1.5,3.5]),0.5,0.5,0.0),
    #     rectangle(np.array([5.5,4.0]),0.5,0.5,0.0),

    #     rectangle(np.array([9.0,4.5]),0.5,0.5,0.0),
    #     rectangle(np.array([1.0,6.0]),1.0,1.0,0.0),
    #     rectangle(np.array([7.0,7.0]),2.0,1.0,0.0),
    #     rectangle(np.array([3.5,8.5]),0.5,0.5,0.0)
    # ]
    ini_obstacle_list = [
        geometry.rectangle(np.array([ob[0], ob[1]]),
                           ob[2],
                           ob[2],
                           0.0)
        for ob in zr.obstacles
    ]

    # the obstacle-inflated enviroment which consider the diameter of robots
    global obstacle_list
    # obstacle_list=[
    #     rectangle(np.array([1.0,1.5]),0.5,0.5,ExtendWidth),
    #     rectangle(np.array([4.0,1.0]),1.0,1.0,ExtendWidth),
    #     rectangle(np.array([8.0,1.5]),0.5,0.5,ExtendWidth),
    #     rectangle(np.array([1.5,3.5]),0.5,0.5,ExtendWidth),
    #     rectangle(np.array([5.5,4.0]),0.5,0.5,ExtendWidth),

    #     rectangle(np.array([9.0,4.5]),0.5,0.5,ExtendWidth),
    #     rectangle(np.array([1.0,6.0]),1.0,1.0,ExtendWidth),
    #     rectangle(np.array([7.0,7.0]),2.0,1.0,ExtendWidth),
    #     rectangle(np.array([3.5,8.5]),0.5,0.5,ExtendWidth)
    # ]
    obstacle_list = [
        geometry.rectangle(np.array([ob[0], ob[1]]),
                           ob[2],
                           ob[2],
                           ExtendWidth)
        for ob in zr.obstacles
    ]

    # the enviroment used for path planning
    global path_obstacle_list

    # path_obstacle_list=np.loadtxt('map/forest.csv')

    # 好像可以不用了，这是 path_planning.py 调用
    # path_obstacle_list = Build_ExtensionZone(obstacle_list, 0.1)

    # path_obstacle_list = grid(path_obstacle_list, resolution, map_range)

    # np.savetxt('map/forest.csv', path_obstacle_list)
