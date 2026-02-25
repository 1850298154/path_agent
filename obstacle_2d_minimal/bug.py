import output_filename as of
import zrand as zr
# from bug5 import *
# from bug6 import *
# from bug7 import *
# from bug8 import *
# from bug9 import *
# from bug10 import *
from bug11 import *
# import math
# import numpy as np
# import time
# import copy
# import matplotlib.pyplot as plt

cnt = 0


# def path_plan(start_point, end_point, obstacle_list, step_size=zr.bug_step_size, inflated_size=zr.inflated_size):
def path_plan(agent_index, start_point, end_point, obstacle_list, 
              step_size=zr.bug_step_size, inflated_size=zr.more_inflated_size,
              ):
    print('agent_index      : ', agent_index)
    print('start_point      : ', start_point)
    print('end_point        : ', end_point)
    print('obstacle_list    : ', obstacle_list)
    print('step_size        : ', step_size)
    print('inflated_size    : ', inflated_size)

    # obscacles
    # [center_x, center_y], width, height, 实际矩形长宽
    # obstacle_list = [[np.array([2.0, 2.0]), 1.0, 1.0],
    #                  [np.array([4.0, 4.0]), 2.0, 2.0],
    #                  [np.array([7.0, 7.0]), 1.0, 1.0]]

    # start_point = np.array([1.0, 2.0])
    # end_point = np.array([9.0, 9.0])

    # ws_size = [0, 10, 0, 10]  # 限制区域， 但是没有用到

    time_start = time.time()
    bug_planner = BugPlanner(start_point, end_point,
                             step_size, inflated_size, obstacle_list)
    # bug_planner.nearest_intersection()
    # bug_planner.step_toward_obstacle()
    # bug_planner.nearest_obstacle()
    # bug_planner.find_nearest_corner()

    bug_planner.run()
    final_path = bug_planner.path
    time_end = time.time()
    total_time = time_end - time_start
    print(total_time)

    # print(final_path)

    # fig = plt.figure()
    # ax = fig.add_subplot(111)

    # # ax.set_xlim([0, 10])
    # # ax.set_ylim([0, 10])
    # ax.set_xlim([0, zr.set_xlim])
    # ax.set_ylim([0, zr.set_ylim])
    # ax.set_aspect('equal')

    # bug_planner.plot_rectangulars(ax)
    # bug_planner.plot_path(ax)

    # # line = Line(start_point, end_point)
    # # ax = line.plot_line(ax)

    # # plt.plot(start_point, end_point, 'r', linewidth=1.5)
    # #
    # # center_rect_i = (rect_center[0] - rect_width / 2, rect_center[1] - rect_height / 2)
    # # rect_i = plt.Rectangle(center_rect_i, rect_width, rect_height, edgecolor='k',
    # #                        facecolor='k', linewidth=1.5, fill=False)
    # # ax.add_patch(rect_i)
    # #
    # # plt.show()
    # global cnt
    # cnt = agent_index
    # filename = of.path_dir+'/bug_planner/' + str(cnt) + '.png'
    # import os
    # if not os.path.exists(filename):
    #     # cnt += 1 # 用index 不用 +1
    #     of.create_file(filename)
    # else:
    #     exist_no = 1
    #     filename = of.path_dir+'/bug_planner/' + str(cnt) + '-' + str(exist_no) + '.png'
    #     while os.path.exists(filename):
    #         exist_no += 1
    #         filename = of.path_dir+'/bug_planner/' + str(cnt) + '-' + str(exist_no) + '.png'
    # plt.savefig(filename)
    return np.array(final_path)


def obstacle_adapter(obstacle_list=zr.obstacles):
    obstacle_list = [
        [
            [
                ob[0]+ob[-1]/2,
                ob[1]+ob[-1]/2
            ],
            ob[-1],
            ob[-1]
        ]
        for ob in obstacle_list]
    return obstacle_list


def send2wsk():
    # info_list       [(2, 201.95519065842674)]
    obstacle_list=[(198.1614385518977, 168.60392428676852, 50.0), (64.20016593092528, 57.458110032562736, 50.0), (39.55418822256267, 158.0988433861503, 50.0), (158.17594654411153, 57.03676336852559, 50.0), (116.43565691766102, 142.3280620894149, 50.0)]

    start_point=[(214.9004439665877, 233.88056443304234)]

    end_point=[(189.90750992631965, 21.41688294597475)]
    obstacle_list = obstacle_adapter(obstacle_list)
    print('obstacle_list =')
    print(obstacle_list)
    # path_plan(start_point, end_point, obstacle_list)

# send2wsk()
# exit()

if __name__ == '__main__':
    # obscacles
    # [center_x, center_y], width, height, 实际矩形长宽
    # obstacle_list = [[np.array([2.0, 2.0]), 1.0, 1.0],
    #                  [np.array([4.0, 4.0]), 2.0, 2.0],
    #                  [np.array([7.0, 7.0]), 1.0, 1.0]]
    obstacle_list = [[2.0, 2.0, 1.0],
                     [4.0, 4.0, 2.0],
                     [7.0, 7.0, 1.0]]
    obstacle_list = obstacle_adapter(obstacle_list)
    # start_point = np.array([1.0, 2.0])
    # end_point = np.array([9.0, 9.0])
    start_point = np.array([1.0, 2.0])
    end_point = np.array([9.0, 9.0])

    path_plan(start_point, end_point, obstacle_list)
