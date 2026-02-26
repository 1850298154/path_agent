from geometry import *
import SET
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Circle
import output_filename as of
import COLOR
import zrand as zr
# color=[ '#1f77b4',
#         '#ff7f0e',
#         '#2ca02c',
#         '#d62728',
#         '#9467bd',
#         '#8c564b',
#         '#e377c2',
#         '#7f7f7f',
#         '#bcbd22',
#         '#17becf',
#         '#2F4F4F',
#         '#CD5C5C',
#         '#ADD8E6',
#         '#663399',
#         '#8FBC8F',
#         '#00CED1',
#         '#6A5ACD',
#         '#808000',
#         '#A0522D',
#         '#FF4500',
#         '#708090',
#         '#BDB76B',
#         '#FF6347',
#         '#E9967A',
#         '#F5DEB3',
#         '#FFB6C1',
#         '#556B2F',
#         '#008080',
#         '#7FFF00',
#         '#FFA500',
#         '#FF8C00',
#         '#00FF7F',
#         '#C0C0C0',
#         '#483D8B',
#         '#F08080',
#         '#D3D3D3',
#         '#66CDAA',
#         '#FA8072',
#         '#F4A460',
#         '#48D1CC',
#         '#8A2BE2',
#         '#2E8B57']

color = COLOR.colors_hex


def plot_corridor_list(plane_list):

    if plane_list is None:
        return None

    i = 0
    for segment_plane in plane_list:

        i = i+1
        for plane in segment_plane:
            if abs(plane[0]) > abs(plane[1]):
                y1 = 0
                y2 = 1
                x1 = -plane[2]/plane[0]
                x2 = -(plane[1]+plane[2])/plane[0]
            else:
                x1 = 0
                x2 = 1
                y1 = -plane[2]/plane[1]
                y2 = -(plane[0]+plane[2])/plane[1]
            plt.axline([x1, y1], [x2, y2], linestyle='--',
                       color=color[i], linewidth=0.5)


def plot_obstacle(obstacle_list, extend=False):

    for ob in obstacle_list:
        X = []
        Y = []
        for v in ob.vertex_list:
            X += [v[0]]
            Y += [v[1]]
        if not extend:
            plt.fill(X, Y, c='forestgreen', alpha=0.3)
        else:
            plt.fill(X, Y, c='#D3D3D3', alpha=0.3)


def plot_grid_map(grid_map, res):

    print(grid_map)
    print(res)

    x_0 = SET.map_range['x'][0]
    y_0 = SET.map_range['y'][0]

    for i in range(grid_map.shape[0]):
        for j in range(grid_map.shape[1]):
            if grid_map[i][j] == 1:
                X = [x_0+i*res, x_0+(i+1)*res, x_0+(i+1)*res, x_0+i*res]
                Y = [y_0+j*res, y_0+j*res, y_0+(j+1)*res, y_0+(j+1)*res]
                plt.fill(X, Y, c='k')


def plot_convex_plane(points, segment_plane, obstacle_list):

    plt.figure(figsize=(10, 5))

    plot_obstacle(obstacle_list)

    for plane in segment_plane:
        if abs(plane[0]) > abs(plane[1]):
            y1 = 0
            y2 = 1
            x1 = -plane[2]/plane[0]
            x2 = -(plane[1]+plane[2])/plane[0]
        else:
            x1 = 0
            x2 = 1
            y1 = -plane[2]/plane[1]
            y2 = -(plane[0]+plane[2])/plane[1]
        plt.axline([x1, y1], [x2, y2], linestyle='--', linewidth=0.2)

    plt.scatter(points[0], points[1])

    plt.show()
    plt.close()


def plot_connect(connect_list, agent_list):

    for connect in connect_list:

        p1 = agent_list[connect[0]].p
        p2 = agent_list[connect[1]].p

        x = [p1[0], p2[0]]
        y = [p1[1], p2[1]]

        plt.plot(x, y, "--", linewidth=3, c='r')


def plot_pre_traj(agent_list, obstacle_list, show, episodes):

    if show:
        # fig = plt.figure(figsize=SET.plot_range['size'])
        fig = plt.figure(figsize=(10, 10))
        axes = fig.subplots(1, 1)
        for i in range(len(agent_list)):
            # 当前第0步位置
            circle = Circle(
                # xy=agent_list[i].pre_traj[0], radius=SET.ExtendWidth, fc=color[i], ec='k', lw=1.0)
                xy=agent_list[i].pre_traj[0], radius=agent_list[i].physical_radius, fc=color[i], ec='k', lw=1.0)
            plt.annotate(str(i),                 # 数字文本
                         agent_list[i].pre_traj[0],  # 菱形的中心坐标
                         #  (agent_list[i].target[0],
                         #   agent_list[i].target[1]),  # 菱形的中心坐标
                         textcoords="offset points",  # 文本的坐标系设置为偏移点
                         xytext=(0, 10),         # 文本位置相对于注释点的偏移
                         ha='center',           # 水平对齐方式设为居中
                         fontsize=12)           # 字体大小设为12

            axes.add_patch(p=circle)
            # 生成等间隔刻度的列表
            num_ticks = 51
            x_ticks = np.linspace(0, zr.set_xlim, num_ticks)
            y_ticks = np.linspace(0, zr.set_ylim, num_ticks)
            # 将数字刻度垂直显示
            axes.tick_params(axis='x', labelrotation=90)
            # 设置x轴刻度
            plt.xticks(x_ticks)
            plt.yticks(y_ticks)

            # 当前0~1步走的线段
            plt.plot(agent_list[i].pre_traj[0:2, 0],
                     agent_list[i].pre_traj[0:2, 1], c=color[i], linewidth=3)
            # 当前1步的位置，比第0步小一点，用来表示方向
            plt.plot(agent_list[i].pre_traj[1:, 0], agent_list[i].pre_traj[1:, 1], marker='o', zorder=4,
                     markeredgecolor='k', linewidth=3, markersize=10, c=color[i])

            # 终点
            plt.scatter(agent_list[i].target[0], agent_list[i].target[1], marker='d', s=400, zorder=2,
                        edgecolor='k', color=color[i])

            # x = [agent_list[i].pre_traj[-1][0],
            #      agent_list[i].tractive_point[0]]
            # y = [agent_list[i].pre_traj[-1][1],
            #      agent_list[i].tractive_point[1]]
            # # 到终点的虚线
            # plt.plot(x, y, ':', c=color[i], linewidth=4)

            if SET.REALFLY:

                polyx = agent_list[i].input_traj[0]
                polyy = agent_list[i].input_traj[1]

                X = SET.K*SET.h*np.arange(100)/100
                plt.plot(np.polyval(polyx, X), np.polyval(
                    polyy, X), ':', c=color[i])

        plot_obstacle(obstacle_list)

        plt.xlim(SET.plot_range['x'])
        plt.ylim(SET.plot_range['y'])
        # plt.show()
        filename = of.path_dir+'savefig/episode-'+str(episodes)+SET.format
        of.create_file(filename)
        plt.savefig(filename, bbox_inches='tight')
        plt.close()

    return None


def plot_all_pre_traj(agent_list, ini_obstacle_list, obstacle_list, show, episodes):
    episodes_path_list=[]
    ob_rate = 2 # 原来地图 figure size 20*20， 现在 10*10
    # if show:
    # for i in range(len(agent_list)):
    for step in range(len(agent_list[0].pre_traj_list)):
        # fig = plt.figure(figsize=SET.plot_range['size'])
        fig = plt.figure(figsize=(10, 10))
        axes = fig.subplots(1, 1)
        # 生成等间隔刻度的列表
        num_ticks = 51
        x_ticks = np.linspace(0, zr.set_xlim, num_ticks)
        y_ticks = np.linspace(0, zr.set_ylim, num_ticks)
        # 将数字刻度垂直显示
        axes.tick_params(axis='x', labelrotation=90)
        # 设置x轴刻度
        plt.xticks(x_ticks)
        plt.yticks(y_ticks)

        # for step in range(len(agent_list[0].pre_traj_list)):
        for i in range(len(agent_list)):
            # 当前第0步位置
            # circle = Circle(
            #     # xy=agent_list[i].pre_traj[0], radius=SET.ExtendWidth, fc=color[i], ec='k', lw=1.0)
            #     # xy=agent_list[i].pre_traj_list[step][0], radius=SET.ExtendWidth, fc=color[i], ec='k', lw=1.0)
            #     xy=agent_list[i].pre_traj_list[step][0], 
            #     radius=agent_list[i].physical_radius, 
            #     fc=color[i], 
            #     ec='k', 
            #     # ec=color[i], 
            #     lw=0.5,
            #     # lw=1.0,
            #     zorder=5,
            #     )
            import zyaml as zy  
            # 当前第0步位置
            # circle2 = Circle(
            #     # xy=agent_list[i].pre_traj[0], radius=SET.ExtendWidth, fc=color[i], ec='k', lw=1.0)
            #     # xy=agent_list[i].pre_traj_list[step][0], radius=SET.ExtendWidth, fc=color[i], ec='k', lw=1.0)
            #     xy=agent_list[i].pre_traj_list[step][0], 
            #     radius=zy.parameters['radius'], 
            #     fc=color[i], 
            #     ec='k', 
            #     # ec=color[i], 
            #     lw=0.5,
            #     # lw=1.0,
            #     zorder=5,
            #     alpha=0.5
            #     )

            UnmannedSystem_opt = [
                'unicycle',
                'Mecanum',
                'Quadcopter',
                'patrol_missile',
                'fixed_wing',]
            # self.UnmannedSystem = self.UnmannedSystem_opt[1]
            UnmannedSystem = zr.UnmannedSystem_list[agent_list[i].index]
            shape_list = ['o','P','X','p','*','>']
            index_unmanned = UnmannedSystem_opt.index(UnmannedSystem)
            marker_shape = shape_list[index_unmanned]

            plt.plot(agent_list[i].position[step, 0], 
                agent_list[i].position[step, 1],
                marker=marker_shape,
                zorder=2, c=color[i], linewidth=4)
            # radius 圆圈大小
            # ec 是指圆圈边界（edge）的颜色，这里设置为 'k' 表示黑色。
            # lw 是指圆圈边界（edge）的线宽（linewidth），这里设置为 1.0。
            plt.annotate(str(i),                 # 数字文本
                         #  agent_list[i].pre_traj[0],  # 菱形的中心坐标
                         agent_list[i].pre_traj_list[step][0],  # 菱形的中心坐标
                         #  (agent_list[i].target[0],
                         #   agent_list[i].target[1]),  # 菱形的中心坐标
                         textcoords="offset points",  # 文本的坐标系设置为偏移点
                         xytext=(0, 10),         # 文本位置相对于注释点的偏移
                         ha='center',           # 水平对齐方式设为居中
                         fontsize=6)           # 字体大小设为12
                        #  fontsize=12)           # 字体大小设为12

            # xy=agent_list[i].pre_traj[0]：指定圆的中心点坐标，agent_list[i].pre_traj[0] 是一个包含 x 和 y 坐标的数组。
            # radius=SET.ExtendWidth：指定圆的半径大小，SET.ExtendWidth 是一个表示半径的变量。
            # fc=color[i]：指定圆的填充颜色，color[i] 是一个字符串，表示颜色的名称或十六进制值。
            # ec='k'：指定圆的边界颜色为黑色。
            # lw=1.0：指定圆的边界线宽度为 1.0。

            # axes.add_patch(p=circle)
            # axes.add_patch(p=circle2)

            # # 这个半径跟真实半径对不上
            # # plt.plot(agent_list[i].pre_traj[0:2, 0],
            # #          agent_list[i].pre_traj[0:2, 1], c=color[i], linewidth=3)
            # # plt.plot(agent_list[i].pre_traj[1:, 0], agent_list[i].pre_traj[1:, 1], marker='o', zorder=4,
            # #          markeredgecolor='k', linewidth=3, markersize=10, c=color[i])
            # # 当前0~1步走的线段
            # # plt.plot(agent_list[i].pre_traj_list[step][0:2, 0],
            # #          agent_list[i].pre_traj_list[step][0:2, 1],
            # plt.plot(agent_list[i].pre_traj_list[step][0:1, 0],
            #          agent_list[i].pre_traj_list[step][0:1, 1],
            #         #  c=color[i],
            #         #  linewidth=3/ob_rate)
            #          marker='o',  # 要加线 ， 使用 'o-'
            #          zorder=4,
            #         #  markeredgecolor='k',
            #          markeredgecolor=color[i],
            #          linewidth=3/ob_rate,  # 还真有线
            #          markersize=10/ob_rate,
            #          c=color[i],
            #          alpha=0.5,
            #          )

            # agent_list[i].pre_traj[0:2, 0] 和 agent_list[i].pre_traj[0:2, 1]：指定要绘制的线条的 x 和 y 坐标数组，用于绘制线段的起始部分。
            # c=color[i]：指定线条的颜色，color[i] 是一个字符串，表示颜色的名称或十六进制值。
            # linewidth=3：指定线条的宽度为 3 像素。
            # 没有指明标记符号，默认情况下，plt.plot() 函数会使用线条作为标记符号

            # 当前1:K步的位置，比第0步小一点，用来表示方向
            # plt.plot(agent_list[i].pre_traj_list[step][1:, 0],
            #          agent_list[i].pre_traj_list[step][1:, 1],
            plt.plot(agent_list[i].pre_traj_list[step][:, 0],
                     agent_list[i].pre_traj_list[step][:, 1],
                    #  marker='o',  # 要加线 ， 使用 'o-'
                    #  zorder=4,
                    #  markeredgecolor='k',
                    #  linewidth=3/ob_rate,  # 还真有线
                    #  markersize=10/ob_rate,
                    #  c=color[i])
                     c=color[i],
                     linewidth=3/ob_rate,
                     alpha=0.5,
                     )
            # agent_list[i].pre_traj[1:, 0] 和 agent_list[i].pre_traj[1:, 1]：指定要绘制的线条的 x 和 y 坐标数组，用于绘制线段的其余部分（除起始部分外）。
            # marker='o'：指定线条中每个数据点的标记符号为圆形。
            # zorder=4：指定绘图元素的层次顺序，数值越大表示绘制在更上层。
            # markeredgecolor='k'：指定标记符号的边界颜色为黑色。
            # linewidth=3：指定线条的宽度为 3 像素。
            # markersize=10：指定标记符号的大小为 10。

            # 起点
            plt.scatter(agent_list[i].position[0][0],
                        agent_list[i].position[0][1],
                        marker='s',
                        # s=40/ob_rate,
                        s=40,
                        zorder=1,
                        edgecolor='k',
                        color=color[i],
                        alpha=0.5,
                        )
            # 终点
            plt.scatter(agent_list[i].target[0],
                        agent_list[i].target[1],
                        marker='d',
                        s=40,
                        # s=40/ob_rate,
                        # s=400/ob_rate,
                        zorder=2,
                        edgecolor='k',
                        color=color[i],
                        alpha=0.5,
                        )
            # agent_list[i].target[0] 和 agent_list[i].target[1]：指定散点的 x 和 y 坐标位置。
            # marker='d'：指定散点的标记符号为菱形。
            # s=400：指定散点的尺寸为 400 像素平方。
            # 如果你想要根据图的大小来调整散点的大小，
            # 你可以将 s 的值设置为一个相对于图的大小的比例值，
            # 而不是一个固定的像素面积值。
            # 例如，你可以根据图的大小，将 s 设置为图面积的某个比例来控制散点的尺寸。
            # zorder=2：指定绘图元素的层次顺序，数值越大表示绘制在更上层。
            # edgecolor='k'：指定散点的边界颜色为黑色。
            # color=color[i]：指定散点的颜色，color[i] 是一个字符串，表示颜色的名称或十六进制值。

            plt.annotate(str(i),                 # 数字文本
                            (agent_list[i].target),  # 菱形的中心坐标
                        #  (agent_list[i].target[0],
                        #   agent_list[i].target[1]),  # 菱形的中心坐标
                            textcoords="offset points",  # 文本的坐标系设置为偏移点
                            xytext=(0, 10),         # 文本位置相对于注释点的偏移
                            ha='center',           # 水平对齐方式设为居中
                            fontsize=6)           # 字体大小设为12
            # 其中text是要写上去的数字文本，(agent_list[i].target[0], agent_list[i].target[1])是菱形的中心坐标，xytext=(0,10)表示文本相对于注释点向上偏移10个像素，ha='center'表示水平方向居中对齐，fontsize=12是字体大小。

            # # x = [agent_list[i].pre_traj[-1][0],
            # x = [agent_list[i].pre_traj_list[step][-1][0],
            #      agent_list[i].step_tractive[step][0]]
            #     #  agent_list[i].tractive_point[0]]
            # # y = [agent_list[i].pre_traj[-1][1],
            # y = [agent_list[i].pre_traj_list[step][-1][1],
            #      agent_list[i].step_tractive[step][1]]
            #     #  agent_list[i].tractive_point[1]]

            # # 到终点的虚线
            # plt.plot(x, y,
            #          ':',
            #          c=color[i],
            #         #  linewidth=4/ob_rate)
            #          linewidth=2/ob_rate,
            #         #  alpha=0.5,
            #          )
            # # x 和 y：指定要绘制的线条的 x 和 y 坐标数组。
            # # ':'：指定线条的线型为虚线，可以根据需要选择其他线型。
            # # c=color[i]：指定线条的颜色，color[i] 是一个字符串，表示颜色的名称或十六进制值。
            # # linewidth=4：指定线条的宽度为 4 像素。

        if SET.REALFLY:

            polyx = agent_list[i].input_traj[0]
            polyy = agent_list[i].input_traj[1]

            X = SET.K*SET.h*np.arange(100)/100
            plt.plot(np.polyval(polyx, X), np.polyval(
                polyy, X), ':', c=color[i])

        plot_obstacle(ini_obstacle_list)
        plot_obstacle(obstacle_list)
        axes.set_ylabel('Y', fontdict={'size': 10, 'color': 'red'})
        axes.set_xlabel('X', fontdict={'size': 10, 'color': 'red'})

        plt.xlim(SET.plot_range['x'])
        plt.ylim(SET.plot_range['y'])
        # plt.show()
        # filename = of.path_dir+'savefig/episode-'+str(episodes)+SET.format
        filename = of.path_dir+'savefig/episode-'+str(step)+SET.format
        episodes_path_list.append(filename)
        of.create_file(filename)
        plt.savefig(filename, bbox_inches='tight')
        plt.close()
        # if step == 10:
        #     import time
        #     # time.sleep(0.1)
        #     # time.sleep(0.05) # k
        #     # time.sleep(0.5)
        #     time.sleep(1)  # k

    return episodes_path_list

# plot
# plot

import uav 
from typing import List
def plot_position(
        agent_list:List[uav.uav2D], 
        ini_obstacle_list, 
        obstacle_list, 
        twice_base_line_bool=False,
        ):

    # fig = plt.figure(figsize=SET.plot_range['size'])
    fig = plt.figure(figsize=(10, 10))

    axes = fig.subplots(1, 1)

    # 生成等间隔刻度的列表
    num_ticks = 51
    x_ticks = np.linspace(0, zr.set_xlim, num_ticks)
    y_ticks = np.linspace(0, zr.set_ylim, num_ticks)
    # 将数字刻度垂直显示
    axes.tick_params(axis='x', labelrotation=90)
    # 设置x轴刻度
    plt.xticks(x_ticks)
    plt.yticks(y_ticks)
    axes.set_ylabel('Y', fontdict={'size': 10, 'color': 'red'})
    axes.set_xlabel('X', fontdict={'size': 10, 'color': 'red'})


    for i in range(SET.Num):
        UnmannedSystem_opt = [
            'unicycle',
            'Mecanum',
            'Quadcopter',
            'patrol_missile',
            'fixed_wing',]
        # self.UnmannedSystem = self.UnmannedSystem_opt[1]
        UnmannedSystem = zr.UnmannedSystem_list[agent_list[i].index]
        shape_list = ['o','P','X','p','*','>']
        index_unmanned = UnmannedSystem_opt.index(UnmannedSystem)
        marker_shape = shape_list[index_unmanned]


        if agent_list[i].type == "Anchor":
            continue

        # plt.scatter(agent_list[i].position[0][0],
        #             agent_list[i].position[0][1],
        plt.scatter(agent_list[i].ini_p[0],
                    agent_list[i].ini_p[1],
                    marker='s',s=40,zorder=1,edgecolor='k',color=color[i],
                    alpha=0.5,
                    )
                    # marker='s',s=400,zorder=1,edgecolor='k',color=color[i])
        plt.scatter(agent_list[i].target[0], 
                    agent_list[i].target[1],
                    marker='d', s=40, zorder=3, edgecolor='k', color=color[i],
                    alpha=0.5,
                    )
                    # marker='d', s=600, zorder=3, edgecolor='k', color=color[i])
        # Use path if position is 1D (no simulation history)
        pos_data = agent_list[i].path if agent_list[i].position.ndim == 1 else agent_list[i].position
        plt.plot(
                pos_data[:, 0][::-1],
                pos_data[:, 1][::-1],
                zorder=2, c=color[i], linewidth=4,
                alpha=0.3,
                marker=marker_shape,
                )
                # agent_list[i].position[:, 1],1
                # agent_list[i].position[:, 2], 
                # zorder=2, c=color[i], linewidth=4)
        
    # 轨迹用上面的先挑画出来
    # for j in range(len(agent_list[0].position)):
    #     for i in range(SET.Num):
    #         if agent_list[i].type == "Anchor":
    #             continue

    #         circle = Circle(
    #             xy=agent_list[i].position[j], 
    #             radius=SET.ExtendWidth, fc=color[i], ec=color[i])
    #             # xy=agent_list[i].position[j], radius=SET.ExtendWidth, fc=color[i], ec='k')

    #         axes.add_patch(p=circle)

    plot_obstacle(ini_obstacle_list)
    plot_obstacle(obstacle_list)

    if twice_base_line_bool == True:
        from dead_record import pair_timeline, dead_timeline, g_dead_list, g_p_list
        g_dead_list:List[dead_timeline] = g_dead_list
        print('plot g_dead_list')
        for d in g_dead_list:

            # 计算中心点
            center = np.mean([d.pi, d.pj], axis=0)

            # 计算半径
            radius = 1.5 * np.linalg.norm(np.array(d.pi) - np.array(d.pj))
            radius = 10+1.5 * np.linalg.norm(np.array(d.pi) - np.array(d.pj))

            # # 创建图形
            # fig, ax = plt.subplots()

            # 绘制圆圈
            circle = plt.Circle(center, radius, 
                                zorder=5,
                                color='red', 
                                linewidth=3,
                                fill=False,
                                )
            axes.add_patch(circle)

            # # 绘制 pi 和 pj 点
            # ax.plot(pi[0], pi[1], 'bo')  # pi 点，蓝色
            # ax.plot(pj[0], pj[1], 'go')  # pj 点，绿色


    plt.xlim(SET.plot_range['x'])
    plt.ylim(SET.plot_range['y'])

    filename = of.path_dir+'savefig/trajecotry'+SET.format
    # import zyaml as zy
    if twice_base_line_bool == True:
        filename += '.baseline_bool'+SET.format
    of.create_file(filename)
    plt.savefig(filename, bbox_inches='tight')
    # plt.show()
    plt.close()


def plot_path_planning(agent_list):

    # plt.figure(figsize=SET.plot_range['size'])
    fig = plt.figure(figsize=(10, 10))
    axes = fig.add_subplot(111)
    # 生成等间隔刻度的列表
    num_ticks = 51
    x_ticks = np.linspace(0, zr.set_xlim, num_ticks)
    y_ticks = np.linspace(0, zr.set_ylim, num_ticks)
    # 将数字刻度垂直显示
    axes.tick_params(axis='x', labelrotation=90)
    # 设置x轴刻度
    plt.xticks(x_ticks)
    plt.yticks(y_ticks)

    # plot_obstacle(SET.path_plot_obstacle_list,extend=True)

    plot_obstacle(SET.obstacle_list)

    i = 0

    for agent in agent_list:
        if agent.type == "Obstacle-transitor" or agent.type == "Free-transitor" or agent.type == "Searcher":
            if agent.path is not None:
                # print('agent.path')
                # print(type(agent.path))
                # print(agent.path)
                # print(*agent.path, sep='\n')
                # print(flush=True)
                # print('agent.path[:,0]')
                # print(type(agent.path[:, 0]))
                # print(agent.path[:, 0])
                # print(*agent.path[:, 0], sep='\n')
                # print('agent.path[:,1]')
                # print(type(agent.path[:, 1]))
                # print(agent.path[:, 1])
                # print(*agent.path[:, 1], sep='\n')
                # print(flush=True)
                # plt.plot的语法是 np.array(list)
                plt.plot(agent.path[:, 0], agent.path[:, 1], c=color[i])
                plt.xlim(SET.plot_range['x'])
                plt.ylim(SET.plot_range['y'])

            plt.scatter(agent.terminal_p[0], agent.terminal_p[1],
                        marker='o', s=40, zorder=2, edgecolor='k', color=color[i])
                        # marker='o', s=400, zorder=2, edgecolor='k', color=color[i])
            plt.scatter(agent.target[0], agent.target[1], marker='d',
                        s=40, zorder=3, edgecolor='k', color=color[i])
                        # s=600, zorder=3, edgecolor='k', color=color[i])
            i = i+1

    filename = of.path_dir+'savefig/path'+SET.format
    of.create_file(filename)
    plt.savefig(filename, bbox_inches='tight')

    # plt.show()
    plt.close()


def plot_circle(obstacle_list, connection_constraint_list, episodes):

    plt.figure(figsize=SET.plot_range['size'])

    t = np.block([[np.cos(2*np.pi * np.arange(200)/200),
                 np.sin(2*np.pi * np.arange(200)/200)]])
    t = t.reshape((200, 2), order='F')

    for connection_constraint in connection_constraint_list:
        center_list = connection_constraint[0]
        r_list = connection_constraint[1]

        for i in range(len(center_list)):
            x_c = center_list[i]+r_list[i]*t
            plt.plot(x_c[:, 0], x_c[:, 1])

    plot_obstacle(obstacle_list)
    # plot_corridor_list(agent_list[0].ob_corridor_list)

    plt.xlim(SET.plot_range['x'])
    plt.ylim(SET.plot_range['y'])
    # plt.show()
    filename = of.path_dir+'savefig/e'+str(episodes)+SET.format
    of.create_file(filename)
    plt.savefig(filename)
    plt.close()

    return None
