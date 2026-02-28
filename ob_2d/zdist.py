from shapely.geometry import Point, LineString
import shapely
from shapely.geometry import LineString
import math
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon
# import obstacle_corridora

############################################################
###############     仅在文件中作为测试查看     ##############
############################################################


def plotPoly(polygons, x=0, y=100):
    polygons = [
        [
            v[:2]  # 前两个为x,y 什么膨胀啥的全部扔掉
            for v in vs
        ]
        for vs in polygons
    ]
    # # 凸多边形的顶点坐标
    # polygons = [
    #     [(1, 2), (4, 4), (5, 6)],  # 第一个多边形的顶点坐标
    #     [(7, 8), (9, 10), (11, 12), (13, 14)]  # 第二个多边形的顶点坐标
    # ]

    # 创建一个新的图形窗口
    fig, ax = plt.subplots()

    color = ['red', 'green', 'blue', 'yellow',
             'orange', 'purple', 'pink',
             'black', 'white',]
    # 绘制每条线段
    for i, vs in enumerate(polygons):
        for j, v in enumerate(vs):
            # ax.plot(v[0], v[1], marker='o', markersize=8, linewidth=2)
            ax.plot(v[0], v[1],
                    marker='o', markersize=8,
                    linewidth=2, color=color[i])

    # 绘制每个凸多边形
    for polygon in polygons:
        poly_patch = Polygon(polygon, closed=True,
                             edgecolor='black', linewidth=1, facecolor='none')
        #  edgecolor='black', linewidth=1, facecolor='pink')
        ax.add_patch(poly_patch)

    # 设置坐标轴范围
    ax.set_xlim([x, y])
    ax.set_ylim([x, y])

    # 设置坐标轴刻度长度相等
    plt.axis('equal')

    # 显示网格线
    ax.grid(True)

    # 显示图形
    plt.show()


############################################################
###############  先求所有顶点到另一个多边形边   ##############
############################################################
# def polygon_vertices_to_edge_distance():
def polygon_vertices_to_edge_distance(polygon1, polygon2_edge):

    def point_to_edge_distance(point, edge):
        # x1, y1 = edge[0]
        # x2, y2 = edge[1]
        x1, y1 = edge[0][:2]
        x2, y2 = edge[1][:2]

        # 判断线段是否为一个点 可能 edge_length = 0
        if x1 == x2 and y1 == y2:
            return math.sqrt((point[0] - x1) ** 2 + (point[1] - y1) ** 2)

        # 计算边向量
        edge_vector = [x2 - x1, y2 - y1]

        # 计算边的长度
        edge_length = math.sqrt(edge_vector[0] ** 2 + edge_vector[1] ** 2)

        # 将边向量标准化
        normalized_edge_vector = [edge_vector[0] /
                                  edge_length, edge_vector[1] / edge_length]

        # 计算从边起点到顶点的向量
        vertex_vector = [point[0] - x1, point[1] - y1]

        # 计算点到边的投影长度
        projection_length = vertex_vector[0] * normalized_edge_vector[0] + \
            vertex_vector[1] * normalized_edge_vector[1]

        if projection_length <= 0:
            # 如果点在边的起点的前方，返回起点到点的距离
            return math.sqrt((point[0] - x1) ** 2 + (point[1] - y1) ** 2)
        elif projection_length >= edge_length:
            # 如果点在边的终点的后方，返回终点到点的距离
            return math.sqrt((point[0] - x2) ** 2 + (point[1] - y2) ** 2)
        else:
            # 如果点在边的范围内，计算点到边的垂直距离
            vertical_distance = abs(
                (x2 - x1) * (y1 - point[1]) - (x1 - point[0]) * (y2 - y1)) / edge_length
            return vertical_distance

    # 示例使用
    # polygon1 = [(0, 0), (5, 0), (5, 5), (0, 5)]  # 第一个凸多边形的顶点坐标
    # polygon2_edge = [(2, 2), (8, 4)]  # 第二个凸多边形的一条边的端点坐标

    distances = []
    for vertex in polygon1:
        distance = point_to_edge_distance(vertex, polygon2_edge)
        distances.append(distance)

    # print(distances)
    # print(min(distances))
    return distances


############################################################
###########  再求所有顶点到另一个多边形所有边   ##############
############################################################
def edges(polygon1):
    # polygon1 = [[8.60070679, 2.73776152, 0.],
    #             [10.62070679, 2.73776152, 0.],
    #             [10.62070679, 4.75776152, 0.],
    #             [8.60070679, 4.75776152, 0.]]
    edges = []
    for i in range(len(polygon1)):
        edge = [polygon1[i], polygon1[(i + 1) % len(polygon1)]]
        edges.append(edge)
    # print(edges)
    return edges


def distance_between_polygon_line(polygon_coords, line_coords):
    from shapely.geometry import LineString, Polygon

    # 定义线段和多边形的坐标
    # line_coords = [(1, 1), (4, 5)]
    # polygon_coords = [(0, 0), (0, 3), (4, 3), (4, 0)]
    # line_coords = np.array(line_coords)
    # polygon_coords = np.array(polygon_coords)

    # 创建LineString和Polygon对象
    line = LineString(line_coords)
    polygon = Polygon(polygon_coords)

    # 计算线段到多边形的距离
    distance = line.distance(polygon)
    return distance


def distance_between_polygon_peer(polygon1, polygon2):
    from shapely.geometry import Polygon

    # 定义两个多边形的坐标
    # polygon1_coords = [(0, 0), (0, 3), (4, 3), (4, 0)]
    # polygon2_coords = [(5, 1), (5, 4), (8, 4), (8, 1)]
    polygon1_coords = polygon1
    polygon2_coords = polygon2
    # print('polygon1_coords=22')
    # print(polygon1_coords)
    # print(polygon2_coords)
    # print(flush=True)

    # 创建两个多边形对象
    polygon1 = Polygon(polygon1_coords)
    polygon2 = Polygon(polygon2_coords)

    # 计算两个多边形之间的距离
    distance = polygon1.distance(polygon2)

    # print(distance)
    return distance


def distance_between_polygon_MultiLine(polygon_coords, line_coords):

    from shapely.geometry import MultiLineString, Polygon

    # # 定义多段连接的折线和多边形坐标
    # line_coords = [((1, 1), (2, 3)), ((2, 3), (4, 5)), ((4, 5), (6, 2))]
    # polygon_coords = [(0, 0), (0, 3), (4, 3), (4, 0)]

    # 创建MultiLineString和Polygon对象
    line = MultiLineString(line_coords)
    polygon = Polygon(polygon_coords)

    # 计算多段连接的折线到多边形的最短距离
    distance = line.distance(polygon)

    # print(distance)
    return distance






def old_distance_between_polygon_peer(polygon1, polygon2):
    if check_polygon_intersection(polygon1, polygon2):
        return 0.0

    dis_list = []
    verts2e_list1 = []
    for edge in edges(polygon2):
        verts2e = polygon_vertices_to_edge_distance(
            polygon1=polygon1,
            polygon2_edge=edge)
        verts2e_list1.append(verts2e)
        dis_list += verts2e

    verts2e_list2 = []
    for edge in edges(polygon1):
        verts2e = polygon_vertices_to_edge_distance(
            polygon1=polygon2,
            polygon2_edge=edge)
        verts2e_list2.append(verts2e)
        dis_list += verts2e
    mindis = min(dis_list)
    # print('mindis')
    # print(mindis)
    return mindis


############################################################
###########        先判断两个线段是否相交       ##############
############################################################
def are_segments_intersecting(segment1, segment2):
    line1 = shapely.geometry.LineString(segment1)
    line2 = shapely.geometry.LineString(segment2)
    return line1.intersects(line2)


# 不要用了，有问题
def segments_intersect(segment1, segment2):
    # print('segment1')
    # print(type(segment1))
    # print(segment1)
    # print(*segment1, sep='\n')
    # print('segment2')
    # print(type(segment2))
    # print(segment2)
    # print(*segment2, sep='\n')
    # print(flush=True)
    # 解构线段1的端点坐标
    x11, y11, x12, y12 = *segment1[0], *segment1[1]
    # 解构线段2的端点坐标
    x21, y21, x22, y22 = *segment2[0], *segment2[1]

    # 向量叉积法
    # 构造向量表示线段1的起点和终点 A_B
    segment1_vector = np.array([x12 - x11, y12 - y11])
    # 构造向量表示线段2的起点和终点 C_D
    segment2_vector = np.array([x22 - x21, y22 - y21])
    # 构造向量表示线段1的起点到线段2的起点的向量 A_C
    start_to_start_vector = np.array([x21 - x11, y21 - y11])

    # 计算叉积
    cross_product_1 = np.cross(segment1_vector, start_to_start_vector)
    cross_product_2 = np.cross(segment1_vector, segment2_vector)

    # print('np.sign(cross_product_1) != np.sign(cross_product_2')
    # print((cross_product_1, cross_product_2))
    # print((np.sign(cross_product_1), np.sign(cross_product_2)))
    # print((np.sign(cross_product_1) != np.sign(cross_product_2)))
    # 判断线段是否相交
    # 如果 cross1 与 cross2 的乘积小于0，
    # 则线段 AB 和线段 CD 相交。否则，它们不相交。
    # if np.sign(cross_product_1) != np.sign(cross_product_2):
    if not np.isclose(np.sign(cross_product_1), np.sign(cross_product_2), atol=1e-3):
        # 计算叉积
        cross_product_3 = np.cross(segment2_vector, start_to_start_vector)
        cross_product_4 = np.cross(segment2_vector, segment1_vector)
        # print('np.sign(cross_product_3) != np.sign(cross_product_4')
        # print((cross_product_1, cross_product_2))
        # print((np.sign(cross_product_3), np.sign(cross_product_4)))
        # print((np.sign(cross_product_3) != np.sign(cross_product_4)))
        # 判断线段是否相交
        # if np.sign(cross_product_3) != np.sign(cross_product_4):
        if not np.isclose(np.sign(cross_product_3), np.sign(cross_product_4), atol=1e-3):
            return True

    return False


def check_polygon_intersection(polygon1, polygon2):
    polygon1 = [v[:2] for v in polygon1]
    polygon2 = [v[:2] for v in polygon2]
    # print('polygon1')
    # print(type(polygon1))
    # print(polygon1)
    # print(*polygon1, sep='\n')
    # print(flush=True)
    # print('polygon2')
    # print(type(polygon2))
    # print(polygon2)
    # print(*polygon2, sep='\n')
    # print(flush=True)

    # 检查多边形1的每条边是否与多边形2的边相交
    for edge1 in edges(polygon1):
        for edge2 in edges(polygon2):
            # 排除相邻的线段两端点重合的情况
            segment1 = edge1
            segment2 = edge2
            # if (segment1[0] == segment2[0] or segment1[0] == segment2[1] or
            #    segment1[1] == segment2[0] or segment1[1] == segment2[1]):
            #     continue
            # if (segment1[0] == segment1[1] or
            #         segment2[0] == segment2[1]):
            # print('edge1')
            # print((edge1))
            # print(edge1)
            # if (segment1[0] == segment1[1] or
            #         segment2[0] == segment2[1]):
            # 0是起点 1是重点
            if (np.all(segment1[0] == segment1[1]) or
                    np.all(segment2[0] == segment2[1])):

                # print('起点终点重合')
                continue
            # if segments_intersect(edge1, edge2):
            if are_segments_intersecting(edge1, edge2):
                # print('相交：', edge1, edge2)
                return True

    return False


############################################################
#################    本文件中 用来测试   ####################
############################################################
def test_distance_between_polygon_peer():
    # openGJK  6.701536790928937
    # 本文     6.7015367900000005
    # polygon1 = [[8.60070679, 2.73776152, 0.],
    #             [10.62070679, 2.73776152, 0.],
    #             [10.62070679, 4.75776152, 0.],
    #             [8.60070679, 4.75776152, 0.]]
    # polygon2 = [[1.89917,  4.1948,   0.], [0.36827046,  7.80459727,  0.]]
    # polygon1 = [
    #     np.array([8.60070679, 2.73776152]),
    #     np.array([10.62070679, 2.73776152]),
    #     np.array([10.62070679, 4.75776152]),
    #     np.array([8.60070679, 4.75776152])
    # ]
    # polygon2 = [
    #     np.array([1.89917, 4.1948]),
    #     np.array([0.36827046, 7.80459727])
    # ]

    # # openGJK  0.6637734748890693
    # # 本文     0.6637734700000006
    # polygon1 = [[10.89348297, 9.1509833,  0.],
    #             [12.91348297, 9.1509833,  0.],
    #             [12.91348297, 11.1709833,  0.],
    #             [10.89348297, 11.1709833,  0.],]
    # polygon2 = [[12.78633229, 8.48720983, 0.],
    #             [12.78633229, 8.48720983, 0.],
    #             [12.78633229, 8.48720983, 0.],]

    # # openGJK  0.0
    # # 本文     0.0
    # polygon1 = [[10.95091558, 15.95190518, 0.],
    #             [55.97091558, 15.95190518, 0.],
    #             [55.97091558, 60.97190518, 0.],
    #             [10.95091558, 60.97190518, 0.],]
    # polygon2 = [[18.5809,    91.2099,     0.],
    #             [4.85874489, 28.54751254, 0.],]

    # # openGJK  0.0
    # # 本文     0.0
    # polygon1 = [[10.95091558, 15.95190518, 0.],
    #             [55.97091558, 15.95190518, 0.],
    #             [55.97091558, 60.97190518, 0.],
    #             [10.95091558, 60.97190518, 0.],]
    # polygon2 = [[18.5809,    91.2099,     0.],
    #             [4.85874489, 28.54751254, 0.],]

    # # openGJK  0.0
    # # 本文     0.0
    # polygon1 = [[10.95091558, 15.95190518, 0.],
    #             [55.97091558, 15.95190518, 0.],
    #             [55.97091558, 60.97190518, 0.],
    #             [10.95091558, 60.97190518, 0.],]
    # polygon2 = [[23.472,     86.0518,     0.],
    #             [47.94912051, 8.34347667, 0.],]

    # # openGJK  0.6637734748890693
    # # 本文     0.6637734700000006
    # polygon1 = [[17.90724387, 19.88291599, 0.],
    #             [19.92724387, 19.88291599, 0.],
    #             [19.92724387, 21.90291599, 0.],
    #             [17.90724387, 21.90291599, 0.],]
    # polygon2 = [[21.17875072, 5.42570174, 0.],
    #             [7.5363646, 14.29302781, 0.],]  # 不要用了，有问题

    # # # openGJK  nan
    # # # 本文     0.05378868034837395
    # polygon1 = [[49.97027693, 6.05563668, 0.],   # 左下
    #             [94.99027693, 6.05563668, 0.],   # 右下
    #             [94.99027693, 51.07563668, 0.],  # 右上
    #             [49.97027693, 51.07563668, 0.],] # 左上
    # polygon2 = [[49.93027693, 51.11563668, 0.],
    #             [81.09726473, 66.9708885,  0.],]  # 线段的y都比正方形的左上高

    # # openGJK  0.0
    # # 本文     0.0
    # polygon1 = [[2.69313319, 29.15979064, 0.],
    #             [47.71313319, 29.15979064, 0.],
    #             [47.71313319, 74.17979064, 0.],
    #             [2.69313319, 74.17979064, 0.],]
    # polygon2 = [[19.21718661, 14.49686076, 0.],
    #             [68.47774185, 49.06638256, 0.],]
    # # openGJK  0.6637734748890693
    # # 本文     0.6637734700000006
    # polygon1 = [[2.69313319, 29.15979064, 0.],
    #             [47.71313319, 29.15979064, 0.],
    #             [47.71313319, 74.17979064, 0.],
    #             [2.69313319, 74.17979064, 0.],]
    # polygon2 = [[18.81121486, 13.24224731, 0.],
    #             [53.58108097, 72.33598104, 0.],]
    # # openGJK  0.6637734748890693
    # # 本文     0.6637734700000006
    # polygon1 = [[1.99823943, 3.06687449], [1.99823943, 1.66687449],
    #             [3.39823943, 3.06687449], [3.39823943, 1.66687449]]
    # polygon2 = [[3.15952956, 1.66687449],
    #             [3.15952956, 1.66687449]]
    # # openGJK  0.6637734748890693
    # # 本文     0.6637734700000006
    # polygon1 = [[3.39823943, 3.06687449], [1.99823943, 3.06687449],
    #             [1.99823943, 1.66687449], [3.39823943, 1.66687449]]
    # polygon2 = [[3.15952956, 1.66687449],
    #             [3.15952956, 1.66687449]]
    # # openGJK  0.6637734748890693
    # # 本文     0.6637734700000006
    # polygon1 = [[3.61386895, 1.30195649], [2.41386895, 1.30195649],
    #             [2.41386895, 0.10195649], [3.61386895, 0.10195649]]
    # polygon2 = [[3.71386895, 0.95643772], [3.71386895, 0.95643772]]

    # # openGJK
    # # 本文     相交
    # polygon1 = [
    #     [25.63778480849162, 25.164460736606728],
    #     [81.61004055900824, 25.164460736606728],
    #     [81.61004055900824, 81.13671648712335],
    #     [25.63778480849162, 81.13671648712335],
    # ]
    # polygon2 = [
    #     [82.21004055900823, 24.564460736606726],
    #     [24.037587513551955, 25.08215580121789],
    #     [24.037587513551955, 25.08215580121789],
    #     [23.997886225095453, 25.102705008846083],
    #     [23.914980581599057, 25.145616566815093],
    #     [23.788870498020355, 25.210890310786553],
    #     [23.619555960514315, 25.298526213963417],
    #     [23.407037015000412, 25.40852436498673],
    #     [23.151313614822712, 25.540884673559802],
    # ]

    # # openGJK
    # # 本文
    # polygon1 = [
    #     [43.863889765000934, 31.889715805623787],
    #     [99.83614551551756, 31.889715805623787],
    #     [99.83614551551756, 87.8619715561404],
    #     [43.863889765000934, 87.8619715561404],
    # ]
    # polygon2 = [[44.92612022, 30.88596314],
    #             [44.70513711, 31.13985539],
    #             [44.51615804, 31.35710841],
    #             [44.35918299, 31.5377222,],
    #             [44.23421201, 31.68169678],
    #             [44.14124514, 31.78903222],
    #             [44.08028247, 31.85972857],
    #             [44.05128751, 31.89375437],
    #             [44.05128751, 31.89375437],]

    # # openGJK
    # # 本文
    polygon1 = [
        [43.863889765000934, 31.889715805623787],
        [99.83614551551756, 31.889715805623787],
        [99.83614551551756, 87.8619715561404],
        [43.863889765000934, 87.8619715561404],
    ]
    polygon2 = [
        [50,50],
        [50,50],
        [50,50],
    ]

    # # openGJK
    # # 本文
    # polygon1 =
    # polygon2 =

    plist1 = [v[:2] for v in polygon1]
    plist2 = [v[:2] for v in polygon2]
    plist = [plist1, plist2]
    # obstacle_corridor.get_segment_list(plist1, plist2)

    if check_polygon_intersection(polygon1, polygon2):
        print("多边形相交")
        pass
    else:
        print("多边形不相交")
    dis = distance_between_polygon_peer(polygon1, polygon2)
    print('dis')
    print(dis)

    # 递归的方式来比较大小
    def find_max(lst): return max(
        find_max(x) if isinstance(x, list) else x
        for x in lst
    )
    plotPoly([polygon1, polygon2], x=-1, y=find_max(plist))
    # plotPoly(plist, x=0, y=100)


def test_get_segment_list():
    pass
######################################################################
######################################################################
######################################################################
######################################################################


def check_point_on_line(point, line):
    # 创建Point和LineString对象
    point = Point(point)
    line = LineString(line)

    # 判断点是否在线段上
    if line.contains(point):
        return True
    else:
        return False


def test_check_point_on_line():
    # 示例点和线段
    # point = (1, 1)
    # line = [(0, 0), (2, 2)]
    # polygon1 = [[3.39823943, 3.06687449], [1.99823943, 3.06687449],
    #             [1.99823943, 1.66687449], [3.39823943, 1.66687449]]
    # polygon2 = [[3.15952956, 1.66687449],
    #             [3.15952956, 1.66687449]]
    polygon1 = [[3.61386895, 1.30195649], [2.41386895, 1.30195649],
                [2.41386895, 0.10195649], [3.61386895, 0.10195649]]
    polygon2 = [[3.71386895, 0.95643772], [3.71386895, 0.95643772]]
    polygon2 = [
        [3.71386895, 0.95643772], 
        [3.71386895, 0.95643772],
        [3.71386895, 0.95643772],
        ]

    point = polygon2[0]
    # line = polygon1

    # 调用函数进行判断
    for i, edge in enumerate(edges(polygon1)):
        is_on_line = check_point_on_line(point, edge)

        if is_on_line:
            print("点在线段上")
        else:
            print("点不在线段上")


def test_distance_between_polygon_MultiLine():
    # openGJK  0.6637734748890693
    # 本文     0.6637734700000006
    polygon1 = [[3.61386895, 1.30195649], [2.41386895, 1.30195649],
                [2.41386895, 0.10195649], [3.61386895, 0.10195649]]
    polygon2 = [
        [3.71386895, 0.95643772], 
        [3.71386895, 0.95643772]
    ]

    polygon_coords = polygon1 
    line_coords = polygon2
    nline=[]
    for i in range(len(line_coords)-1):
        nline.append([line_coords[i],line_coords[i+1]])
    line_coords=nline
    # line_coords = [((1, 1), (2, 3)), ((2, 3), (4, 5)), ((4, 5), (6, 2))]
    # polygon_coords = [(0, 0), (0, 3), (4, 3), (4, 0)]
    print('polygon_coords, line_coords')
    print(polygon_coords)
    print(line_coords)
    dis = distance_between_polygon_MultiLine(polygon_coords, line_coords)
    print(dis)
    # 递归的方式来比较大小
    def find_max(lst): return max(
        find_max(x) if isinstance(x, list) else x
        for x in lst
    )
    pl=[polygon1, polygon2]
    # pl=[line_coords, polygon_coords]

    plotPoly(pl, x=-1, y=find_max(pl))
    
def test_distance_between_polygon_line():
    line_coords = [((1, 1), (2, 3)), ((2, 3), (4, 5)), ((4, 5), (6, 2))]
    polygon_coords = [(0, 0), (0, 3), (4, 3), (4, 0)]
    d=distance_between_polygon_line(polygon_coords, line_coords)
    print('d',d)
    def find_max(lst): return max(
        find_max(x) if isinstance(x, list) else x
        for x in lst
    )
    pl=[line_coords, polygon_coords]
    plotPoly(pl, x=-1, y=find_max(pl))

if __name__ == '__main__':
    # test_check_point_on_line()
    # test_distance_between_polygon_peer()
    test_distance_between_polygon_MultiLine()
    # test_distance_between_polygon_line()