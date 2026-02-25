

# https://stackoverflow.com/questions/55460133/distance-to-convex-hull-from-point-in-3d-in-python


# pip install PyGEL3D
import numpy as np
from scipy.spatial import ConvexHull
from pygel3d import hmesh

mat = np.random.rand(100, 3)
mat = np.array(
    [
        [0,0,0],
        [1,0,0],
        [0,1,0],
        [1,1,0],
        [0,0,1],
        [1,0,1],
        [0,1,1],
        [1,1,1],
    ]
)
hull = ConvexHull(mat)
points = np.random.rand(10, 3)
points = np.array(
    [
        [0.1,0.1,0.1],
        [1,1,1],
        [2,2,2],
    ]
)

def dist(hull, points):
    # Construct PyGEL Manifold from the convex hull
    m = hmesh.Manifold()
    for s in hull.simplices:
        m.add_face(hull.points[s])

    dist = hmesh.MeshDistance(m)
    res = []
    for p in points:
        # Get the distance to the point
        # But don't trust its sign, because of possible
        # wrong orientation of mesh face
        d = dist.signed_distance(p)

        # Correct the sign with ray inside test
        if dist.ray_inside_test(p):
            if d > 0:
                d *= -1
        else:
            if d < 0:
                d *= -1
        res.append(d)
    return np.array(res)

print(dist(hull, points))


# ##################################################################
# import numpy as np
# from scipy.spatial import ConvexHull
# # from PyGEL3D import gel
# from pygel3d import gel
# # import PyGEL3D
# import pygel3d

# mat = np.random.rand(100, 3)
# hull = ConvexHull(mat)
# points = np.random.rand(10, 3)

# def dist(hull, points):
#     # Construct PyGEL Manifold from the convex hull
#     m = gel.Manifold()
#     for s in hull.simplices:
#         m.add_face(hull.points[s])

#     dist = gel.MeshDistance(m)
#     res = []
#     for p in points:
#         # Get the distance to the point
#         # But don't trust its sign, because of possible
#         # wrong orientation of mesh face
#         d = dist.signed_distance(p)

#         # Correct the sign with ray inside test
#         if dist.ray_inside_test(p):
#             if d > 0:
#                 d *= -1
#         else:
#             if d < 0:
#                 d *= -1
#         res.append(d)
#     return np.array(res)

# print(dist(hull, points))
# ##################################################################





# ##################################################################
# import numpy as np
# from scipy.spatial import ConvexHull

# # 定义两个多面体（或凸包）的顶点坐标
# points1 = np.array([[0, 0, 0], [1, 0, 0], [0, 1, 0], [0, 0, 1]])
# points2 = np.array([[1, 1, 1], [2, 1, 1], [1, 2, 1], [1, 1, 2]])

# # 计算两个多面体的凸包
# convex_hull1 = ConvexHull(points1)
# convex_hull2 = ConvexHull(points2)

# # 获取凸包的顶点和边
# vertices1 = points1[convex_hull1.vertices]
# vertices2 = points2[convex_hull2.vertices]
# edges1 = convex_hull1.simplices
# edges2 = convex_hull2.simplices

# # 计算点到面的距离
# def point_to_plane_distance(point, plane_vertices):
#     p1, p2, p3 = plane_vertices
#     normal = np.cross(p2 - p1, p3 - p1)
#     d = -np.dot(normal, p1)
#     distance = (np.dot(normal, point) + d) / np.linalg.norm(normal)
#     return distance

# # 计算两个凸包之间的最短距离
# def minimum_distance(convex_hull1, convex_hull2):
#     min_distance = np.inf
#     for vertex1 in convex_hull1.points:
#         distance_to_plane = point_to_plane_distance(vertex1, vertices2)
#         if distance_to_plane < min_distance:
#             min_distance = distance_to_plane
#     for vertex2 in convex_hull2.points:
#         distance_to_plane = point_to_plane_distance(vertex2, vertices1)
#         if distance_to_plane < min_distance:
#             min_distance = distance_to_plane
#     for edge1 in edges1:
#         for edge2 in edges2:
#             distance = np.inf
#             for i in range(2):
#                 for j in range(2):
#                     dist = np.linalg.norm(vertices1[edge1[i]] - vertices2[edge2[j]])
#                     if dist < distance:
#                         distance = dist
#             if distance < min_distance:
#                 min_distance = distance
#     return min_distance

# # 计算两个凸包之间的最短距离
# min_dist = minimum_distance(convex_hull1, convex_hull2)

# print("两个凸包之间的最短距离为:", min_dist)
# ##################################################################


# https://blog.csdn.net/qwerpoiu66/article/details/131370634?spm=1001.2101.3001.6650.2&utm_medium=distribute.pc_relevant.none-task-blog-2%7Edefault%7EYuanLiJiHua%7EPosition-2-131370634-blog-23997293.235%5Ev38%5Epc_relevant_sort_base1&depth_1-utm_source=distribute.pc_relevant.none-task-blog-2%7Edefault%7EYuanLiJiHua%7EPosition-2-131370634-blog-23997293.235%5Ev38%5Epc_relevant_sort_base1&utm_relevant_index=5
# def GJK(s1,s2)
# #两个形状s1,s2相交则返回True。所有的向量/点都是三维的，例如（[x,y,0]）
# #第一步：选择一个初始方向，这个初始方向可以是随机选择的，但通常来说是两个形状中心之间的向量，即：
#     d= normalize(s2.center-s1.center)
# #第二步：找到支撑点，即第一个支撑点
#     simplex=[support(s1,s2,d)]
# #第三步：找到第一个支撑点后，以第一个支撑点为起点指向原点O的方向为新方向d
#      d=ORIGIN-simplex[0]
# #第四步：开始循环，找下一个支撑点
#     while True
#         A=[support(s1,s2,d)]
# #当新的支撑点A没有经过原点，那我们就返回False，即两个形状没有相交
#         if dot(A,d) <0:
#             return False
# #否则，我们就将该点A加入到simplex中
#         simplex.append(A)
# #handleSimplex负责主要逻辑部分。主要负责处理寻找新方向和更新simplex的逻辑内容,当当前simplex包含原点，则返回Ture
#         if handleSimplex(simplex,d):
#             return Ture
 
# def handleSimplex(simplex,d)
# #如果当前的simplex为直线情况，则进入lineCase(simplex,d)函数,寻找下一个方向d,并返回False，即直线情况下的simplex不包含原点
#     if len(simplex==2):
#         return lineCase(simplex,d)
# #如果当前的simplex为三角情况，则进入triangleCase(simplex,d,
#     return triangleCase(simplex,d)
 
# def  lineCase(simplex,d)
# #构建向量AB与AO,并使用三重积得到下一个方向
#     B,A = simplex
#     AB,AO=B-A,ORIGIN-A
#     ABprep= tripleProd(AB,AO,AB)
#     d.set(ABprep)
# #由于一条直线的情况下，原点不能包含在simplex中，所以返回False
#     return False
 
# def triangleCase(simplex,d)
# #构建向量AB,AC与AO,并来检测原点在空间的哪个区域。
#     C,B,A = simplex
#     AB,AC,AO=B-A,C-A,ORIGIN-A
# #通过三重积分别得到垂直于AB、AC的向量，检测区域Rab、Rac中是否包含原点。
#     ABprep= tripleProd(AC,AB,AB)
#     ACprep= tripleProd(AB,AC,AC)
# #如果原点在AB区域中，我们移除点C以寻找更加完美的simplex，新的方向就是垂直于AB的向量
#     if dot(ABprep,AO)>0:
#        simplex.remove(C);d.set(ABprep) 
#        return False
# #如果原点在AC区域中，我们移除点B以寻找更加完美的simplex，新的方向就是垂直于AC的向量
#     elif dot(ACprep,AO)>0:
#        simplex.remove(Ba);d.set(ACprep) 
#        return False
# #如果这两种情况都不符合，那就说明当前的三角形中包含原点，两个形状相交
#     return Ture
 
# def support(s1,s2,d)
# #取第一个形状上方向d上最远点并减去第二个形状上相反反向（-d）上最远的点
#     return s1.furthestPoint(d)-s2.furthestPoint(-d)

# exit()

# ##################################################################
# import numpy as np
# from scipy.spatial import ConvexHull

# def compute_convex_hull_distance(convex_hull1, convex_hull2):
#     # 计算两个凸包之间的最短距离
#     min_distance = float("inf")
#     for point1 in convex_hull1:
#         for point2 in convex_hull2:
#             distance = np.linalg.norm(point1 - point2)
#             if distance < min_distance:
#                 min_distance = distance
#     return min_distance


# def compute_shortest_distance(convex_hull_points1, convex_hull_points2):
#     # 使用Scipy库计算凸包
#     convex_hull1 = ConvexHull(convex_hull_points1).points
#     convex_hull2 = ConvexHull(convex_hull_points2).points

#     distance = None
#     if len(convex_hull1) > 0 and len(convex_hull2) > 0:
#         distance = compute_convex_hull_distance(convex_hull1, convex_hull2)

#     return distance


# # 示例用法
# convex_hull_points1 = np.array([[1, 1, 1], [2, 2, 2], [3, 3, 3], [4, 4, 4]])
# convex_hull_points2 = np.array([[5, 5, 5], [6, 6, 6], [7, 7, 7], [8, 8, 8]])

# shortest_distance = compute_shortest_distance(convex_hull_points1, convex_hull_points2)
# print("最短距离:", shortest_distance)
# ##################################################################


# import pyhull.convex_hull as ch
# a=1

# vertices_cube = [
#     [0, 0, 0],
#     [a, 0, 0],
#     [0, a, 0],
#     [a, a, 0],
#     [0, 0, a],
#     [a, 0, a],
#     [0, a, a],
#     [a, a, a]
# ]



# convex_hull_cube = ch.ConvexHull(vertices_cube)
# # vertices_segment = [[x1, y1, z1], [x2, y2, z2]]
# vertices_segment = [[2,2,2], [3,3,3]]
# distance = convex_hull_cube.distance_to(vertices_segment)
########################################################################################

# import math

# def shortest_distance(p1, p2, q1, q2):
#     # 计算向量AB和CD的长度的平方
#     ab_squared = (p2[0] - p1[0])**2 + (p2[1] - p1[1])**2 + (p2[2] - p1[2])**2
#     cd_squared = (q2[0] - q1[0])**2 + (q2[1] - q1[1])**2 + (q2[2] - q1[2])**2
    
#     # 计算向量AC在向量AB上的投影长度
#     ac_dot_ab = (p1[0] - q1[0]) * (p2[0] - p1[0]) + (p1[1] - q1[1]) * (p2[1] - p1[1]) + (p1[2] - q1[2]) * (p2[2] - p1[2])
#     projection_length = ac_dot_ab / ab_squared
    
#     # 判断最短距离的情况并计算
#     if projection_length < 0:
#         shortest_dist = math.sqrt((p1[0] - q1[0])**2 + (p1[1] - q1[1])**2 + (p1[2] - q1[2])**2)
#     elif projection_length > 1:
#         shortest_dist = math.sqrt((p1[0] - q2[0])**2 + (p1[1] - q2[1])**2 + (p1[2] - q2[2])**2)
#     else:
#         perpendicular_length_squared = (p1[0] - q1[0] - projection_length * (p2[0] - p1[0]))**2 + \
#                                        (p1[1] - q1[1] - projection_length * (p2[1] - p1[1]))**2 + \
#                                        (p1[2] - q1[2] - projection_length * (p2[2] - p1[2]))**2
#         shortest_dist = math.sqrt(perpendicular_length_squared)
    
#     return shortest_dist

# # 示例点坐标
# # point_p1 = (1, 2, 3)
# # point_p2 = (4, 5, 6)
# # point_q1 = (7, 8, 9)
# # point_q2 = (10, 11, 12)

# a4=[
#     (0, -1, 0),
#     (0, 1, 0),
#     (-1, 0, 1),
#     (1, 0, 1),
#     # (0, -1, 0),
#     # (0, 1, 0),
#     # (0, -1, 1),
#     # (0, 1, 1),
# ]

# # 计算最短距离
# distance = shortest_distance(*a4)
# # distance = shortest_distance(point_p1, point_p2, point_q1, point_q2)
# print(distance)

# exit()
########################################################################################



# import cvxopt

# def distance_to_cube(segment_start, segment_end, cube_min, cube_max):
#     # 构造二次规划问题
#     P = cvxopt.matrix([[1.0, 0.0, 0.0],
#                        [0.0, 1.0, 0.0],
#                        [0.0, 0.0, 1.0]])  # 优化目标函数的二次项系数矩阵

#     q = cvxopt.matrix([-2 * segment_start[0], -2 * segment_start[1], -2 * segment_start[2]])  # 优化目标函数的一次项系数矩阵

#     G = cvxopt.matrix([[1.0, 0.0, 0.0, 0.0, 0.0, 0.0],
#                        [-1.0, 0.0, 0.0, 0.0, 0.0, 0.0],
#                        [0.0, 1.0, 0.0, 0.0, 0.0, 0.0],
#                        [0.0, -1.0, 0.0, 0.0, 0.0, 0.0],
#                        [0.0, 0.0, 1.0, 0.0, 0.0, 0.0],
#                        [0.0, 0.0, -1.0, 0.0, 0.0, 0.0],
#                        [0.0, 0.0, 0.0, 1.0, 0.0, 0.0],
#                        [0.0, 0.0, 0.0, -1.0, 0.0, 0.0],
#                        [0.0, 0.0, 0.0, 0.0, 1.0, 0.0],
#                        [0.0, 0.0, 0.0, 0.0, -1.0, 0.0],
#                        [0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
#                        [0.0, 0.0, 0.0, 0.0, 0.0, -1.0]])  # 不等式约束矩阵

#     h = cvxopt.matrix([cube_max[0], -cube_min[0], cube_max[1], -cube_min[1], cube_max[2], -cube_min[2],
#                        -segment_start[0], segment_end[0],
#                        -segment_start[1], segment_end[1],
#                        -segment_start[2], segment_end[2]])  # 不等式约束的右侧项矩阵

#     A = cvxopt.matrix([[0.0, 0.0, 0.0]])  # 等式约束矩阵
#     b = cvxopt.matrix([1.0])  # 等式约束的右侧项矩阵

#     # 求解
#     sol = cvxopt.solvers.qp(P, q, G, h, A, b)

#     # 提取最优解
#     distance = sol['primal objective']
#     return distance

# # 示例测试
# segment_start = [1.0, 2.0, 3.0]
# segment_end = [4.0, 5.0, 6.0]
# cube_min = [2.0, 3.0, 4.0]
# cube_max = [5.0, 6.0, 7.0]
# arg4=[
#     (2,0,0),
#     (0,2,0),
#     (0,0,0),
#     (1,1,1),
#     ]
# distance = distance_to_cube(*arg4)
# # distance = distance_to_cube(segment_start, segment_end, cube_min, cube_max)
# print('最短距离为：', distance)
########################################################################################



import math

def distance_segment_to_cube(segment, cube):
    # 计算线段的两个端点
    p1, p2 = segment

    # 将立方体的顶点坐标表示为列表
    vertices = [
        # x 0 3
        # y 1 4
        # z 2 5
        [cube[0], cube[1], cube[2]],  # V0
        [cube[3], cube[1], cube[2]],  # V1
        [cube[3], cube[4], cube[2]],  # V2
        [cube[0], cube[4], cube[2]],  # V3
        [cube[0], cube[1], cube[5]],  # V4
        [cube[3], cube[1], cube[5]],  # V5
        [cube[3], cube[4], cube[5]],  # V6
        [cube[0], cube[4], cube[5]]   # V7
    ]

    # 定义立方体的12条边，每条边由两个顶点索引构成
    edges_indices = [
        (0, 1), (1, 2), (2, 3), (3, 0),  # 底面四条边
        (4, 5), (5, 6), (6, 7), (7, 4),  # 顶面四条边
        (0, 4), (1, 5), (2, 6), (3, 7)   # 连接底面和顶面的四条边
    ]

    # 根据顶点索引获取边的坐标
    edges = []
    for edge in edges_indices:
        start_vertex = vertices[edge[0]]
        end_vertex = vertices[edge[1]]
        edges.append((start_vertex, end_vertex))



    # 初始化最小距离为正无穷大
    min_distance = math.inf


    # 遍历立方体的每条边
    # for i in range(8):
    for edge in edges:
        # v1 = vertices[i]
        # v2 = vertices[(i + 1) % 8]
        v1,v2=edge
        # 计算线段与当前边之间的距离
        distance = distance_segment_to_line(p1, p2, v1, v2)

        # 更新最小距离
        min_distance = min(min_distance, distance)

    return min_distance

def distance_segment_to_line(p1, p2, v1, v2):
    # 计算线段的方向向量
    d = [p2[0] - p1[0], p2[1] - p1[1], p2[2] - p1[2]]

    # 计算边的方向向量
    e = [v2[0] - v1[0], v2[1] - v1[1], v2[2] - v1[2]]

    # 计算边的长度平方
    e_length_square = e[0] ** 2 + e[1] ** 2 + e[2] ** 2

    # 计算线段到边的垂直投影点
    t = (
        (d[0] * (v1[0] - p1[0]) + d[1] * (v1[1] - p1[1]) + d[2] * (v1[2] - p1[2]))
        /
        (d[0] ** 2 + d[1] ** 2 + d[2] ** 2)
    )

    # 如果投影点在边的延长线上
    if t < 0:
        closest_point = v1
    elif t > 1:
        closest_point = v2
    else:
        # 计算线段到边的最近点
        closest_point = [
            p1[0] + t * d[0],
            p1[1] + t * d[1],
            p1[2] + t * d[2]
        ]

    # 计算线段到最近点的距离
    distance = math.sqrt(
        (closest_point[0] - p1[0]) ** 2
        + (closest_point[1] - p1[1]) ** 2
        + (closest_point[2] - p1[2]) ** 2
    )

    return distance

# 示例测试
segment = [[1, 2, 3], [4, 5, 6]]
segment = [[2, 2, 2], [3, 3, 3]]
segment = [[2, 2, 2], [1, 1, 1]]
segment = [[2, 2, 2], [0, 0, 0]]
segment = [[0, 0, 0], [1, 1, 1]]
segment = [[1, 0, 0], [0, 1, 0]]
segment = [[2, 0, 0], [0, 2, 0]]
# cube = [0, 1, 2, 3, 4, 5]
cube = [0, 0, 0, 1, 1, 1]
distance = distance_segment_to_cube(segment, cube)
print(distance)
distance = distance_segment_to_cube([segment[1],segment[0]], cube)
print(distance)


exit()


import numpy as np

def distance_to_square(segment, square_center, square_size):
    # 将线段端点坐标分别记为P和Q
    P = np.array(segment[0])
    Q = np.array(segment[1])

    # 计算从点P到平面的距离
    # 平面上一点A为正方形中心位置，法向量为(0, 0, 1)
    A = np.array(square_center)
    normal = np.array([0, 0, 1])
    distance_P = np.abs(np.dot(P - A, normal))

    # 计算从点Q到平面的距离
    distance_Q = np.abs(np.dot(Q - A, normal))

    # 判断线段是否与平面相交
    if distance_P == 0 or distance_Q == 0:
        return 0.0

    # 如果线段两个端点都在平面同侧，则直接返回两点之间的距离
    if (distance_P > 0 and distance_Q > 0) or (distance_P < 0 and distance_Q < 0):
        return np.linalg.norm(Q - P)

    # 计算A到线段的垂足坐标H
    t = np.dot(A - P, Q - P) / np.dot(Q - P, Q - P)
    H = P + t * (Q - P)

    # 如果线段与平面相交，则返回AH的距离
    return np.linalg.norm(H - A)


# 测试代码
segment = [(1, 2, 3), (4, 5, 6)]  # 线段的两个端点坐标
square_center = (0, 0, 0)  # 正方形的中心位置坐标
square_size = 1  # 正方形的边长

distance = distance_to_square(segment, square_center, square_size)
print("线段到正方形平面的最短距离为:", distance)


exit()

import numpy as np

def distance_point_to_plane(point, plane_normal, plane_point):
    """
    计算点到平面的距离
    point: 点的坐标，形如 [x, y, z]
    plane_normal: 平面的法向量，形如 [x, y, z]
    plane_point: 平面上的一点，形如 [x, y, z]
    """
    point = np.array(point)
    plane_normal = np.array(plane_normal)
    plane_point = np.array(plane_point)

    # 计算点到平面的距离
    distance = np.abs(np.dot(point - plane_point, plane_normal)) / np.linalg.norm(plane_normal)
    return distance

def distance_segment_to_cube(segment_start, segment_end, cube_min, cube_max):
    """
    计算三维线段到立方体的距离
    segment_start: 线段起点坐标，形如 [x, y, z]
    segment_end: 线段终点坐标，形如 [x, y, z]
    cube_min: 立方体最小顶点坐标，形如 [x, y, z]
    cube_max: 立方体最大顶点坐标，形如 [x, y, z]
    """
    segment_start = np.array(segment_start)
    segment_end = np.array(segment_end)
    cube_min = np.array(cube_min)
    cube_max = np.array(cube_max)

    # 判断线段是否与立方体相交
    if np.any(segment_start < cube_min) and np.any(segment_end < cube_min):
        # 线段完全在立方体的外部
        min_distance = np.linalg.norm(segment_end - segment_start)
        return min_distance

    if np.any(segment_start > cube_max) and np.any(segment_end > cube_max):
        # 线段完全在立方体的外部
        min_distance = np.linalg.norm(segment_end - segment_start)
        return min_distance

    # 计算线段与立方体的距离
    min_distance = np.inf
    
    # 计算与立方体的六个面的距离
    faces = [(np.array([1, 0, 0]), np.array([cube_min[0], 0, 0])),  # x = cube_min[0]
             (np.array([-1, 0, 0]), np.array([cube_max[0], 0, 0])),  # x = cube_max[0]
             (np.array([0, 1, 0]), np.array([0, cube_min[1], 0])),  # y = cube_min[1]
             (np.array([0, -1, 0]), np.array([0, cube_max[1], 0])),  # y = cube_max[1]
             (np.array([0, 0, 1]), np.array([0, 0, cube_min[2]])),  # z = cube_min[2]
             (np.array([0, 0, -1]), np.array([0, 0, cube_max[2]]))]  # z = cube_max[2]

    for face_normal, face_point in faces:
        # 计算线段与面的距离
        distance = distance_point_to_plane(segment_start, face_normal, face_point)
        min_distance = min(min_distance, distance)

    return min_distance

# 示例用法
# segment_start = [1, 2, 3]
# segment_end = [4, 5, 6]
# cube_min = [2, 3, 4]
# cube_max = [5, 6, 7]

# distance = distance_segment_to_cube(segment_start, segment_end, cube_min, cube_max)
arg4=[
    (2,0,0),
    (0,2,0),
    (0,0,0),
    (1,1,1),
    ]
distance = distance_segment_to_cube(*arg4)
print("距离：", distance)

exit()


import numpy as np

def point_to_plane_dist(point, plane_normal, plane_point):
    """
    计算点到平面的距离
    """
    return abs(np.dot(plane_normal, point - plane_point)) / np.linalg.norm(plane_normal)

def line_to_plane_dist(line_start, line_end, plane_normal, plane_point):
    """
    计算线段到平面的距离
    """
    line_dir = line_end - line_start
    t = np.dot(plane_normal, plane_point - line_start) / np.dot(plane_normal, line_dir)
    if t >= 0 and t <= 1:
        intersection = line_start + t * line_dir
        if intersection[0] >= plane_point[0] and intersection[0] <= plane_point[0] + Lx and \
           intersection[1] >= plane_point[1] and intersection[1] <= plane_point[1] + Ly and \
           intersection[2] >= plane_point[2] and intersection[2] <= plane_point[2] + Lz:
            # intersection is inside the rectangle
            return 0
    # line segment does not intersect the rectangle
    plane_distances = []
    for i in range(3):
        normal = np.zeros(3)
        normal[i] = 1
        point = np.zeros(3)
        point[i] = plane_point[i]
        plane_distances.append(point_to_plane_dist(line_start, normal, point))
        plane_distances.append(point_to_plane_dist(line_end, normal, point))
    return min(plane_distances)

# Example usage
P1 = np.array([0,0,0])
P2 = np.array([1,1,1])
C = np.array([2,2,2])
# P1 = np.array([0,0,0])
# P2 = np.array([1,1,1])
# C = np.array([0.5,0.5,0.5])
Lx = Ly = Lz = 1
dist = line_to_plane_dist(P1, P2, np.array([1,0,0]), C - np.array([Lx/2,0,0]))  # distance to the left plane
dist = min(dist, line_to_plane_dist(P1, P2, np.array([-1,0,0]), C + np.array([Lx/2,0,0])))  # distance to the right plane
dist = min(dist, line_to_plane_dist(P1, P2, np.array([0,1,0]), C - np.array([0,Ly/2,0])))  # distance to the bottom plane
dist = min(dist, line_to_plane_dist(P1, P2, np.array([0,-1,0]), C + np.array([0,Ly/2,0])))  # distance to the top plane
dist = min(dist, line_to_plane_dist(P1, P2, np.array([0,0,1]), C - np.array([0,0,Lz/2])))  # distance to the front plane
dist = min(dist, line_to_plane_dist(P1, P2, np.array([0,0,-1]), C + np.array([0,0,Lz/2])))  # distance to the back plane
print(dist)


exit()


import numpy as np
from scipy.spatial.distance import cdist

# 定义长方体的八个顶点坐标
cube_vertices = np.array([
    [0, 0, 0],
    [1, 0, 0],
    [1, 1, 0],
    [0, 1, 0],
    [0, 0, 1],
    [1, 0, 1],
    [1, 1, 1],
    [0, 1, 1]
])

# 定义线段的两个端点坐标
line_start = np.array([0.5, 0.5, 0.5])
line_end = np.array([1.5, 1.5, 1.5])

# 计算线段上每个点到长方体顶点的距离
distances = cdist(np.vstack((line_start, line_end)), cube_vertices)
print('distances')
print(distances)
# 找到距离最小的顶点
nearest_vertex_index = np.argmin(distances)

# 输出最近的点坐标
nearest_vertex = cube_vertices[nearest_vertex_index]
print(nearest_vertex)

exit()


from shapely.geometry import Point, LineString, box
from shapely.ops import nearest_points

# 创建长方体
cuboid = box(0, 0, 0, 2, 2, 3)

# 创建线段
line = LineString([(1, -1, 1), (1, 1, 2)])

# 使用nearest_points函数找到最近的点
nearest_point_cuboid, nearest_point_line = nearest_points(cuboid, line)

print("长方体中最近的点坐标：", nearest_point_cuboid)
print("线段中最近的点坐标：", nearest_point_line)

exit()

from euclid import *

def distance_line_to_box(line_start, line_end, box_min, box_max):
    # 创建直线对象
    line = Line3(Point3(*line_start), Point3(*line_end))

    # 在长方体上找到离直线最近的点
    nearest_point = line.closest_point_on_line_to_point(Point3(*box_min), Point3(*box_max))

    # 计算最近点与长方体的距离
    distance = nearest_point.distance_to(Point3(*box_min), Point3(*box_max))

    return distance

# 示例使用
line_start = [1, 1, 1]
line_end = [4, 4, 4]
box_min = [2, 2, 2]
box_max = [6, 6, 6]

min_distance = distance_line_to_box(line_start, line_end, box_min, box_max)
print(f"最短距离: {min_distance}")

exit()


import numpy as np

def distance_segment_to_box(segment_start, segment_end, box_min, box_max):
    # 将线段转换为向量表示
    segment_vector = np.array(segment_end) - np.array(segment_start)

    # 将长方体边界转换为向量表示
    box_min = np.array(box_min)
    box_max = np.array(box_max)

    # 线段中心点和长方体中心点之间的向量差
    center_diff = (box_max + box_min) / 2.0 - (segment_start + segment_end) / 2.0

    # 长方体边界的一半大小的向量表示
    box_half_size = (box_max - box_min) / 2.0

    # 计算线段投影在长方体上的距离
    projection_distance = np.abs(np.dot(center_diff, segment_vector)) / np.linalg.norm(segment_vector)

    # 计算线段到长方体每个维度的最短距离
    min_distance = np.max(np.abs(center_diff) - box_half_size) + projection_distance

    return min_distance

# 示例使用
segment_start = [1, 1, 1]
segment_end = [4, 4, 4]
box_min = [2, 2, 2]
box_max = [6, 6, 6]

min_distance = distance_segment_to_box(segment_start, segment_end, box_min, box_max)
print(f"最短距离: {min_distance}")

exit()

import numpy as np
from scipy.spatial import distance

# 定义多面体的顶点坐标
vertices = np.array([[0, 0, 0], [1, 0, 0], [0, 1, 0], [0, 0, 1]])

# 定义点的坐标
point = np.array([1, 1, 1])
point2 = np.array([1, 1, 0])

# 使用cdist函数计算点到多面体的距离
# distances = distance.cdist([point], vertices)
distances = distance.cdist([point,point2], vertices)

# 输出距离
print(distances)

exit()








import numpy as np

from shapely.geometry import Point, Polygon

# 定义多边形的坐标
polygon_coords = [(0, 0), (0, 3), (4, 3), (4, 0)]

# 创建多边形对象
polygon = Polygon(polygon_coords)

# 创建点对象
point = Point(1, 2)

# 计算点到多边形的距离
distance = point.distance(polygon)

print(distance)


from shapely.geometry import Polygon

# 定义两个多边形的坐标
polygon1_coords = [(0, 0), (0, 3), (4, 3), (4, 0)]
polygon2_coords = [(5, 1), (5, 4), (8, 4), (8, 1)]

# 创建两个多边形对象
polygon1 = Polygon(polygon1_coords)
polygon2 = Polygon(polygon2_coords)

# 计算两个多边形之间的距离
distance = polygon1.distance(polygon2)

print(distance)


from shapely.geometry import LineString, Polygon

# 定义线段和多边形的坐标
line_coords = [(1, 1), (4, 5)]
polygon_coords = [(0, 0), (0, 3), (4, 3), (4, 0)]
line_coords=np.array(line_coords)
polygon_coords=np.array(polygon_coords)
# 创建LineString和Polygon对象
line = LineString(line_coords)
polygon = Polygon(polygon_coords)

# 计算线段到多边形的距离
distance = line.distance(polygon)

print(distance)



from shapely.geometry import MultiLineString, Polygon

# 定义多段连接的折线和多边形坐标
line_coords = [((1, 1), (2, 3)), ((2, 3), (4, 5)), ((4, 5), (6, 2))]
polygon_coords = [(0, 0), (0, 3), (4, 3), (4, 0)]

# 创建MultiLineString和Polygon对象
line = MultiLineString(line_coords)
polygon = Polygon(polygon_coords)

# 计算多段连接的折线到多边形的最短距离
distance = line.distance(polygon)

print(distance)




######################################################



import numpy as np
from scipy.spatial import ConvexHull
from scipy.spatial.distance import cdist

# 定义多面体的顶点
vertices = np.array([[0, 0, 0], [1, 0, 0], [0, 1, 0], [0, 0, 1]])

# 创建凸包
hull = ConvexHull(vertices)

# 定义一个点
point = np.array([0.5, 0.5, 0.5])

# 计算点到凸包的距离
distance = cdist([point], vertices[hull.vertices]).min()

print("点到多面体的距离:", distance)








######################################################


from shapely.geometry import Point, MultiPolygon

# 定义点和多面体坐标
point_coords = (2, 2)
multipolygon_coords = [
    [[(0, 0), (0, 3), (3, 3), (3, 0)]],  # 第一个面
    [[(4, 1), (4, 4), (6, 4), (6, 1)]]   # 第二个面
]

# 创建Point和MultiPolygon对象
point = Point(point_coords)
multipolygon = MultiPolygon(multipolygon_coords)

# 计算点到多面体的距离
distance = point.distance(multipolygon)

print(distance)

from shapely.geometry import MultiPolygon

# 定义两个多面体的坐标
multipolygon1_coords = [
    [[(0, 0), (0, 3), (3, 3), (3, 0)]],  # 多面体1的第一个面
    [[(4, 1), (4, 4), (6, 4), (6, 1)]]   # 多面体1的第二个面
]

multipolygon2_coords = [
    [[(2, 0), (2, 2), (5, 2), (5, 0)]],  # 多面体2的第一个面
    [[(3, 3), (3, 6), (6, 6), (6, 3)]]   # 多面体2的第二个面
]

# 创建MultiPolygon对象
multipolygon1 = MultiPolygon(multipolygon1_coords)
multipolygon2 = MultiPolygon(multipolygon2_coords)

# 计算两个多面体之间的距离
distance = multipolygon1.distance(multipolygon2)

print(distance)

from shapely.geometry import LineString, MultiPolygon

# 定义线段和多面体的坐标
line_coords = [(1, 1), (4, 4)]
multipolygon_coords = [
    [[(0, 0), (0, 3), (3, 3), (3, 0)]],  # 多面体的第一个面
    [[(4, 1), (4, 4), (6, 4), (6, 1)]]   # 多面体的第二个面
]

# 创建LineString和MultiPolygon对象
line = LineString(line_coords)
multipolygon = MultiPolygon(multipolygon_coords)

# 计算线段到多面体的距离
distance = line.distance(multipolygon)

print(distance)

from shapely.geometry import LineString, MultiPolygon

# 定义折线和多面体的坐标
line_coords = [(1, 1), (2, 3), (4, 2), (5, 4)]
multipolygon_coords = [
    [[(0, 0), (0, 3), (3, 3), (3, 0)]],  # 多面体的第一个面
    [[(4, 1), (4, 4), (6, 4), (6, 1)]]   # 多面体的第二个面
]

# 创建LineString和MultiPolygon对象
line = LineString(line_coords)
multipolygon = MultiPolygon(multipolygon_coords)

# 计算折线到多面体的最短距离
distance = line.distance(multipolygon)

print(distance)
