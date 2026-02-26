"""
简化的距离计算模块
由于原zdist.py已删除，提供geometry.py所需的基本函数
"""
import numpy as np

def distance_between_polygon_peer(poly1_vertices, poly2_vertices):
    """
    简化的多边形距离计算
    返回一个简化值用于碰撞检测
    """
    # 简化版本：返回最小边界框距离
    # 真实的复杂多边形距离计算需要更复杂的算法
    try:
        # 计算两个多边形中心之间的距离
        center1 = np.mean(poly1_vertices, axis=0)
        center2 = np.mean(poly2_vertices, axis=0)
        return np.linalg.norm(center1 - center2)
    except:
        return 1000.0  # 默认大距离

def distance_between_polygon_line(poly_vertices, line_vertices):
    """
    简化的多边形到线段距离计算
    返回一个简化值用于碰撞检测
    """
    try:
        # 计算多边形中心到线段的距离
        center = np.mean(poly_vertices, axis=0)
        # 简化：计算中心到线段中点的距离
        if len(line_vertices) >= 2:
            mid_point = (np.array(line_vertices[0]) + np.array(line_vertices[1])) / 2
            return np.linalg.norm(center - mid_point)
        else:
            return 1000.0
    except:
        return 1000.0  # 默认大距离
