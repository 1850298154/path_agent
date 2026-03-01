"""
线段类，用于计算线段交点、截距范围等
"""
import numpy as np


class LineSeg:
    """线段类，用于计算线段交点、截距范围等"""

    def __init__(self, start_pt, end_pt):
        self.start_pt = np.array(start_pt)
        self.end_pt = np.array(end_pt)

        self.x, self.y = start_pt
        self.x2, self.y2 = end_pt

        if self.x != self.x2:
            self.m = (self.y2 - self.y) / (self.x2 - self.x)
            self.b = self.y - self.m * self.x

            self.minv = min(self.x, self.x2)
            self.maxv = max(self.x, self.x2)
        else:
            self.m = None
            self.b = self.x
            self.minv = min(self.y, self.y2)
            self.maxv = max(self.y, self.y2)

    def length(self):
        """返回线段长度"""
        return np.linalg.norm([self.x2 - self.x, self.y2 - self.y])

    def intersect_w_line(self, m, b):
        """
        求线段 self 与线 y = mx + b 的交点

        Args:
            m: 直线斜率
            b: 直线截距

        Returns:
            (x, y): 交点坐标，如果无交点则返回 (None, None)
        """
        # 平行线
        if m == self.m:
            return (None, None)

        # 直线垂直但线段不垂直
        elif m is None:
            if self.minv <= b <= self.maxv:
                return (b, self.m * b + self.b)
            else:
                return (None, None)

        # 线段垂直，但直线不垂直
        elif self.m is None:
            y = m * self.b + b

            if self.minv <= y <= self.maxv:
                return (self.b, y)
            else:
                return (None, None)
        else:
            x = (b - self.b) / (self.m - m)
            y = self.m * x + self.b

            if self.minv <= x <= self.maxv:
                return (x, y)
            else:
                return (None, None)

    def intercept_range(self, m):
        """
        使用斜率 m 求两个端点的截距跨度

        Args:
            m: 直线斜率

        Returns:
            [b1, b2]: 两个端点的截距，已排序
        """
        if self.m == m:
            return (self.b, self.b)

        # 直线垂直但线段不垂直
        elif m is None:
            return sorted([self.x, self.x2])

        # 直线不垂直
        else:
            b = self.y - m * self.x
            b2 = self.y2 - m * self.x2

            return sorted([b, b2])

    def calc_alpha(self, p):
        """
        计算 p 点在线段上的投影位置比例
        alpha = (AC dot AB) / (|AB|^2)

        Args:
            p: 点坐标

        Returns:
            alpha: 投影点 C 在线段 AB 上的比例
        """
        return np.dot(
            (self.end_pt - self.start_pt), (p - self.start_pt)
        ) / (self.length() * self.length())

    def distance_to_point(self, p):
        """
        计算点到线段的距离

        Args:
            p: 点坐标

        Returns:
            distance: 点到线段的距离
        """
        alpha = self.calc_alpha(p)
        if alpha <= 0 or alpha >= 1:
            return np.min([
                np.linalg.norm(p - self.start_pt),
                np.linalg.norm(p - self.end_pt)
            ])
        else:
            return np.abs(
                (self.y2 - self.y) * p[0] -
                (self.x2 - self.x) * p[1] +
                self.x2 * self.y -
                self.y2 * self.x
            ) / self.length()
