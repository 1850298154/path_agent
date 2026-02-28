import math
import numpy as np
import time
import copy
import matplotlib.pyplot as plt


class Intersection(object):
    def __init__(self, point, rect):
        self.point = point
        self.rect = rect


class RectCorner(object):
    def __init__(self, corner):
        self.corner = corner
        self.visited = False


class Rectangular(object):
    def __init__(self, center: np.ndarray, width: float, height: float):
        self._center = center
        self._width = width
        self._height = height
        self._s = 1.0
        self.corners = []

        self.initialize_corner()

    def initialize_corner(self):
        rect_half_width = self.width / 2
        rect_half_height = self.height / 2
        rect_left = self.center[0] - rect_half_width
        rect_right = self.center[0] + rect_half_width
        rect_top = self.center[1] + rect_half_height
        rect_bottom = self.center[1] - rect_half_height
        left_top = RectCorner(np.array([rect_left, rect_top]))
        left_bottom = RectCorner(np.array([rect_left, rect_bottom]))
        right_top = RectCorner(np.array([rect_right, rect_top]))
        right_bottom = RectCorner(np.array([rect_right, rect_bottom]))

        self.corners = [left_top,
                        left_bottom,
                        right_top,
                        right_bottom]

    @property
    def center(self) -> np.ndarray:
        return np.array(self._center)

    @property
    def width(self) -> float:
        return self._width

    @property
    def height(self) -> float:
        return self._height

    @property
    def s(self) -> float:
        return self._s

    def function(self, q: np.ndarray) -> float:
        x, y, x_0, y_0, a, b = q[0], q[1], self.center[0], self.center[1], self.width / \
            2, self.height / 2
        value = ((x - x_0) ** 2 + (y - y_0) ** 2 + (((x - x_0) ** 2 -
                                                     (y - y_0) ** 2 + b ** 2 - a ** 2) ** 2 + (
            1 - self.s ** 2) * (
            a ** 2 + b ** 2)) ** 0.5) - (a ** 2 + b ** 2)
        return value

    def compute_squicle_length_ray(self, q: np.ndarray):
        normalized_q = q / np.linalg.norm(q)
        transformed_q = np.array(
            [normalized_q[0] / self.width, normalized_q[1] / self.height])
        normalized_transformed_q = transformed_q / \
            np.linalg.norm(transformed_q)
        scale = math.sqrt(
            (normalized_transformed_q[0] * self.width) ** 2 + (normalized_transformed_q[1] * self.height) ** 2)
        rho_q = scale * math.sqrt(
            2 / (1 + math.sqrt(1 - 4 * self.s ** 2 * (normalized_transformed_q[0] * normalized_transformed_q[1]) ** 2)))
        return rho_q

    def check_point_inside(self, q: np.ndarray):
        if self.function(q) < 1e-6:
            return True
        else:
            return False

    def plot_rectangular(self, ax, color='r'):
        center_rect_i = (self.center[0] - self.width / 2,
                         self.center[1] - self.height / 2)
        original_rect_i = plt.Rectangle(center_rect_i, self.width, self.height, edgecolor=color,
                                        facecolor=color, linewidth=1.5, fill=True, alpha=0.6)
        ax.add_patch(original_rect_i)
        return ax


class Line(object):
    def __init__(self, start, end):
        self._start = start
        self._end = end

    @property
    def start(self) -> np.ndarray:
        return np.array(self._start)

    @property
    def end(self) -> np.ndarray:
        return np.array(self._end)

    def check_point_on_line(self, q):
        start_x = self.start[0]
        start_y = self.start[1]
        end_x = self.end[0]
        end_y = self.end[1]
        value = abs((end_y - start_y) * q[0] - (end_x - start_x)
                    * q[1] + end_x * start_y - end_y * start_x)
        if value < 1e-3:
            return True
        else:
            return False

    def check_point_between_line(self, q):
        start_x = self.start[0]
        start_y = self.start[1]
        end_x = self.end[0]
        end_y = self.end[1]
        center = np.array([(start_x + end_x) / 2, (start_y + end_y) / 2])
        width = abs(start_x - end_x)
        height = abs(start_y - end_y)
        virtual_rect = Rectangular(center, width, height)
        value = virtual_rect.function(q)
        if value < 1e-6:
            return True
        else:
            return False

    def plot_line(self, ax):
        x_set = [self.start[0], self.end[0]]
        y_set = [self.start[1], self.end[1]]
        ax.plot(x_set, y_set, color='r', linewidth=1.5)
        return ax


class BugPlanner(object):

    def __init__(self, start_point, goal_point, step_size, inflated_size, obstacle_list):
        self.start_point = start_point
        self.goal_point = goal_point
        self.step_size = step_size
        self.inflated_size = inflated_size
        self.obstacle_list = obstacle_list
        self.current_start_point = start_point
        self.path = [start_point]
        self.inflated_rects = []

        self.min_intersection = None
        self.min_obstacle = None
        self.nearest_rect_corner = None
        self.distance_from_start_to_corner = None

        self.initialize_obstacle()
        # print(self.obstacles)

    def initialize_obstacle(self):
        for obs_data_i in self.obstacle_list:
            center_i = np.array(obs_data_i[0])
            width_i = obs_data_i[1] + 2 * self.inflated_size
            height_i = obs_data_i[2] + 2 * self.inflated_size
            obs_i = Rectangular(center_i, width_i, height_i)
            self.inflated_rects.append(obs_i)

    def distance(self, point_1, point_2):
        return np.linalg.norm(point_1 - point_2)

    def check_point_in_rect_corner(self, point, rect):
        for corner in rect.corners:
            if self.distance(corner.corner, point) < 1e-3:
                return True
        else:
            return False

    def line_rectangle_intersection(self, line, rect):
        rect_half_width = rect.width / 2
        rect_half_height = rect.height / 2
        rect_left = rect.center[0] - rect_half_width
        rect_right = rect.center[0] + rect_half_width
        rect_top = rect.center[1] + rect_half_height
        rect_bottom = rect.center[1] - rect_half_height

        start_x = line.start[0]
        start_y = line.start[1]
        end_x = line.end[0]
        end_y = line.end[1]

        intersections = []

        if start_x == end_x:
            y_top = rect_top
            y_bottom = rect_bottom
            if start_y < end_y:
                y_top = min(y_top, end_y)
                y_bottom = max(y_bottom, start_y)
            else:
                y_top = min(y_top, start_y)
                y_bottom = max(y_bottom, end_y)

            if y_bottom <= y_top:
                new_intersection_point = np.array([start_x, y_bottom])
                if not self.check_point_in_rect_corner(new_intersection_point, rect) and \
                        line.check_point_between_line(new_intersection_point):
                    new_intersection = Intersection(
                        new_intersection_point, rect)
                    intersections.append(new_intersection)

                new_intersection_point = np.array([start_x, y_top])
                if not self.check_point_in_rect_corner(new_intersection_point, rect) and \
                        line.check_point_between_line(new_intersection_point):
                    new_intersection = Intersection(
                        new_intersection_point, rect)
                    intersections.append(new_intersection)
        else:
            slope = (end_y - start_y) / (end_x - start_x)
            intercept = start_y - slope * start_x

            x_left = rect_left
            x_right = rect_right
            y_left = intercept + slope * x_left
            y_right = intercept + slope * x_right

            if rect_bottom <= y_left <= rect_top:
                new_intersection_point = np.array([x_left, y_left])
                if not self.check_point_in_rect_corner(new_intersection_point, rect) and \
                        line.check_point_between_line(new_intersection_point):
                    new_intersection = Intersection(
                        new_intersection_point, rect)
                    intersections.append(new_intersection)

            if rect_bottom <= y_right <= rect_top:
                new_intersection_point = np.array([x_right, y_right])
                if not self.check_point_in_rect_corner(new_intersection_point, rect) and \
                        line.check_point_between_line(new_intersection_point):
                    # print("new", new_intersection)
                    # print(rect.corners)
                    # print(self.check_point_in_rect_corner(new_intersection, rect))
                    # print("[x_right, y_right]")
                    new_intersection = Intersection(
                        new_intersection_point, rect)
                    intersections.append(new_intersection)

            y_top = rect_top
            y_bottom = rect_bottom
            x_top = (y_top - intercept) / slope
            x_bottom = (y_bottom - intercept) / slope

            if rect_left <= x_top <= rect_right:
                new_intersection_point = np.array([x_top, y_top])
                if not self.check_point_in_rect_corner(new_intersection_point, rect) and \
                        line.check_point_between_line(new_intersection_point):
                    new_intersection = Intersection(
                        new_intersection_point, rect)
                    intersections.append(new_intersection)
            if rect_left <= x_bottom <= rect_right:
                new_intersection_point = np.array([x_bottom, y_bottom])
                if not self.check_point_in_rect_corner(new_intersection_point, rect) and \
                        line.check_point_between_line(new_intersection_point):
                    new_intersection = Intersection(
                        new_intersection_point, rect)
                    intersections.append(new_intersection)

        if intersections:
            return True, intersections
        else:
            return False, intersections

    def check_line_all_obstacles_intersection(self, line):
        for rect_i in self.inflated_rects:
            intersection, intersections = self.line_rectangle_intersection(
                line, rect_i)
            inside = rect_i.check_point_inside(
                line.start) or rect_i.check_point_inside(line.end)
            if (intersection and not inside) or len(intersections) > 1:
                print("rect center", rect_i.center)
                return True
        return False

    def nearest_intersection(self):
        all_intersections = []
        line = Line(self.current_start_point, self.goal_point)

        for rect_i in self.inflated_rects:
            intersect, intersections = self.line_rectangle_intersection(
                line, rect_i)
            if intersect:
                if len(intersections) == 1:
                    all_intersections.append(intersections[0])
                else:
                    all_intersections.append(intersections[0])
                    all_intersections.append(intersections[1])
        if len(all_intersections) == 0:
            self.min_intersection = None
            return
        min_distance_to_start = np.inf
        min_intersection_to_start = self.current_start_point

        for intersection_i in all_intersections:
            distance_to_start = self.distance(
                intersection_i.point, self.current_start_point)
            if distance_to_start < min_distance_to_start:
                min_distance_to_start = distance_to_start
                min_intersection_to_start = intersection_i
        self.min_intersection = min_intersection_to_start
        # print(self.min_intersection)

    def step_toward_intersection(self):
        intersection_to_start = self.min_intersection
        distance_to_start = self.distance(
            self.current_start_point, intersection_to_start.point)
        step_num = int(distance_to_start / self.step_size)
        if distance_to_start < 1e-5:
            distance_to_start = 1e-5
        vector_to_intersection = (
            intersection_to_start.point - self.current_start_point) / distance_to_start
        for step_i in range(step_num):
            self.current_start_point = self.current_start_point + \
                vector_to_intersection * self.step_size
            self.path.append(self.current_start_point)
        self.current_start_point = intersection_to_start.point
        self.path.append(intersection_to_start.point)

    def step_toward_corner(self):
        nearest_corner = self.nearest_rect_corner
        distance_to_corner = self.distance(
            self.current_start_point, nearest_corner)
        step_num = int(distance_to_corner / self.step_size)
        if distance_to_corner < 1e-5:
            distance_to_corner = 1e-5
        vector_to_corner = (
            nearest_corner - self.current_start_point) / distance_to_corner
        for step_i in range(step_num):
            self.current_start_point = self.current_start_point + \
                vector_to_corner * self.step_size
            self.path.append(self.current_start_point)
        self.current_start_point = nearest_corner
        self.path.append(nearest_corner)

    def step_toward_goal(self):
        distance_to_start = self.distance(
            self.current_start_point, self.goal_point)
        step_num = int(distance_to_start / self.step_size)
        if distance_to_start < 1e-5:
            distance_to_start = 1e-5
        vector_to_intersection = (
            self.goal_point - self.current_start_point) / distance_to_start
        for step_i in range(step_num):
            self.current_start_point = self.current_start_point + \
                vector_to_intersection * self.step_size
            self.path.append(self.current_start_point)
        self.current_start_point = self.goal_point
        self.path.append(self.goal_point)

    def nearest_obstacle(self):
        self.min_obstacle = self.min_intersection.rect
        # print(self.min_obstacle.center)

    def find_intersection_nearest_corner(self):
        nearest_obstacle = self.min_obstacle
        nearest_intersection_point = self.min_intersection.point

        distance_from_start_to_corner = np.inf
        cost_to_go = np.inf
        nearest_rect_corner = None
        # print("\n=============================")
        for corner in nearest_obstacle.corners:
            # print("current_point", self.current_start_point)
            # print("corner_point", corner.corner)
            # print("corner_visited", corner.visited)
            if (abs(nearest_intersection_point[0] - corner.corner[0]) < 1e-3 or
                    abs(nearest_intersection_point[1] - corner.corner[1]) < 1e-3):
                if self.distance(nearest_intersection_point, corner.corner) + \
                        self.distance(corner.corner, self.goal_point) < cost_to_go:
                    nearest_rect_corner = copy.copy(corner)

                    distance_from_start_to_corner = self.distance(
                        self.current_start_point, corner.corner)
                    cost_to_go = self.distance(nearest_intersection_point, corner.corner) + \
                        self.distance(corner.corner, self.goal_point)
                    # print("corner", corner.corner)
                    # print("current_point", self.current_start_point)
                    # print("distance_from_start_to_corner", distance_from_start_to_corner)
        self.nearest_rect_corner = nearest_rect_corner.corner
        self.distance_from_start_to_corner = distance_from_start_to_corner
        # print("nearest_corner", self.nearest_rect_corner)

    def find_nearest_corner(self):
        nearest_obstacle = self.min_obstacle

        distance_from_start_to_corner = np.inf
        cost_to_go = np.inf
        nearest_rect_corner = None
        # print("\n=============================")
        for corner in nearest_obstacle.corners:
            # print("current_point", self.current_start_point)
            # print("corner_point", corner.corner)
            # print("corner_visited", corner.visited)
            if not corner.visited and (abs(self.current_start_point[0] - corner.corner[0]) < 1e-3 or
                                       abs(self.current_start_point[1] - corner.corner[1]) < 1e-3):
                if self.distance(self.current_start_point, corner.corner) + \
                        self.distance(corner.corner, self.goal_point) < cost_to_go:
                    nearest_rect_corner = copy.copy(corner)

                    distance_from_start_to_corner = self.distance(
                        self.current_start_point, corner.corner)
                    cost_to_go = self.distance(self.current_start_point, corner.corner) + \
                        self.distance(corner.corner, self.goal_point)
                    # print("corner", corner.corner)
                    # print("current_point", self.current_start_point)
                    # print("distance_from_start_to_corner", distance_from_start_to_corner)
        for corner in nearest_obstacle.corners:
            if self.distance(nearest_rect_corner.corner, corner.corner) < 1e-3:
                corner.visited = True
        self.nearest_rect_corner = nearest_rect_corner.corner
        self.distance_from_start_to_corner = distance_from_start_to_corner
        # print("nearest_corner", self.nearest_rect_corner)

    def one_step_along_rect(self):
        nearest_rect_corner = self.nearest_rect_corner
        distance_from_start_to_corner = self.distance(
            self.current_start_point, nearest_rect_corner)
        # print(nearest_rect_corner)
        # print("distance_from_start_to_corner", self.distance_from_start_to_corner)
        if distance_from_start_to_corner < self.step_size:
            self.current_start_point = nearest_rect_corner
            self.path.append(nearest_rect_corner)
        else:
            vector_to_conor = (
                nearest_rect_corner - self.current_start_point) / distance_from_start_to_corner
            self.current_start_point = self.current_start_point + \
                vector_to_conor * self.step_size
            self.path.append(self.current_start_point)
        # print("current_start_point====", self.current_start_point)

    def run(self):

        while self.distance(self.current_start_point, self.goal_point) > self.step_size:
            # print(self.path[-1])
            self.nearest_intersection()
            # print("min_intersection", self.min_intersection)
            if self.min_intersection is None:
                self.step_toward_goal()
            else:
                self.nearest_obstacle()
                self.find_intersection_nearest_corner()
                line = Line(self.current_start_point, self.nearest_rect_corner)
                if self.check_line_all_obstacles_intersection(line):
                    self.step_toward_intersection()
                    print("============")
                else:
                    self.step_toward_corner()

                # self.step_toward_intersection()

                line = Line(self.current_start_point, self.goal_point)
                intersection, _ = self.line_rectangle_intersection(
                    line, self.min_obstacle)
                # print(intersection)
                # print(_)
                while intersection:
                    self.find_nearest_corner()
                    # print(self.nearest_rect_corner)
                    while self.distance(self.current_start_point, self.nearest_rect_corner) > self.step_size:
                        # print("distance", self.distance(self.current_start_point, self.nearest_rect_corner))
                        # print(intersection)
                        self.one_step_along_rect()
                        # print("nearest_rect_corner", self.nearest_rect_corner)
                        # print("current_start_point", self.current_start_point)
                        # print("intersection", intersection)
                        # print("intersection_points", _)
                        # print(self.current_start_point)
                    self.one_step_along_rect()
                    line = Line(self.current_start_point, self.goal_point)
                    intersection, _ = self.line_rectangle_intersection(
                        line, self.min_obstacle)
        if self.distance(self.start_point, self.goal_point) < self.step_size:
            self.path.append(self.goal_point)
        self.smooth_path()

    def smooth_path(self):
        final_path = self.path
        new_path = [final_path[0]]
        current_point = final_path[0]
        next_point = final_path[1]
        line = Line(current_point, next_point)
        for i, path_i in enumerate(final_path):
            if line.check_point_on_line(path_i):
                continue
            else:
                new_path.append(final_path[i - 1])
                current_point = final_path[i - 1]
                next_point = path_i
                line = Line(current_point, next_point)
        new_path.append(final_path[-1])
        self.path = new_path

        # final_path = self.path
        # if len(final_path) > 2:
        #     new_path = [final_path[0]]
        #     for i in range(len(final_path) - 1):
        #         current_point = final_path[i]
        #         count = 0
        #         for j in range(i + 1, len(final_path)):
        #             next_point = final_path[j]
        #             line = Line(current_point, next_point)
        #             if self.check_line_all_obstacles_intersection(line):
        #                 print("line", line.start, line.end)
        #                 print("===========collision===========")
        #                 new_path.append(next_point)
        #                 break
        #             count += 1
        #         i += count
        #     new_path.append(final_path[-1])
        # self.path = new_path

    def plot_rectangulars(self, ax):

        for rect_i in self.inflated_rects[0: len(self.obstacle_list)]:
            inflated_rect_i = rect_i
            origin_rect_i = Rectangular(rect_i.center, rect_i.width - 2 * self.inflated_size,
                                        rect_i.height - 2 * self.inflated_size)
            ax = inflated_rect_i.plot_rectangular(ax, color='grey')
            ax = origin_rect_i.plot_rectangular(ax, color='b')

    def plot_path(self, ax):
        path_x = []
        path_y = []
        ax.plot(self.path[0][0], self.path[0][1], color='r', marker='*')
        ax.plot(self.path[-1][0], self.path[-1][1], color='orange', marker='o')
        for path_i in self.path:
            # print(path_i)
            path_x.append(path_i[0])
            path_y.append(path_i[1])
        ax.plot(path_x, path_y, '-g', linewidth=1.5)



def obstacle_adapter(obstacle_list):
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

if __name__ == '__main__':
    # obscacles
    # [center_x, center_y], width, height
    obstacle_list = [[np.array([20.0, 20.0]), 10.0, 10.0],
                     [np.array([40.0, 40.0]), 20.0, 20.0],
                     [np.array([70.0, 70.0]), 10.0, 10.0]]

    start_point = np.array([4.873229445099659, 1.5969777481981196])

    end_point = np.array([0.873229445099659, 4.5969777481981196])

    agent_start = [(77.40459085448113, 97.6925422132548), (78.86434569252066, 92.44353828203033),
                   (66.20679980890706, 85.61165056129137), (67.7517756288355,
                                                            96.53077987008162),
                   (68.36233950255817, 25.06987315324649), (41.72413357777185,
                                                            16.94332670765216),
                   (9.841618061357556, 53.4409321793724), (30.512086054424678,
                                                           1.4879614245067003),
                   (96.32785777059202, 59.0348361920288), (89.77308130236506, 13.203804295232954)]
    agent_end = [(12.422927264051033, 8.121958675287246), (75.85743273079159, 47.84232274704885),
                 (19.34961145177644, 78.12345215159169), (14.779457744145816,
                                                          2.260018073337426),
                 (96.17496513492746, 51.44283076062357), (20.98963092151416,
                                                          5.100546122041285),
                 (40.27267022907046, 66.85502416103609), (95.90494867536484,
                                                          10.796985316676285),
                 (99.24586076543194, 79.45238243939187), (11.580667792436259, 48.6246814609457)]
    step_size = 2.0
    inflated_size = 2.0

    obstacle_list = [(98.62059293197879, 65.0468372745427, 25.0), (32.64867312871597, 22.275734052545317, 25.0), (98.85442763604486, 195.10023403713512, 25.0), (49.359636243945324, 219.5603649384463, 25.0), (7.208753320778191, 146.57413280963843, 25.0), (217.64660399324939, 90.96844346267316, 25.0), (159.85800356396993, 175.00932197257433, 25.0), (153.0625847682124, 8.085191135952424, 25.0), (49.38841506709535, 89.51238827447325, 25.0), (221.133747284546, 22.41361157698463, 25.0), (78.00457361348398, 152.0049010444844, 25.0), (94.4590965039842, 21.41025170744024, 25.0), (165.74449117974297, 60.80775991342344, 25.0), (157.08844300081194, 106.97640120596685, 25.0), (2.665120711323035, 207.14664382748597, 25.0), (206.22063669743966, 164.02300134736484, 25.0), (3.7876381331673867, 69.31633946603066, 25.0), (162.09669794590326, 219.66165786874146, 25.0), (218.6929267155397, 213.02648268249982, 25.0), (112.18142545146664, 108.48538257194997, 25.0)]
    obstacle_list = obstacle_adapter(obstacle_list)
    agent_start = [(47.90095227494814, 126.65080114316997), (235.00637286599743, 203.24826305952396), (90.08473182073986, 96.88486047918973), (37.45042366791427, 88.5477090520643), (145.05187094692062, 1.2688341950288962), (215.33998322432163, 56.66149914475456), (26.783742305877787, 197.79323239639024), (196.4569242609654, 152.79725591743215), (155.98971252856492, 82.38356739665814), (140.1159767476669, 99.67467758426535), (165.7043419405526, 212.2872296151849), (154.66260087451087, 94.46063017011099), (116.07416439261124, 186.03012178829948), (148.23016171501453, 107.64378456172432), (99.64629704485672, 2.4573627145690544), (83.10460422492483, 57.907447751569755), (196.38053775884975, 139.07039883502748), (197.81507386812413, 39.45638290221797), (83.72737064737349, 134.28648390770599), (198.58572452247614, 123.38675782357534), (184.300361927256, 168.24703631836584), (36.47970163919272, 222.97801389237537), (194.19881282573107, 125.7319738266041), (82.00411121295119, 103.75190957762051), (242.58649502305377, 138.87629617961727), (185.87975251490877, 163.3952922187407), (211.20146900846052, 44.640921302580914), (142.84709279861443, 185.17320157463377), (100.04806603659675, 104.39654415206365), (8.712337406547567, 54.668879898200714), (51.48532717005151, 73.1411901404521), (119.4248044680655, 58.520502516202754), (224.0341845246843, 65.59181898347853), (68.87017130394311, 204.75993435133717), (5.41665098897769, 194.58116874603445), (160.75153126277849, 50.38998913107901), (138.23593220510244, 76.88788461284194), (112.83047490696701, 141.6146442103642), (18.381499295590665, 47.86405787067686), (16.935640512349316, 108.29326030683544), (127.33402054533137, 44.87524233433697), (128.87669402235008, 16.424176689866865), (26.870686895108225, 193.2226617790654), (32.42142823708346, 105.17705966470083), (44.55189316361144, 211.80458407020944), (231.3678241749853, 82.39958736632667), (80.53445394804888, 224.8871457287561), (145.5620708782167, 72.79677240578187), (55.76010509307748, 143.06153844666363), (194.22610982720175, 17.27297368749234), (240.884454763759, 9.744161224202557), (214.79411357218163, 197.38027411928886), (76.75750781867892, 48.73309869698121), (201.5054223981056, 11.2366205246542), (209.8384444478183, 2.0094309637217087), (13.522709397678522, 243.84134585246284), (122.26020539688788, 1.303869609621776), (44.1619931331456, 78.18899048589351), (57.03037029802065, 60.84085828058031), (82.75871818575098, 8.079790664905868), (45.83581085984246, 199.54365534282812), (199.62109528641284, 186.54893755061505), (10.081249486822092, 109.61179527680348), (95.34783447901637, 11.303189608959775), (133.29269580802168, 184.4850923735157), (38.684492959957815, 196.9161026462083), (97.05268660994417, 144.87468828279708), (122.94765823958022, 174.48653712442658), (131.54858336639757, 93.34950630753345), (149.74047251909238, 42.82541844586608), (22.012023931252976, 116.0377812028543), (64.2019338388235, 6.390109337338374), (65.22763343566082, 35.41371976350672), (242.01041643486573, 126.11338142818519), (247.63596198708734, 195.83726798312472), (213.05742262686078, 145.2328889628018), (185.36368171852075, 2.6710600296365197), (129.29513789341317, 57.9443382389395), (148.33078464554754, 143.27487034023054), (201.05927745234519, 65.16876696774618), (177.04168683638062, 144.93239393540793), (197.66770118475122, 228.25378857636878), (86.97790506957514, 79.18276341491928), (72.42244390082928, 67.81760341412652), (228.58238941532713, 69.71244769924837), (201.55012371412826, 157.38608044701115), (193.1493307618645, 168.26133464206015), (70.9978532275655, 180.06825024287483), (226.96925265623207, 124.62522701332651), (74.19061596680615, 35.76303255699662), (203.60536660257924, 88.53473283221619), (27.61584068146562, 136.42101200741544), (8.905356897290712, 38.2606055591195), (139.92417977174978, 67.25791268775055), (205.20462914410948, 208.87424826927233), (189.7182975550795, 45.57475752758442), (1.518586693926772, 244.33112243091318), (202.09216770390162, 103.74756140258742), (75.24317252533733, 188.49505446365848), (100.3118129424677, 130.1806547454348)]
    agent_start=agent_start[47:48]
    agent_start=[[129,90]]
    agent_end = [(159.53948166635314, 145.57385901077), (55.18842115293451, 77.82530212313605), (228.20420589684062, 247.77106364059085), (209.9545182763212, 200.69797365349964), (144.8835265812631, 233.68596916953902), (1.766673362035152, 139.75994446288155), (165.03189049747064, 147.314188748654), (53.83082827933578, 196.53035848404735), (125.6113076678872, 33.629231590372434), (247.1734904460312, 149.63286993624826), (159.84440865725438, 98.844981336093), (245.31180157447005, 13.550887585934795), (22.4008974043086, 21.06805199090884), (40.13794765171795, 88.01136484391199), (103.12627075416937, 232.0245507218739), (236.71952158713376, 132.37757666790912), (50.13357112458676, 136.40175837302306), (12.733376741841962, 32.81311433001069), (217.96789707952195, 195.23834650852444), (223.79178059472548, 14.256395027532932), (43.844518925645744, 182.61430402607883), (201.1480758916558, 243.35748861103448), (81.04668211171688, 211.7489965731593), (3.521185979372844, 241.97416062770498), (203.14277257118937, 206.8607956536928), (51.25276015052673, 69.25587380345213), (64.11780341717153, 160.63735287198085), (4.155069630942049, 16.466075575055637), (98.58204546852312, 120.37755998336137), (97.83102626830157, 141.46626899799978), (113.8241122809035, 238.91615311998706), (27.117010539220107, 115.96066232337361), (140.3188542868691, 205.21902504394316), (214.05548311839001, 30.6403586079933), (191.9098867481814, 15.68087886053155), (115.1283272067808, 180.53995491205302), (245.13462753144324, 131.16577381744452), (59.06140802093565, 197.96047617322196), (43.03638996030887, 238.9962947500149), (118.04814154833163, 165.97138361120264), (3.7332021330313534, 127.47897483496433), (58.107023787419095, 148.34513047118674), (74.30956848804934, 64.4211973337541), (238.02183886691532, 54.5375186709798), (194.50959288953882, 122.14740147340399), (109.55377563565243, 98.74214507421986), (122.9320834722623, 98.3897262208468), (2.515376970880639, 107.1384810490979), (209.57893556933556, 40.71907221322048), (36.9825564584647, 75.12232406660266), (34.40458165535481, 226.30182863846437), (88.17778523826479, 88.04750223259468), (15.845470492395236, 136.90845475122492), (35.396917504058464, 4.53811486205705), (196.1112710524008, 232.95311960789405), (95.93005420555687, 11.299068645516606), (244.27849215781256, 181.61631459546908), (218.3842245271793, 122.52581965271581), (88.27645198070537, 43.881654695244535), (84.91445818510259, 234.76641085323874), (158.29639998622602, 77.15056503481193), (90.26547490258079, 56.315470010072595), (219.42190835523533, 7.332730755680173), (141.91489519695733, 31.659311112129167), (79.21381095163657, 137.6125748851883), (87.16823645392245, 112.53808917479596), (60.91568526804321, 129.6106258665728), (92.19835595954801, 200.45753345734812), (119.43080750238188, 242.6166170721509), (6.664544201817671, 4.7784182175469425), (51.739673521981274, 121.49918771159726), (80.72919213873865, 27.11379123596559), (144.83831358425493, 215.45080909055633), (77.72613775929443, 9.81633304373714), (111.30595524926831, 226.71911794229212), (11.732496748386286, 120.2691427987116), (29.849107450744345, 241.48429699166195), (207.06200262704408, 71.92462525036446), (40.628465364705065, 163.89574288848365), (146.5570025346, 206.44894280223124), (131.37294962962295, 47.41940941437971), (224.2938020646243, 78.81861164574315), (217.13028773170296, 202.9727119641645), (140.0052204985611, 53.773402998744444), (206.46377245375857, 58.14741496395385), (210.5810057609687, 93.31545489482104), (69.43086608345602, 61.62277613507778), (145.9347499641178, 141.91600593712894), (242.71577799022825, 8.282402256768986), (185.7573633536703, 47.97925355657163), (86.05698638989614, 103.4328203034551), (122.44614350143807, 58.40237391297058), (52.01186595482652, 209.9992878197609), (200.78136343279368, 87.53447635426396), (40.4118509033227, 9.313610048637544), (130.4522422529173, 169.3970913558085), (159.58888303573235, 139.60242264716143), (78.49026799928333, 14.057387472677181), (241.65612444777844, 156.67482642733935), (168.26582355767061, 92.21944118620243)]
    agent_end=agent_end[47:48] # 47号 agent 路径有问题， 只看 47号agent
    step_size =  0.5
    inflated_size = 4.0

    # obstacle_list = 
    # agent_start =
    # agent_end =
    # step_size = 2.0
    # inflated_size = 2.0

    fig = plt.figure()
    ax = fig.add_subplot(111)
    ax.set_xlim([0, 100])
    ax.set_ylim([0, 100])
    ax.set_xlim([0, 200])
    ax.set_ylim([0, 200])
    ax.set_aspect('equal')

    for i, agent_start_i in enumerate(agent_start):
        start_point = np.array(agent_start[i])
        end_point = np.array(agent_end[i])

        # print("=============agent", i, "======")
        # print("start_point", start_point)
        # print("end_point", end_point)
        time_start = time.time()
        bug_planner = BugPlanner(
            start_point, end_point, step_size, inflated_size, obstacle_list)

        bug_planner.run()
        final_path = bug_planner.path
        bug_planner.plot_path(ax)
        time_end = time.time()
        total_time = time_end - time_start
        print(total_time)
    bug_planner = BugPlanner(start_point, end_point,
                             step_size, inflated_size, obstacle_list)
    bug_planner.plot_rectangulars(ax)

    plt.show()

    # print(final_path)
    #
    # [print(path_i) for path_i in final_path]
