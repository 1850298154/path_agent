import output_filename as of
import zrand as zr
import numpy as np
import time
import math

cnt = 0


class BugPlanner:
    """简化的BUG路径规划算法实现"""
    def __init__(self, start_point, end_point, step_size, inflated_size, obstacle_list):
        self.start = np.array(start_point, dtype=float)
        self.goal = np.array(end_point, dtype=float)
        self.step_size = step_size
        self.inflated_size = inflated_size
        self.obstacle_list = obstacle_list
        self.path = [self.start.copy()]
        self.current = self.start.copy()
        self.mode = 'direct'  # 'direct' = direct to goal, 'follow' = follow obstacle
        self.leaving_point = None
        self.goal_direction = (self.goal - self.start)
        self.goal_distance = np.linalg.norm(self.goal_direction)
        if self.goal_distance > 0:
            self.goal_direction = self.goal_direction / self.goal_distance
        self.max_steps = 1000
        self.step_count = 0

    def is_collision(self, point):
        """检查点是否与障碍物碰撞"""
        for ob in self.obstacle_list:
            center = np.array(ob[0])
            half_width = ob[1] / 2.0
            half_height = ob[2] / 2.0
            # 矩形碰撞检测
            if (point[0] >= center[0] - half_width - self.inflated_size/2 and
                point[0] <= center[0] + half_width + self.inflated_size/2 and
                point[1] >= center[1] - half_height - self.inflated_size/2 and
                point[1] <= center[1] + half_height + self.inflated_size/2):
                return True
        return False

    def run(self):
        """运行BUG算法"""
        while self.step_count < self.max_steps:
            self.step_count += 1

            # 检查是否到达目标
            if np.linalg.norm(self.current - self.goal) < self.step_size:
                self.path.append(self.goal.copy())
                break

            if self.mode == 'direct':
                # 直接朝目标移动
                direction = self.goal - self.current
                dist = np.linalg.norm(direction)
                if dist > 0:
                    direction = direction / dist
                next_pos = self.current + direction * self.step_size

                # 检查是否即将碰撞
                if self.is_collision(next_pos):
                    self.mode = 'follow'
                    self.leaving_point = self.current.copy()
                    # 计算绕障碍物方向（逆时针）
                    perp = np.array([-direction[1], direction[0]])
                    self.follow_direction = perp
                else:
                    self.current = next_pos
                    self.path.append(self.current.copy())

            elif self.mode == 'follow':
                # 沿障碍物边界移动
                # 尝试多个方向找到可行路径
                directions = [
                    self.follow_direction,
                    np.array([-self.follow_direction[1], self.follow_direction[0]]),
                    -self.follow_direction,
                    np.array([self.follow_direction[1], -self.follow_direction[0]])
                ]

                moved = False
                for direction in directions:
                    next_pos = self.current + direction * self.step_size
                    if not self.is_collision(next_pos):
                        self.current = next_pos
                        self.path.append(self.current.copy())
                        self.follow_direction = direction
                        moved = True
                        break

                if not moved:
                    # 如果无法移动，尝试随机方向
                    angle = np.random.uniform(0, 2 * np.pi)
                    direction = np.array([np.cos(angle), np.sin(angle)])
                    next_pos = self.current + direction * self.step_size
                    if not self.is_collision(next_pos):
                        self.current = next_pos
                        self.path.append(self.current.copy())

                # 检查是否可以离开障碍物
                to_goal = self.goal - self.current
                dist_to_goal = np.linalg.norm(to_goal)
                if dist_to_goal > 0:
                    to_goal = to_goal / dist_to_goal

                    # 检查离开点距离
                    dist_from_leaving = np.linalg.norm(self.current - self.leaving_point)
                    if dist_from_leaving > self.step_size * 5:  # 确保离开足够远
                        # 检查是否可以直接朝目标移动
                        test_pos = self.current + to_goal * self.step_size
                        if not self.is_collision(test_pos):
                            # 检查是否比离开点更接近目标
                            if np.linalg.norm(self.current - self.goal) < np.linalg.norm(self.leaving_point - self.goal) - self.step_size:
                                self.mode = 'direct'

        # 转换为numpy数组
        self.path = np.array(self.path)


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
    # 将圆形障碍物 [center_x, center_y, radius] 转换为方形障碍物格式 [center, width, height]
    # 使用直径作为方形边长，确保碰撞检测区域正确
    obstacle_list = [
        [
            [ob[0], ob[1]],  # 中心点保持不变
            ob[-1] * 2,      # 宽度 = 直径
            ob[-1] * 2       # 高度 = 直径
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
