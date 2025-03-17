import numpy as np
import heapq
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation


class Point:
    """
    定义坐标点的类
    """

    def __init__(self, x=0, y=0):
        self.x = x  # 横坐标
        self.y = y  # 纵坐标
        self.f = 0  # 总代价
        self.g = 0  # 起点到当前节点的代价
        self.h = 0  # 当前节点到终点的代价
        self.parent = None  # 定义父节点
        self.radius = 0
        self.is_in_openlist = 0  # 待考察点列表
        self.is_in_closelist = 0  # 已考察节点

    def __lt__(self, other):
        return self.f < other.f


class Map:
    def __init__(self, map_size):
        self.map_size = map_size  # 搜索图大小
        self.width = map_size[0]  # x坐标长度
        self.height = map_size[1]  # y坐标长度
        self.map = [[Point(x, y) for y in range(self.map_size[1])] for x in range(self.map_size[0])]

    def set_obstacles(self, obstacles, obstacle_radius):
        """
        设置障碍范围
        :param obstacles: 障碍物点列表或者范围
        :return:
        """
        obstacle_list = []
        for obstacle in obstacles:
            x = obstacle[0]
            y = obstacle[1]
            # 有障碍物，就将该点设置为已关闭节点（要不然还得设置一个障碍标志，多少有点重复了）
            self.map[x][y].is_in_closelist = 1
            # 障碍物半径
            self.map[x][y].radius = obstacle_radius
            obstacle_list.append((x, y, 1))
        return obstacle_list


class A_star:
    """
    算法实现
    """

    def __init__(self, map, start_point, end_point, car_radious=0, num_connect=8):
        self.map: Map = map
        self.start_point = start_point
        self.end_point = end_point
        self.car_radius = car_radious  # 小车半径
        self.open_list = [(0, self.start_point)]  # 待考察点
        self.closed_list = []  # 考察完毕的点
        self.start_point.is_in_openlist = 1
        self.num_connect = num_connect  # 连通数，8个方向
        self.spread_direction = [(1, 0), (-1, 0), (0, 1), (0, -1), (1, 1), (-1, 1), (1, -1), (-1, -1)]

    def spread_point(self, current_point):
        """
        以当前点为核心扩展周围点
        :return:
        """
        search_points = []
        for direction in self.spread_direction:
            x = current_point.x + direction[0]
            y = current_point.y + direction[1]
            # 检测扩展的点是否会超出边界，是否已经处于关闭考察状态
            if x < 0 or x >= self.map.width or y < 0 or y >= self.map.height or self.map.map[x][y].is_in_closelist:
                continue
            # 具有半径的小车与具有半径的障碍物的碰撞检测，主要检测4个斜方向的路径是否可通过，假设在斜路径上的任意一侧有障碍，都无法通过
            if direction[0] != 0 and direction[1] != 0 and (
                    direction[0] == direction[1] or direction[0] == -direction[1]):
                # 减法的原因是上面x = current_point.x + direction[0]加了一个direction[0]
                if (self.map.map[x - direction[0]][y].is_in_closelist
                        or self.map.map[x][y - direction[1]].is_in_closelist):
                    continue
            search_points.append((x, y))
        return search_points

    def judge_in_obstacles(self, point):
        """
        判断是否在障碍物范围内或者在地图外
        :return:
        """
        if self.map.map[point[0]][point[1]].connect == 1:
            return True
        return False

    def calculate_cost(self, min_cost, parents, point):
        """
        计算每个点的f，g，h
        :return:
        """
        # 计算当前的点的代价，以欧式距离计算，如果以曼哈顿方式计算，斜方向就不会起作用
        cost_g = parents.g + np.sqrt(abs(parents.x - point.x) ** 2 + abs(parents.y - point.y) ** 2)
        cost_h = np.sqrt(abs(point.x - self.end_point.x) ** 2 + abs(point.y - self.end_point.y) ** 2)
        cost_f = round(cost_g + cost_h, 2)
        # 假如该没有没有在被考察列表中，就将该点代价计算出来放入带考察点中
        if point.is_in_openlist == 0 and point.is_in_closelist == 0:
            point.g = cost_g
            point.h = cost_h
            point.f = cost_f
            heapq.heappush(self.open_list, (cost_f, point))
            point.parent = parents
            point.is_in_openlist = 1
        # 如果该点已经在带考察列表中，则该点的之前就存在个父节点，究竟哪个父节点对该节点更亲近呢，肯定得拿出来比一比了
        elif point.is_in_openlist == 1 and point.is_in_closelist == 0:
            pre_parent = point.parent
            cost_g_pre = pre_parent.g + np.sqrt(abs(pre_parent.x - point.x) ** 2 + abs(pre_parent.y - point.y) ** 2)
            cost_f_pre = round(cost_g_pre + cost_h, 2)
            if cost_f < cost_f_pre:
                point.g = cost_g
                point.h = cost_h
                point.f = cost_f
                heapq.heappush(self.open_list, (cost_f, point))
                point.parent = parents
                point.is_in_openlist = 1

    def choose_mincost_point(self):
        """
        选取最小代价的点
        :return:
        """
        pass

    def search_path(self):
        """
        寻路核心逻辑:
        对于A*路径规划算法来说，一切定义的变量和函数逻辑都要围绕[f = g + h]公式进行，该启发式目标函数是该算法的核心，
        需要深入体会其中的思想，无论如何变化，精髓还是在此；
        :return:
        """
        # 这里从哈希map中弹出点，少了排序的过程，要不然还得加一个排序函数
        while self.open_list:
            try:
                # 为什么在弹出最小代价和点之后要设置考察点列表状态呢，因为该点在被弹出是已经被设置is_in_openlist=1，
                # 在被弹出那一刻，就注定该点永远不会被访问了，is_in_openlist=0，要不然你喜欢走回头路？乖乖的把is_in_closelist=1，
                # 永不回头
                min_cost, closest_point = heapq.heappop(self.open_list)
                closest_point.is_in_openlist = 0
                closest_point.is_in_closelist = 1
                # open_list实在太长了，每次弹出就清空吧，有人说万一需要探索其他的点怎么办，该算法的路径不会跳跃前进，弹出的点至少保证当前
                # 的点是可以过去的，至于之后咋走，就根据要过去的点向周围探索，这样在以f最小代价去探索，最合理不是吗，但有一点，别回头。
                heapq.heapify(self.open_list)
            except:
                raise "No path is found!"
            # 该逻辑为如果当前弹出的最小代价比之前还大，要你何用？
            if min_cost > closest_point.f:
                continue
            # 到达终点就输出路径
            if closest_point.x == self.end_point.x and closest_point.y == self.end_point.y:
                return self.output_path(closest_point)
            # 向8个方向扩展点
            search_point = self.spread_point(closest_point)
            # 每个可扩展的点计算[f = g + h]
            for point in search_point:
                self.calculate_cost(min_cost, closest_point, self.map.map[point[0]][point[1]])
        return []

    def output_path(self, point):
        """
        打印规划路径
        :return:
        """
        path = []
        while point.parent is not None and point.is_in_closelist == 1:
            path.insert(0, (point.x, point.y))
            point = point.parent
        path.insert(0, (self.start_point.x, self.start_point.y))
        return path

    def plot_animation(cls, start, goal, obstacle_list, path, robot_radius):
        fig, ax = plt.subplots()
        ax.set_xlim(0, 9)
        ax.set_ylim(0, 9)
        ax.grid(True)

        # 设置坐标轴刻度间隔为1
        ax.set_xticks(range(0, 10))
        ax.set_yticks(range(0, 10))

        # 绘制障碍物
        for obs in obstacle_list:
            circle = plt.Circle((obs[0], obs[1]), 0.5, color='black')  # 障碍物半径设为1
            ax.add_patch(circle)

        # 绘制起点和终点
        ax.scatter(start[1], start[0], color='green', marker='o', label='Start')  # 起点用绿色圆形表示
        ax.scatter(goal[1], goal[0], color='red', marker='x', label='Goal')  # 终点用红色叉号表示

        # 绘制路径
        path_x, path_y = zip(*path)
        line, = ax.plot(path_x, path_y, 'b-', label='Path', linewidth=2, markersize=5)

        def update(frame):
            if frame < len(path):
                line.set_data(path_x[:frame + 1], path_y[:frame + 1])
            else:
                # 质点到达终点，停止动画
                ani.event_source.stop()
                print("到达终点，重新生成路径...")
                # 这里可以添加生成新路径的逻辑
                # 例如：new_path = generate_new_path(start, goal, obstacle_list)
                # 然后重新调用 plot_animation 函数
            return line,

        ani = FuncAnimation(fig, update, frames=len(path), interval=200, blit=True)
        ax.legend()  # 添加图例
        plt.show()


if __name__ == '__main__':
    # 定义地图、起点、终点、障碍物列表等参数
    grid_size = 1
    # 定义地图大小
    map_size = (10, 10)
    Astar_map = Map(map_size)
    # 定义起点和终点
    start_point = Astar_map.map[1][1]
    end_point = Astar_map.map[8][8]
    # 小车半径
    car_radius = 0.5
    # 定义障碍物列表
    # obstacles = []
    # for i in range(20):
    #     x = np.random.randint(10)
    #     y = np.random.randint(10)
    #     if x == 1 and y == 1:
    #         continue
    #     if x == 8 and y == 8:
    #         continue
    #     if x % 2 == 0 and y % 2 == 0 and x==y:
    #         obstacles.append((x, y))
    #     obstacles.append((x,y))
    # print(obstacles)
    obstacles = [
        (0, 2),
        (1, 2),
        (2, 2), (2, 3), (2, 5),
        (3, 2), (3, 3), (3, 7),
        (4, 6),
        (5, 3), (5, 4), (5, 6), (5, 5), (5, 7), (5, 9),
        (6, 5),
        (7, 3), (7, 5), (7, 7), (7, 6), (7, 8),
        (8, 7),
        (9, 6)
    ]
    obstacles_list = Astar_map.set_obstacles(obstacles, 0.5)
    # 将障碍物映射到地图上
    path = A_star(Astar_map, start_point, end_point, car_radious)
    p = path.search_path()
    if len(p) > 1:
        path.plot_animation(start=(1, 1), goal=(8, 8), obstacle_list=obstacles_list, path=p, robot_radius=0.5)
    print(f"从起点{(start_point.x, start_point.y)}到终点{(end_point.x, end_point.y)}的路径是:{p}")
