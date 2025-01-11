
import numpy as np
import heapq
import math

class Dijkstra:
    """
    Dijkstra图搜索算法类，支持栅格图和有向图。

    栅格图中：
        - 0 代表障碍物
        - 1 代表可通行区域

    Attributes:
        directed_graph (dict): 有向图的邻接表
        grid (numpy.ndarray): 栅格图的表示
    """
    __neighbors = {"上": (-1, 0), "下": (1, 0), "左": (0, -1), "右": (0, 1)}  # 上下左右
    def __init__(self, graph = None, start = None, goal = None, is_grid = False):
        # 是栅格图
        if is_grid:
            self.__grid = graph
            self.__grid_size = len(graph[0])
            self._directed_map = self.__generate_direct_graph()
        # 不是栅格图
        else:
            self._directed_map = graph
            self.__grid_size = len(graph.keys())

        self.__start = start
        self.__goal = goal


    @property
    def grid(self):
        return self.__grid

    @property
    def directed_map(self):
        return self._directed_map

    @directed_map.setter
    def directed_map(self, obstacles):
        """
        设置图的障碍物
        :param obstacle:
        :return:
        """
        self.__set_obstacle(obstacles)

    def __set_obstacle(self, obstacles):
        """
        设置障碍物
        :param obstacles:
        :return:
        """
        for obstacle in obstacles:
            if obstacle in self._directed_map:
                # 移除障碍物的出边
                self._directed_map[obstacle] = {}

                # 需要遍历图中其他节点，移除指向障碍物的边
                for node in self._directed_map:
                    if obstacle in self._directed_map[node]:
                        del self._directed_map[node][obstacle]

    @classmethod
    @property
    def neighbors(cls):
        return cls.__neighbors

    def __generate_direct_graph(self):
        """
        生成栅格图
        :param grid:
        :return:
        """
        rows, cols = self.__grid.shape
        directed_graph = {}

        for i in range(rows):
            for j in range(cols):
                if self.__grid[i, j] == 0:  # 如果是障碍物，跳过
                    continue

                node = (i, j)
                directed_graph[node] = {}

                # 遍历邻居
                for dx, dy in self.__neighbors.values():
                    ni, nj = i + dx, j + dy
                    if 0 <= ni < rows and 0 <= nj < cols and self.__grid[ni, nj] != 0:
                        # 只考虑可以通行的邻居节点
                        neighbor_node = (ni, nj)
                        directed_graph[node][neighbor_node] = 1  # 假设每个边的权重是 1（可以根据实际情况调整）

        return directed_graph

    def set_grid_map(self, point_a, point_b):
        """
        设置权重
        :param point_a:
        :param point_b:
        :return:
        """
        pass

    def print_map(self):
        # 打印邻接表
        for node, edges in self._directed_map.items():
            print(f"Node {node}: {edges}")

    def compare_algorithm(self, distances1, distances2):
        """
        比较算法
        :param distances1:
        :param distances2:
        :return:
        """
        if distances1 == distances2:
            print("True")
        else:
            if sum(distances1.values()) < sum(distances2.values()):
                print(f"Less distance: distance1->{distances1}")
                print(f"distance2:{distances2}")
            else:
                print(f"Less distance: distance2->:{distances2}")
                print(f"distance1:{distances1}")


    def search_dijkstra(self, graph = None, start = None, goal = None):
        """
        dijkstra寻路核心算法
        :param graph:
        :param start:
        :param goal:
        :return:
        """
        if graph is None:
            graph = self._directed_map
        if start is None:
            start = self.__start
        if goal is None:
            goal = self.__goal
        # 定义每个节点的距离记录表
        distances = {node: float('inf') for node in graph}
        # 每个点是否被访问过
        flag = {node: False for node in graph}
        # 定义前驱节点，比如邻居节点的前驱节点为当前节点
        previous_node = {node: None for node in graph}
        # 初始化优先队列
        priority_queue = [(0, start)]
        # 初始化出发点的distances
        distances[start] = 0
        # 不断检查priority_queue，取出经过当前节点的最小距离的邻居节点
        while priority_queue:
            current_distance, current_node = heapq.heappop(priority_queue)
            flag[current_node] = True
            if current_node == goal:
                path = []
                # 当前节点有前驱节点，将当前节点插入到path，找到前驱节点，作为当前节点
                while previous_node[current_node] is not None:
                    path.insert(0, current_node)
                    current_node = previous_node[current_node]
                path.insert(0, start)
                return path, distances, previous_node
            # 如果从优先队列中提取的点的当前的距离比distances中记录的当前点距离更长，说明a->c之间经过b更长，就不用更新distances
            if current_distance > distances[current_node]:
                continue

            # 遍历当前节点的邻居节点，更新distances记录表，并将邻居节点及最小距离加入堆中，以便直接获取最小距离的节点
            for neighbor, weight in graph[current_node].items():
                # 从当前点经过到邻居节点的距离
                next_distance = weight + current_distance
                if flag[neighbor] is True:
                    continue
                # 如果这个距离小于distances中记录的距离，说明这条路径更短
                if next_distance < distances[neighbor]:
                    # 更新distances
                    distances[neighbor] = next_distance
                    # 将这个邻居节点和最短距离加入最小堆中
                    heapq.heappush(priority_queue, (next_distance, neighbor))
                    # 记录该邻居节点的前驱节点为当前节点
                    previous_node[neighbor] = current_node
        return [], distances, previous_node

    @property
    def grids(self):
        return self.__generate_grid()

    def __generate_grid(self):
        # 清空栅格图并初始化
        grid = np.zeros((self.__grid_size, self.__grid_size))

        # 根据有向图创建栅格图
        for node, neighbors in self._directed_map.items():
            # 获取节点的坐标
            node_pos = node
            if node_pos:
                for neighbor, weight in neighbors.items():
                    # 获取邻居的位置
                    neighbor_pos = neighbor
                    if neighbor_pos:
                        ni, nj = neighbor_pos
                        # 这里假设权重大于0即表示可通行，设置栅格图为1
                        if weight > 0:
                            grid[ni, nj] = 1  # 可通行的路径

        return grid

    @staticmethod
    def distance_dijkstra(graph, start):
        """
        使用Dijkstra算法计算从起点到图中所有其他节点的最短路径。

        参数:
        - graph: 字典形式的加权图，键为节点，值为邻接节点及其边权重的字典。
        - start: 起始节点。

        返回:
        - distances: 从起始节点到每个节点的最短距离字典。
        """
        # 初始化距离字典，所有距离设为无穷大，起始节点距离设为0
        distances = {node: float('infinity') for node in graph}
        distances[start] = 0

        # 优先队列，存储待处理的节点及其当前距离
        priority_queue = [(0, start)]

        while priority_queue:
            # 弹出当前距离最小的节点
            current_distance, current_node = heapq.heappop(priority_queue)

            # 如果当前节点的距离大于已记录的距离，跳过
            if current_distance > distances[current_node]:
                continue

            # 遍历当前节点的所有邻居
            for neighbor, weight in graph[current_node].items():
                distance = current_distance + weight

                # 如果找到更短的路径，更新距离并加入优先队列
                if distance < distances[neighbor]:
                    distances[neighbor] = distance
                    heapq.heappush(priority_queue, (distance, neighbor))

        return distances

if __name__ == "__main__":
    # direction_graph = {
    #     'P1': {'P2': 5, 'P5': 20, 'P6': 40},
    #     'P2': {'P3': 50},
    #     'P3': {'P4': 20, 'P5': 10},
    #     'P4': {'P5': 60},
    #     'P5': {},
    #     'P6': {'P2': 10, 'P4': 30, 'P5': 100}
    # }
    direction_graph = np.array([
        [1, 1, 1, 1, 1],
        [1, 0, 0, 1, 1],
        [1, 1, 1, 0, 1],
        [1, 0, 1, 1, 1],
        [1, 1, 1, 1, 1]
    ])

    start = (0, 0)  # 起点
    goal = (4, 4)  # 终点
    obstacles = [(1, 1), (1, 2), (2, 1)]  # 障碍物位置
    d = Dijkstra(graph=direction_graph, start=(0, 0),goal=(4, 3), is_grid=True)
    d.directed_map = obstacles
    path, dd, pre = d.search_dijkstra()
    print(d.grids)
    print(path)
