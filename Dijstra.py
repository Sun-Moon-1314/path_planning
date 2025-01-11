
import numpy as np
import heapq

class Dijkstra:
    """
    Dijkstra图搜索算法
    """
    def __init__(self, direction_graph = None, x_size = 5, y_size = 5):
        self.neighbors = [(-1, 0), (1, 0), (0, -1), (0, 1)]  # 上下左右
        self.grid_size = (x_size, y_size)
        self.grid = np.ones(self.grid_size)
        self.grid_map = self.generate_graph(self.grid)
        self.print_map()
        self.undirected_graph = None
        if direction_graph:
            self.directed_graph = direction_graph
        else:
            self.directed_graph = {
                'P1': {'P2': 5, 'P5': 20, 'P6': 40},
                'P2': {'P3': 50},
                'P3': {'P4': 20, 'P5': 10},
                'P4': {'P5': 60},
                'P5': {},
                'P6': {'P2': 10, 'P4': 30, 'P5': 100}
            }

    def generate_graph(self, grid):
        """
        生成栅格图
        :param grid:
        :return:
        """
        rows, cols = grid.shape
        self.grid_map = {}

        for i in range(rows):
            for j in range(cols):
                current_node = (i, j)
                neighbors_list = []

                # 遍历邻居
                for dx, dy in self.neighbors:
                    ni, nj = i + dx, j + dy
                    if 0 <= ni < rows and 0 <= nj < cols:  # 确保在图的范围内
                        neighbor_node = (ni, nj)
                        weight = grid[ni, nj]  # 假设边的权重为邻居节点的权重
                        neighbors_list.append((neighbor_node, weight))

                self.grid_map[current_node] = neighbors_list

        return self.grid_map

    def set_grid_map(self, point_a, point_b):
        """
        设置权重
        :param point_a:
        :param point_b:
        :return:
        """
        pass

    def set_obstacle(self, obstacle_list):
        """
        设置障碍物
        :param obstacle_list:
        :return:
        """
        pass

    def print_map(self):
        # 打印邻接表
        for node, edges in self.grid_map.items():
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

    @classmethod
    def search_dijkstra(cls, graph, start, end):
        """
        dijkstra寻路核心算法
        :param graph:
        :param start:
        :param end:
        :return:
        """
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
            if current_node == end:
                path = []
                # 当前节点有前驱节点，将当前节点插入到path，找到前驱节点，作为当前节点
                while previous_node[current_node] is not None:
                    path.insert(0, current_node)
                    current_node = previous_node[current_node]
                path.insert(0, start)
                return path
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
        return []

    @classmethod
    def distance_dijkstra(cls, graph, start):
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
    d = Dijkstra()
    # distances1 = dijkstra(graph, 'P1')
    # distances2 = dijkstra_copy(graph, 'P2')
    # if distances1 == distances2:
    #     print("True")
    # else:
    #     if sum(distances1.values()) < sum(distances2.values()):
    #         print(f"Less distance: distance1->{distances1}")
    #         print(f"distance2:{distances2}")
    #     else:
    #         print(f"Less distance: distance2->:{distances2}")
    #         print(f"distance1:{distances1}")
