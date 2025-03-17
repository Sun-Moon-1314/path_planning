from pickletools import markobject
from venv import logger

import numpy as np

class Floyd:
    """
    弗洛伊德算法
    """

    def __init__(self,
                 graph: dict[str, dict[str, int]] = None,
                 start: str = None,
                 goal: str = None):
        """
        :param graph: 邻接图
        :param start: 起点
        :param goal: 终点
        :return
            - 最短路径矩阵matrix,
            - 路径矩阵:path_matrix,
            - 路径：path
        """
        self.graph = graph
        self.start = start
        self.goal = goal
        self.matrix, self.node_dict = self.transfer_to_matrix()
        self.path_matrix = self.floyd()
        self.path = self.get_shortest_path()

    def transfer_to_matrix(self):
        """
        邻接图转换为二维矩阵
        :return:
        """
        keys_num = len(self.graph.keys())
        matrix = np.full((keys_num, keys_num), np.inf)
        for i in range(keys_num):
            matrix[i][i] = 0

        node_dict = {node: num for num, node in enumerate(self.graph)}

        for i, node in enumerate(self.graph):
            for j, neighbor in enumerate(self.graph[node]):
                if node_dict[node] != node_dict[neighbor]:
                    matrix[node_dict[node]][node_dict[neighbor]] = self.graph[node][neighbor]
                else:
                    matrix[node_dict[node]][node_dict[neighbor]] = 0

        return matrix, node_dict

    def floyd(self):
        """
        核心算法
        :return:
        """
        path_matrix = np.full((len(self.node_dict.keys()), len(self.node_dict.keys())), -1)
        for node, i in self.node_dict.items():
            for node, j in self.node_dict.items():
                path_matrix[i][j] = i
        for node_k, k in self.node_dict.items():
            for node_i, i in self.node_dict.items():
                for node_j, j in self.node_dict.items():
                    if i == j or self.matrix[i][j] == 0:
                        continue
                    if self.matrix[i][k] + self.matrix[k][j] < self.matrix[i][j]:
                        self.matrix[i][j] = self.matrix[i][k] + self.matrix[k][j]
                        path_matrix[i][j] = k

        return path_matrix

    def get_shortest_path(self):
        """
        获取最短路径
        :param path_matrix: 路径矩阵
        :param node_dict: 节点字典
        :param start: 起点
        :param goal: 终点
        :return: 路径列表
        """
        start_index = self.node_dict[self.start]
        end_index = self.node_dict[self.goal]
        path = [self.goal]
        reverse_node_dict = {value: key for key, value in self.node_dict.items()}
        while self.path_matrix[start_index][end_index] != start_index:
            end_index = self.path_matrix[start_index][end_index]
            previous_node = reverse_node_dict[self.path_matrix[start_index][end_index]]
            path.insert(0, previous_node)

        return path


if __name__ == "__main__":
    # # 原始图
    graph = {
        'P0': {'P1': 6, 'P2': 5, 'P3': 5},
        'P1': {'P4': 1, 'P0': 6, 'P2': 2},
        'P2': {'P1': 2, 'P3': 2, 'P4': 1},
        'P3': {'P0': 5, 'P2': 2, 'P5': 1},
        'P4': {'P1': 1, 'P2': 1, 'P6': 3},
        'P5': {'P3': 1, 'P6': 3},
        'P6': {'P4': 3, 'P5': 3}
    }

    # # 输入起始节点和结束节点
    start_node = 'P1'
    end_node = 'P6'
    f = Floyd(graph, start_node, end_node)
    # # 获取最短路径
    shortest_path = f.path
    print(f"从 {start_node} 到 {end_node} 的最短路径: {shortest_path}")
