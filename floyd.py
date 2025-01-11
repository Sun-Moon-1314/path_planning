
import numpy as np
def graph_to_matrix(graph):
    nodes = list(graph.keys())
    num_nodes = len(nodes)

    node_to_index = {node: i for i, node in enumerate(graph)}

    matrix = np.full((num_nodes, num_nodes), np.inf)

    for i in range(num_nodes):
        matrix[i][i] = 0

    for node, neighbor_dic in graph.items():
        for neighbor, weight in neighbor_dic.items():
            matrix[node_to_index[node], node_to_index[neighbor]] = weight

    return matrix, nodes, num_nodes, node_to_index

# def graph_to_matrix(graph):
#     # 获取所有节点的列表
#     nodes = list(graph.keys())
#     num_nodes = len(nodes)
#
#     # 初始化一个 n x n 的矩阵，初始值为无穷大
#     matrix = np.full((num_nodes, num_nodes), np.inf)
#
#     # 创建节点到索引的映射
#     node_to_index = {node: i for i, node in enumerate(nodes)}
#     for i in range(num_nodes):
#         matrix[i][i] = 0
#     # 填充矩阵
#     for node, neighbors in graph.items():
#         for neighbor, weight in neighbors.items():
#             matrix[node_to_index[node], node_to_index[neighbor]] = weight
#
#     return matrix, nodes, num_nodes, node_to_index
def floyd(matrix, num_nodes):

    path = np.full((num_nodes, num_nodes), -1, dtype=int)
    for i in range(num_nodes):
        for j in range(num_nodes):
            if matrix[i][j] != np.inf and i != j:
                path[i][j] = i
    # 从起点开始，逐步遍历经过当前节点是否有更短节点，有则更新，同时更新path矩阵
    for k in range(num_nodes):
        for i in range(num_nodes):
            for j in range(num_nodes):
                if matrix[i][k] + matrix[k][j] < matrix[i][j]:
                    matrix[i][j] = matrix[i][k] + matrix[k][j]
                    path[i][j] = path[k][j]

    return matrix, path

# def floyd(matrix, num_nodes):
#     # 初始化路径矩阵
#     path = np.full((num_nodes, num_nodes), -1, dtype=int)
#
#     for i in range(num_nodes):
#         for j in range(num_nodes):
#             if matrix[i][j] != np.inf and i != j:
#                 path[i][j] = i
#
#     for k in range(num_nodes):
#         if k > 0:
#             for i in range(num_nodes):
#                 for j in range(num_nodes):
#                     if matrix[i][k] + matrix[k][j] < matrix[i][j]:
#                         matrix[i][j] = matrix[i][k] + matrix[k][j]
#                         path[i][j] = path[k][j]
#
#     return matrix, path
def get_shortest_path(path, start, end, node_to_index, nodes):
    """
    核心：获取最短路径的原理其实很简单，基本都是从尾巴向头找，这是经常使用的方法，二维矩阵有点抽象；
    1.首先建立一个矩阵，这个矩阵将通路的位置赋值为横坐标，代表i，j之间暂时不知道是否有其他k使其更近；
    2.那么在更新矩阵的时候，如果找到k比i，j之间距离更近，就更新matrix，顺便更新path，将path横坐标的i换成k,想找到我(i)，得过了k那一关；
    3.在输出路径是否，通过从后往前找，比如起止点2->8之间有个6，就把6放到路径path中，设为终点继续寻找，2->6之间又有个4，把4放到path前面，
    4.就把4换成尾节点继续，直到2->4只有2，说明中间没有使距离更近的节点了，就返回path
    :param path:
    :param start:
    :param end:
    :param node_to_index:
    :param nodes:
    :return:
    """
    start_index = node_to_index[start]
    end_index = node_to_index[end]

    if path[start_index][end_index] == -1:
        return None

    path_indices = [end_index]
    while path[start_index][end_index] != start_index:
        end_index = path[start_index][end_index]
        path_indices.insert(0, end_index)

    return [nodes[i] for i in path_indices]

# def get_shortest_path(path, start, end, node_to_index, nodes):
#     start_index = node_to_index[start]
#     end_index = node_to_index[end]
#
#     if path[start_index][end_index] == -1:
#         return None  # 无路径
#
#     path_indices = [end_index]
#     # 假如path的点不等于横坐标，说明没有中间节点使得i->j更近
#     while path[start_index][end_index] != start_index:
#         end_index = path[start_index][end_index]
#         path_indices.insert(0, end_index)
#
#     # path_indices.reverse()
#     return [nodes[i] for i in path_indices]
#
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

# 转换为二维矩阵
matrix, nodes, num_nodes, node_to_index = graph_to_matrix(graph)
# 打印结果
print("节点顺序:", nodes)
print("二维矩阵:\n", matrix)
# # 计算最短路径矩阵和路径矩阵
matrix, path = floyd(matrix, num_nodes)
print("最短路径矩阵:\n", matrix)
print("路径矩阵：\n", path)
#
# # 输入起始节点和结束节点
start_node = 'P0'
end_node = 'P6'
#
# # 获取最短路径
shortest_path = get_shortest_path(path, start_node, end_node, node_to_index, nodes)
print(f"从 {start_node} 到 {end_node} 的最短路径: {shortest_path}")
