import heapq
from heapq import heappush
from turtledemo.penrose import start


def dijkstra(graph, start, end):
    """

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
            next_disrance = weight + current_distance
            if flag[neighbor] is True:
                continue
            # 如果这个距离小于distances中记录的距离，说明这条路径更短
            if next_disrance < distances[neighbor]:
                # 更新distances
                distances[neighbor] = next_disrance
                # 将这个邻居节点和最短距离加入最小堆中
                heapq.heappush(priority_queue, (next_disrance, neighbor))
                # 记录该邻居节点的前驱节点为当前节点
                previous_node[neighbor] = current_node
    return []

def dijkstra_copy(graph, start_node, end_node):
    """

    :param graph:
    :param start_node:
    :param end_node:
    :return:
    """
    distances = {node: float('inf') for node in graph.keys()}
    distances[start_node] = 0
    priority_queue = [(0, start_node)]
    previous_node = {}
    
    while priority_queue:
        current_dis, current_node = heapq.heappop(priority_queue)

        if current_node is end_node:
            path = []
            while current_node:
                path.insert(0, current_node)
                current_node = previous_node[current_node] if current_node in previous_node else None
            return path

        if current_dis > distances[current_node]:
            continue

        for neighbor_node, neighbor_weight in graph[current_node].items():
            new_dis = neighbor_weight + current_dis
            if new_dis < distances[neighbor_node]:
                distances[neighbor_node] = new_dis
                heapq.heappush(priority_queue, (new_dis, neighbor_node))
                previous_node[neighbor_node] = current_node

    return []

graph = {
    'A': {'B': 1, 'C': 2},
    'B': {'A': 1, 'C': 2, 'D': 4},
    'C': {'A': 2, 'B': 2, 'E': 9, 'D': 1},
    'D': {'B': 4, 'C': 1, 'E': 1},
    'E': {'C': 9, 'D': 1}
}

start_node = 'B'
end_node = 'E'


# 调用 Dijkstra 算法
path1 = dijkstra(graph, start_node, end_node)
print(f"从 {start_node} 到 {end_node} 的最短路径为: {path1}")
path2 = dijkstra_copy(graph, start_node, end_node)
print(f"从 {start_node} 到 {end_node} 的最短路径为: {path2}")
