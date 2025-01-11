import math
import heapq
import numpy as np
import matplotlib.pyplot as plt
import logging

# 配置日志
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')

class Node:
    """
    定义节点，包括坐标以及父节点
    """
    def __init__(self, x, y, record, cost=0):
        self.x = float(x)
        self.y = float(y)
        self.record = record
        self.parent = None
        self.cost = cost    # 该成本为当前节点到达出发点的成本

class RRT_STAR:
    """
    RRT*算法
    """
    def __init__(self, start, goal, obstacles, step_size=0.5):
        self.start = start  # 出发点
        self.goal = goal    # 目的地
        self.obstacles = obstacles  # 障碍物
        self.step_size = step_size  # 步长
        self.max_iter = 600    # 最大生成试探次数
        self.sample_circle = 2 * step_size

    def rrt_search(self):
        """
        RRT*搜索
        :return: 路径和坐标列表
        """
        T = [self.start]    # 初始化节点T集合
        plt.figure(figsize=(8, 8))
        plt.xlim(map[0], map[1])
        plt.ylim(map[0], map[1])

        # 绘制障碍物
        for obstacle in self.obstacles:
            circle = plt.Circle((obstacle[0], obstacle[1]), obstacle[2], color='black', fill=True)
            plt.gca().add_patch(circle)

        # 绘制起始点和目标点
        plt.plot(self.start.x, self.start.y, 'go', markersize=10, label='Start')
        plt.plot(self.goal.x, self.goal.y, 'ro', markersize=10, label='Goal')

        for i in range(self.max_iter):
            logging.info(f"Iteration {i + 1}/{self.max_iter}")
            # 在map中随机生成一个点
            node_random = Node(round(np.random.uniform(map[0], map[1]), 2), round(np.random.uniform(map[0], map[1]), 2), record=i+1)
            logging.debug(f"Random node generated: ({node_random.x}, {node_random.y})")
            # 在节点集合T中寻找最近的点
            nearest_node = self.get_nearest_node(T, node_random)
            logging.debug(f"Nearest node found: ({nearest_node.x}, {nearest_node.y})")
            # 找到之后在两点之间生成新的节点
            new_node = self.get_new_node(nearest_node, node_random, i)
            logging.debug(f"New node generated: ({new_node.x}, {new_node.y})")
            # 检测碰撞，没有碰撞就加入到T中
            if not self.is_in_collision(new_node):
                # 在new_node周围一段距离内做两个操作，第一个重新选择父节点
                new_node = self.rechoose_parent(new_node, T)
                T = self.re_construct_tree(new_node, T)
                T.append(new_node)
                logging.info(f"New node added to the tree: ({new_node.x}, {new_node.y})")
                # 动态绘制生成的点和线
                plt.plot([nearest_node.x, new_node.x], [nearest_node.y, new_node.y], 'k-')
                plt.scatter(new_node.x, new_node.y, color='blue', s=10)
                # 添加节点序号
                plt.text(new_node.x, new_node.y, str(i + 1), fontsize=8, ha='right', va='bottom', color='black')
                plt.pause(0.01)  # 暂停一小段时间以更新图像
                # 在没有碰撞下，且距离目标点小于步长，就停止搜索，输出路径
                if self.in_near_goal(new_node):
                    # 在节点集合T中寻找最近的点
                    self.goal = self.rechoose_parent(self.goal, T)
                    # T = self.re_construct_tree(self.goal, T)
                    logging.info("Goal reached!")
                    path = []
                    cood = []
                    while new_node.parent:
                        path.insert(0, new_node)
                        cood.insert(0, (new_node.x, new_node.y))
                        new_node = new_node.parent
                    path.insert(0, new_node)
                    path.append(self.goal)
                    cood.insert(0, (self.start.x, self.start.y))
                    cood.append((self.goal.x, self.goal.y))
                    # 绘制最终路径
                    for i in range(len(path) - 1):
                        plt.plot([path[i].x, path[i + 1].x], [path[i].y, path[i + 1].y], 'r-', linewidth=3)
                    plt.title('RRT* Path Planning - Final Path Found')
                    plt.legend()
                    plt.show()
                    return path, cood
            else:
                logging.debug(f"Collision detected at node: ({new_node.x}, {new_node.y})")

        plt.title('RRT* Path Planning - No Path Found')
        plt.legend()
        plt.show()
        return [], []

    def re_construct_tree(self, new_node, T):
        """
        重新在new_node(父节点)周围排列节点
        :param new_node:
        :param T:
        :return:
        """
        for node_ready_bechild in T:
            if node_ready_bechild is new_node or node_ready_bechild is new_node.parent:
                continue
            dd = self.calculate_A_to_B(node_ready_bechild, new_node)
            if dd <= self.sample_circle:
                dis_be_child_cost = new_node.cost + dd
                if dis_be_child_cost < node_ready_bechild.cost:
                    node_ready_bechild.parent = new_node
                    node_ready_bechild.cost = dis_be_child_cost
                    logging.info(f"------现在启动第二次父节点更换，重新构建子树------")
                    logging.info(f"{node_ready_bechild.record}的父节点从{node_ready_bechild.parent.record}换成{new_node.record}")
                    logging.info(f"实际上的父节点是{new_node.parent.record}")
        return T

    def calculate_A_to_B(self, node_A, node_B):
        """
        计算节点间距离
        :param node_A:
        :param node_B:
        :return:
        """
        if not isinstance(node_A, Node) or not isinstance(node_B, Node):
            logging.error(f"输入不符合Node定义")
            return float('inf')
        return math.sqrt((node_A.x - node_B.x) ** 2 + (node_A.y - node_B.y) ** 2)

    def rechoose_parent(self, new_node, T):
        """
        重新选择父节点
        :param new_node:
        :param T:
        :return: new_node
        """
        pre = new_node.parent
        min_cost = new_node.cost
        for node_ready_beparent in T:
            if node_ready_beparent is new_node or node_ready_beparent is new_node.parent:
                continue
            dd = self.calculate_A_to_B(node_ready_beparent, new_node)
            if dd <= self.sample_circle:
                dis_be_parent_cost = node_ready_beparent.cost + dd
                if dis_be_parent_cost < min_cost:
                    new_node.parent = node_ready_beparent
                    min_cost = dis_be_parent_cost
                    new_node.cost = min_cost
                    logging.info(f"------现在启动第一次父节点更换，寻找新的父亲------")
                    logging.info(f"{new_node.record}的父节点从{pre.record}变为{node_ready_beparent.record}")
                    logging.info(f"实际上父节点是{new_node.parent.record}")
        return new_node

    def in_near_goal(self, new_node):
        """
        在目标点附近
        :param new_node:
        :return:
        """
        return math.hypot(new_node.x - self.goal.x, new_node.y - self.goal.y) <= self.step_size

    def is_in_collision(self, new_node):
        """
        在nearest节点与new节点的连接线上判断是否在障碍物内
        :param new_node:
        :return:
        """
        for obstacle in self.obstacles:
            dd = math.sqrt((new_node.x - obstacle[0]) ** 2 + (new_node.y - obstacle[1]) ** 2)
            if dd < obstacle[2]:
                return True
        return False

    def get_new_node(self, near_node, rand_node, i):
        """
        在Node_near和Node_rand之间寻找New的节点，使得在new与near之间生成一条路径
        :param near_node:
        :param rand_node:
        :param i:
        :return:
        """
        theta = math.atan2(rand_node.y - near_node.y, rand_node.x - near_node.x)
        new_node_x = near_node.x + self.step_size * math.cos(theta)
        new_node_y = near_node.y + self.step_size * math.sin(theta)
        new_node = Node(round(new_node_x, 2), round(new_node_y, 2), record=i+1)
        new_node.parent = near_node
        new_node.cost = near_node.cost + self.calculate_A_to_B(near_node, new_node)
        return new_node

    def get_nearest_node(self, T, node_rand):
        """
        获取最近的节点以及距离
        :param T:
        :param node_rand:
        :return:
        """
        nearest_node = min(T, key=lambda node: self.calculate_A_to_B(node, node_rand))
        return nearest_node

if __name__ == '__main__':
    map = (0, 10)
    start = Node(1., 9., 0, 0)
    goal = Node(6., 2., -1, 0)
    obstacles = [(2., 3., 2), (4., 5., 2), (7., 7., 2), (4., 4., 1)]
    rrt = RRT_STAR(start, goal, obstacles)
    path, cood = rrt.rrt_search()
    logging.info(f"从出发点{(start.x, start.y)}到目标点{(goal.x, goal.y)}的路径是{cood}")
