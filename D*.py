import heapq
import logging
import math
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# 配置日志
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
class Node:
    """
    Define node structure
    """
    def __init__(self, x, y, h=0.0, k=0.0, tag='NEW'):
        self.x = x  # 横坐标
        self.y = y  # 纵坐标
        self.h = h  # 当前节点到达目标点的成本
        self.k = k  # 当前节点到达目标点的最小成本
        self.tag = tag # NEW：新节点；OPEN：已经在OPEN_LIST中；CLOSE：探索过，不在OPEN_LIST中
        self.obstacle = False
        self.parent = None

    def __lt__(self, other):
        return self.h < other.h

class D_star:
    """
    D* algorithm
    """
    def __init__(self, graph, start, goal, robot_radius=0, obstacle_radius=0):
        """
        参数初始化
        :param graph: 地图
        :param start: 起始点
        :param goal: 目标点
        :param robot_radius: 机器人半径
        :param obstacle_radius: 障碍物半径
        """
        self.scope = graph
        self.graph = [[Node(x, y) for y in range(graph[1])] for x in range(graph[0])]
        self.start = start
        self.goal = goal
        self.directions = [(0,1),(-1,1),(-1,0),(-1,-1),(0,-1),(1,-1),(1,0),(1,1)]
        self.OPEN_LIST = [(0, (self.goal[0], self.goal[1]))]
        self.graph[self.goal[0]][self.goal[1]].tag = "OPEN"
        self.robot_radius = robot_radius
        self.obstacle_radius = obstacle_radius
        self.flag = False

    def set_obstacle(self, obstacles):
        """
        Set obstacle in the graph
        :return:
        """
        logging.info(f"$$$$$$$Start to set obstacle$$$$$$")
        for obstacle in obstacles:
            x = obstacle[0]
            y = obstacle[1]
            self.graph[x][y].h = float('inf')
            self.graph[x][y].k = float('inf')
            self.graph[x][y].tag = "CLOSE"
            self.graph[x][y].obstacle = True
            logging.info(f"{(x, y)} is set up to be obstacle.")
        logging.info(f"$$$$$$finish to set obstacle$$$$$$")

    def move_robot(self, add_obstacle):
        """
        Move robot from start to goal
        :return: path
        """
        if (self.start[0] < 0 or self.start[0] > self.scope[0] or
                self.start[1] < 0 or self.start[1] > self.scope[1]):
            logging.info(f"设置点在界外")
            return []
        val = 0
        # TODO: 处理起点终点存在障碍物特殊情况
        if (self.graph[self.goal[0]][self.goal[1]].obstacle == True
                or self.graph[self.start[0]][self.start[1]].obstacle == True):
            logging.info(f"起止点已经是障碍物，无法路径规划！")
            return []
        if self.OPEN_LIST:
            logging.info(f"OPEN_LIST: {self.OPEN_LIST}")
            k_min, node = heapq.heappop(self.OPEN_LIST)
            logging.info(f"{(k_min, node)} in the OPEN_LIST is pop")
            self.graph[node[0]][node[1]].tag = "CLOSE"
        else:
            # 处理堆为空的情况
            k_min, node = -1, None  # 或者其他默认值
        val = k_min
        # TODO：从目标开始第一次搜索路径，对图中的所有点遍历
        while self.graph[self.start[0]][self.start[1]].tag != "CLOSE" and val != -1:
            logging.info(f"OPEN_LIST: {self.OPEN_LIST}")
            val, node = self.process_state(val, node)

        # TODO 如果最后出发点的tag依然为NEW，说明被挡住了
        if self.graph[self.start[0]][self.start[1]].tag == "NEW":
            logging.info(f"出发点被挡住了，无法寻路！")
            return []

        R = self.graph[self.start[0]][self.start[1]]
        path = self.get_path(R, add_obstacle)
        return path

    def get_path(self, R, add_obstacle, times=0):
        """
        get path
        :return:
        """
        path = []
        # TODO: R点不是目标点就继续
        while R.x != self.goal[0] or R.y != self.goal[1]:
            if self.graph[self.start[0]][self.start[1]].tag == "NEW":
                logging.info(f"在R不断寻找父节点过程中，找不到目标点！")
                return []
            # TODO：设置障碍物
            if times == 1:
                if len(add_obstacle) > 0:
                    for each_obstacle_node in add_obstacle:
                        self.modify_cost(each_obstacle_node)
            if self.OPEN_LIST:
                logging.info(f"OPEN_LIST: {self.OPEN_LIST}")
                k_min, node = heapq.heappop(self.OPEN_LIST)
                logging.info(f"{(k_min, node)} in the OPEN_LIST is pop")
                self.graph[node[0]][node[1]].tag = "CLOSE"
            else:
                k_min, node = -1, None
            # TODO：OPEN_LIST存在结点，对出现的障碍物周围进行处理
            if k_min != -1 or node != None:
                val = k_min
                i = 0
                logging.info(f"开始新增障碍物的重新搜索")
                while val < R.h and val != -1:
                    logging.info(f"OPEN_LIST: {self.OPEN_LIST}")
                    val, node = self.process_state(val, node)
                    logging.info(f"第{i}次搜索，当前的X为{node}")
            path.append((R.x, R.y))
            logging.info(f"------research {times} round------")
            logging.info(f"Adding {(R.x, R.x)} to path.")
            # TODO：过程中R的父节点是障碍物就返回
            if R.parent.obstacle:
                logging.info(f"{R.x, R.y}前方被堵死,位置{(R.parent.x, R.parent.y)}存在障碍物")
                return []
            # TODO：回溯父节点
            R = R.parent
            times += 1
        path.append((self.goal[0], self.goal[1]))
        return path
    def modify_cost(self, each_obstacle_node):
        """
        modify cost and add to OPEN_LIST
        :param each_obstacle_node: Obstacle nodes
        :return: val
        """
        logging.info(f"******Start to modify cost******")
        x = each_obstacle_node[0]
        y = each_obstacle_node[1]
        obstacle_node = self.graph[x][y]
        h_old = obstacle_node.h
        k = h_old
        heapq.heappush(self.OPEN_LIST, (k, (x, y)))
        obstacle_node.h = float("inf")
        obstacle_node.tag = "OPEN"
        obstacle_node.obstacle = True
        logging.info(f"******finish to modify cost******")

    def process_state(self, kmin, node):
        """
        1.Search from goal to start
        2.Search from R existed cost(R, P(R))=inf ahead until lower state
        :return: k-min and its node from OPEN_LIST
        """
        logging.info(f"######Start process_state######")
        if node is None:
            return -1

        k_old = kmin
        neighbors = self.get_neighbor(node)
        logging.info(f"{node}'s neighbor is {neighbors}")
        # first: if there are a node that h=k in its right-upper direction, find a path directly.
        Node_X = self.graph[node[0]][node[1]]
        # TODO：该情况相当于在被障碍物影响的点周围找到一个点，使得当前点可以到达目标点，
        #      而且这个目标点的h小于当前点的历史最小h，表示Y到达目标点的距离比X更近
        # TODO：Node_X.h变大的原因有两个：
        #      1.X本身成为障碍物，但该条件仅满足X不是障碍物，才能计算h_y+h_yx
        #      2.Y是X的孩子，导致其h变为inf
        if k_old < Node_X.h and Node_X.obstacle == False:
            logging.info(f"第一种情况，直接修改X的父节点为Y")
            for neighbor in neighbors:
                Node_Y = self.graph[neighbor[0]][neighbor[1]]
                # logging.info(f"Node:{(node[0], node[1])}'s neighbor is {(neighbor[0], neighbor[1])}")
                hy_cost_xy = Node_Y.h + self.calculate_distance(neighbor, node)
                if Node_Y.h < k_old and Node_X.h > hy_cost_xy:
                    Node_X.parent = Node_Y
                    logging.info(f"{(Node_X.x, Node_X.y)}'s parent is set to {(Node_Y.x, Node_Y.y)}")
                    Node_X.h = hy_cost_xy
                    Node_X.k = hy_cost_xy
        # second:Lower state
        # TODO: 该情况属于当前的点非障碍物，也可以到达目标点
        #       对于此类情况，只需要判断3种条件：
        #       1.孩子是新节点，直接连接，反正当前X是OPEN_LIST中最小的；
        #       2.邻居Y是X的孩子，说明已经探索过，那就比较吧，只要经过X到达Y的代价跟Y的原有代价不一致，就把邻居放进OPEN_LIST
        #       3.邻居Y不是X孩子，是其他节点的，只要经过X到达Y的代价小于Y的原有代价，就把邻居放进OPEN_LIST
        if k_old == Node_X.h:
            logging.info(f"Go to Lower State")
            for neighbor in neighbors:
                Node_Y = self.graph[neighbor[0]][neighbor[1]]
                h_new = Node_X.h + self.calculate_distance(node, neighbor)
                if (Node_Y.tag == "NEW" or
                        (Node_Y.parent == Node_X and Node_Y.h != h_new) or
                        (Node_Y.parent != Node_X and Node_Y.h > h_new)):
                    Node_Y.parent = Node_X
                    logging.info(f"{(Node_Y.x, Node_Y.y)}'s parent is set to {(Node_X.x, Node_X.y)}")
                    Node_Y.h = h_new
                    k = h_new
                    Node_Y.k = k
                    heapq.heappush(self.OPEN_LIST, (k, neighbor))
                    Node_Y.tag = "OPEN"
                    logging.info(f"Neighbor:{(neighbor[0], neighbor[1])} is finished and added to OPEN_LIST")
        # third:Raise state
        # TODO：该情况属于有一部分点成为障碍物了，这块较复杂，对一个点障碍物周围的邻居做以下处理
        #       1.如果邻居Y是新节点，说明图搜索还没完成，或者Y是X孩子，或者Y到达目标点的代价与从X到达Y和代价不同，将Y放进OPEN_LIST，等待处理
        #       2.如果邻居Y不是X的孩子，从X到达Y的代价比Y小，X已经不在OPEN_LIST中，就把X放进OPEN_LIST，等待处理
        #       3.如果邻居Y不是X的孩子，但经过Y到达X的代价比X小，Y没有在OPEN_LIST中，但经过Y到达X的代价要大于原来X的最小代价，把Y放进OPEN_LIST
        else:
            logging.info(f"Go to Raise State")
            for neighbor in neighbors:
                Node_Y = self.graph[neighbor[0]][neighbor[1]]
                value = Node_X.h + self.calculate_distance(node, neighbor)
                if Node_Y.h == "inf" and value == "inf":
                    continue
                if Node_Y.tag == "NEW" or (Node_Y.parent == Node_X and Node_Y.h != value):
                    Node_Y.parent = Node_X
                    logging.info(f"{(Node_Y.x, Node_Y.y)}'s parent is set to {(Node_X.x, Node_X.y)}")
                    k = min(value, Node_Y.h)
                    Node_Y.h = value
                    Node_Y.k = k
                    heapq.heappush(self.OPEN_LIST, (k, neighbor))
                    Node_Y.tag = "OPEN"
                else:
                    if Node_Y.parent != Node_X and Node_Y.h > value and Node_X.tag == "CLOSE":
                        heapq.heappush(self.OPEN_LIST, (Node_X.h, node))
                        Node_X.tag = "OPEN"
                    else:
                        Y_X = Node_Y.h + self.calculate_distance(neighbor, node)
                        if Y_X == "inf" and Node_X.h == "inf":
                            continue
                        if Node_Y.parent != Node_X and Node_X.h > Y_X and Node_Y.tag == "CLOSE":# and Y_X > k_old
                            heapq.heappush(self.OPEN_LIST, (Node_Y.h, neighbor))
                            Node_Y.tag = "OPEN"
        if self.OPEN_LIST:
            logging.info(f"OPEN_LIST: {self.OPEN_LIST}")
            k_new_min, node_new = heapq.heappop(self.OPEN_LIST)
            logging.info(f"{(k_new_min, node_new)} in the OPEN_LIST is pop")
            self.graph[node_new[0]][node_new[1]].tag = "CLOSE"
            # logging.info(f"New k-min:{k_new_min} and its node:{(node_new.x, node_new.y)}")
        else:
            k_new_min, node_new = -1, None
        return k_new_min, node_new

    def calculate_distance(self, X, Y):
        """
        calculate distance from X to Y
        :return:
        """
        if self.graph[X[0]][X[1]].obstacle==True or self.graph[Y[0]][Y[1]].obstacle==True:
            return float("inf")
        return round(math.sqrt(abs(X[0]-Y[0]) ** 2 + abs(X[1]-Y[1]) ** 2), 2)

    def get_neighbor(self, node):
        """
        get neighbor around node
        :return:
        """
        logging.info(f"Start to get neighbor around node:{(node[0], node[1])}")
        result = []

        for direction in self.directions:
            x = node[0] + direction[0]
            y = node[1] + direction[1]
            if x < 0 or x >= self.scope[0] or y < 0 or y >= self.scope[1]:
                continue
            if self.is_in_obstacle((x, y)) is True:
                continue
            result.append((x, y))

        return result

    def is_in_obstacle(self, neighbor):
        """
        detect collision
        :param neighbor:邻居节点
        :return:
        """
        if self.graph[neighbor[0]][neighbor[1]].obstacle is True:
            return True
        return False

    @classmethod
    def plot_animation(cls, graph, start, goal, obstacle_list, add_obstacle, path, obstacle_radius,  robot_radius,save_file=None):
        """
        绘制路径动画
        :param graph: 地图大小
        :param start: 起点
        :param goal: 终点
        :param obstacle_list: 障碍物列表
        :param path: 路径
        :param robot_radius: 机器人半径
        """
        if len(path) == 0:
            return -1
        fig, ax = plt.subplots()
        ax.set_xlim(0, graph[0]-1)
        ax.set_ylim(0, graph[1]-1)
        ax.grid(True)

        # 设置坐标轴刻度间隔为1
        ax.set_xticks(range(0, graph[0]))
        ax.set_yticks(range(0, graph[1]))

        # 绘制障碍物
        for obs in obstacle_list:
            circle = plt.Circle((obs[0], obs[1]), obstacle_radius, color='black')  # 障碍物半径设为1
            ax.add_patch(circle)

        # 绘制起点和终点
        ax.scatter(start[0], start[1], color='green', marker='o', linewidths=1, label='Start')  # 起点用绿色圆形表示
        ax.scatter(goal[0], goal[1], color='red', marker='x', linewidths=1, label='Goal')  # 终点用红色叉号表示

        # 绘制路径
        path_x, path_y = zip(*path)
        line, = ax.plot(path_x, path_y, 'y-', marker = 'o',label='Path', linewidth=3, markersize=10)

        def update(frame):
            if frame < len(path):
                line.set_data(path_x[:frame + 1], path_y[:frame + 1])
                # logging.info(f"{(path_x[:frame + 1], path_y[:frame + 1])}")
                if path_x[1] == 0 or 1 and path_y[1] == 0 or 1:
                    if add_obstacle:
                        for obs in add_obstacle:
                            circle = plt.Circle((obs[0], obs[1]), obstacle_radius, color='red')  # 障碍物半径设为1
                            ax.add_patch(circle)
            else:
                # 质点到达终点，停止动画
                ani.event_source.stop()
                print("到达终点，重新生成路径...")
                # 这里可以添加生成新路径的逻辑
                # 例如：new_path = generate_new_path(start, goal, obstacle_list)
                # 然后重新调用 plot_animation 函数
            return line,

        ani = FuncAnimation(fig, update, frames=len(path), interval=200, blit=False)
        ax.legend()  # 添加图例
        # if save_file:
        ani.save(filename="add.png", writer='pillow')
        plt.show()

if __name__ == "__main__":
    graph = (10, 10)
    start = (0, 0)
    goal = (7, 1)
    robot_radius = 0.
    obstacle_radius = 0.7
    d = D_star(graph, start, goal, robot_radius, obstacle_radius)
    obstacles=[
        (2,3),
        (0,3),
        (1,3),
        (3,6),
        (4,1),(4,6),(4,7),
        (5,0),(5,3),(5,4),(5,5),(5,6),
        (6,3),(6,8),(6,9),
        (7,4),
        (8,5),
        (9,8),
        (4, 2), (7, 7), (8, 4)
    ]
    d.set_obstacle(obstacles)
    add_obstacle = [(2,6),(5,1),(5,2),(5,7),(7,3),(7,6),(7,8)]
    # add_obstacle=[]
    path = d.move_robot(add_obstacle)
    if path:
        logging.info(f"Path is found, which is {path}")
    d.plot_animation(graph, start, goal, obstacles, add_obstacle, path, obstacle_radius, robot_radius)

