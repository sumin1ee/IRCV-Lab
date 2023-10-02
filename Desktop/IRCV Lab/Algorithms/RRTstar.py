import numpy as np
import matplotlib.pyplot as plt

class Node:
    def __init__(self, x, y):
        self.x = x  # 노드의 x 좌표
        self.y = y  # 노드의 y 좌표
        self.cost = 0.0
        self.parent = None  # 부모 노드, 초기값은 None

class RRTStar:

    def __init__(self, start, goal, obstacle_list, rand_area, max_iter, expand_dis, goal_sample_rate):
        self.start = Node(start[0], start[1])
        self.goal = Node(goal[0], goal[1])
        self.obstacle_list = obstacle_list
        self.min_rand = rand_area[0]
        self.max_rand = rand_area[1]
        self.max_iter = max_iter
        self.expand_dis = expand_dis
        self.goal_sample_rate = goal_sample_rate

    def planning(self):
        self.node_list = [self.start]
        for _ in range(self.max_iter):
            if np.random.rand() > self.goal_sample_rate: # goal_sample_rate 의 확률 p의 초과의 수가 뽑혔을 때 -> 1 - p 의 확률
                rand_node = Node(
                    np.random.uniform(self.min_rand, self.max_rand),
                    np.random.uniform(self.min_rand, self.max_rand))
            else: # goal_sample_rate 의 확률 p의 수 이하의 수가 뽑혔을 때 -> p의 확률
                rand_node = Node(self.goal.x, self.goal.y)

            #rand_node와 가장 가까운 노드를 찾아서 인덱스 반환 후 리스트에 넣는다.
            nearest_ind = self.get_nearest_node_index(rand_node)
            nearest_node = self.node_list[nearest_ind]
            new_node = self.steer(nearest_node, rand_node, self.expand_dis)
            print("nearest_node : ", nearest_node.x , nearest_node.y)
            print("rand_node : ", rand_node.x, rand_node.y)
            print("new_node = ", new_node.x, new_node.y)
            if not self.check_collision(new_node, self.obstacle_list): # 충돌 체크
                near_indices = self.find_near_nodes(new_node)
                new_node = self.choose_parent(new_node, near_indices)
                if new_node:
                    self.node_list.append(new_node)
                    self.rewire(new_node, near_indices)
            
            if self.is_goal_reached(new_node, self.goal):
                return self.generate_final_course(len(self.node_list) - 1)

        return None

    # 리스트에서 가장 가까운 노드의 인덱스 찾기t
    def get_nearest_node_index(self, rand_node):
        dlist = [(node.x - rand_node.x) ** 2 + (node.y - rand_node.y) ** 2 for node in self.node_list]
        min_index = dlist.index(min(dlist))
        return min_index

    # 가장 가까운 노드에서 새로운 노드로 확장
    def steer(self, from_node, to_node, extend_length):
        new_node = Node(from_node.x, from_node.y)
        d, theta = self.calc_distance_and_angle(new_node, to_node)

        if extend_length >= d:
            extend_length = d

        new_node.x += extend_length * np.cos(theta)
        new_node.y += extend_length * np.sin(theta)

        new_node.parent = from_node
        return new_node

    # 두 노드 사이의 거리와 각도 계산
    def calc_distance_and_angle(self, from_node, to_node):
        if from_node is None or to_node is None:
            return float("inf"), 0.0 # 노드가 둘 중 하나라도 None일 경우 무한대와 각도 0 반환
        dx = to_node.x - from_node.x
        dy = to_node.y - from_node.y
        d = np.hypot(dx, dy)
        theta = np.arctan2(dy, dx)
        return d, theta

    # 충돌 검사
    def check_collision(self, node, obstacle_list):
        for (ox, oy, size) in obstacle_list:
            dx = ox - node.x
            dy = oy - node.y
            d = np.hypot(dx, dy)
            if d <= size:
                return True  # 충돌
        return False  # 충돌하지 않음

    # 노드 주변의 인덱스를 찾음
    def find_near_nodes(self, new_node):
        nnode = len(self.node_list)
        r = 0.5  * np.sqrt((np.log(nnode + 1) / nnode)) # 실험(경험)적으로 결정한 r값. 주변 노드를 고를 반경을 의미함
        dlist = [(node.x - new_node.x) ** 2 + (node.y - new_node.y) ** 2 for node in self.node_list]
        near_indices = [i for i in range(nnode) if dlist[i] <= r ** 2]
        return near_indices

    # 최적의 부모 노드를 선택
    def choose_parent(self, new_node, near_indices):
        if not near_indices:
            return None

        dlist = []
        for i in near_indices:
            near_node = self.node_list[i]
            if not self.check_collision(near_node, obstacle_list):
                d, _ = self.calc_distance_and_angle(near_node, new_node)
                dlist.append(near_node.cost + d)
            else:
                dlist.append(float("inf"))

        min_cost = min(dlist)
        if min_cost == float("inf"):
            return None

        min_ind = near_indices[dlist.index(min_cost)]
        new_node = self.steer(self.node_list[min_ind], new_node, self.expand_dis)
        new_node.cost = min_cost

        return new_node

    # 노드의 부모를 변경하고 자식 노드를 재귀적으로 변경
    def rewire(self, new_node, near_indices):
        nnode = len(self.node_list)
        for i in near_indices:   
            near_node = self.node_list[i]
            d, _ = self.calc_distance_and_angle(new_node, near_node)
            scost = new_node.cost + d

            if near_node.cost > scost and self.check_collision(near_node, self.obstacle_list):
                near_node.parent = new_node
                near_node.cost = scost

                self.rewire(near_node, self.find_near_nodes(near_node))

    # 목표 지점에 도달했는지 확인
    def is_goal_reached(self, node, goal):
        d, _ = self.calc_distance_and_angle(node, goal)
        return d <= self.expand_dis

    # 최종 경로 생성
    def generate_final_course(self, goal_ind):
        path = [(self.goal.x, self.goal.y)]
        while self.node_list[goal_ind].parent is not None:
            node = self.node_list[goal_ind]
            path.append((node.x, node.y))
            goal_ind = self.node_list.index(node.parent)
        path.append((self.start.x, self.start.y))
        return path


# 경로의 장애물 정보와 시작점, 목표점 설정
obstacle_list = [(0.5, 0.5, 2.0), (1.5, 0.5, 1.5), (2.5, 0.5, 0.8), (4.5, 0.5, 0.6), (4.5, 1.5, 1.2), (2.5, 2.5, 0.8)]  # 장애물 정보: (x, y, radius)
start_point = (0, 4)  # 시작점 좌표
goal_point = (3.5, 0.5)   # 목표점 좌표

# RRT* 알고리즘 설정
rrt_star = RRTStar(
    start = start_point,
    goal = goal_point,
    obstacle_list = obstacle_list,
    rand_area = (0, 6), # 0부터 6사이의 수를 뽑음
    max_iter = 30000, # 최대 3만번 반복
    expand_dis = 0.02, # 노드 사이의 최대 거리는 0.05
    goal_sample_rate = 0.85  # 55%의 확률로 목표 지점을 선택하도록 설정
)

# 경로 계획 수행
path = rrt_star.planning()

# 경로 시각화
if path is not None:
    path_x = [node[0] for node in path]
    path_y = [node[1] for node in path]

    plt.figure(figsize=(8, 8))
    plt.plot(path_x, path_y, '-r', linewidth=2)
    plt.plot(start_point[0], start_point[1], 'og', markersize=10)
    plt.plot(goal_point[0], goal_point[1], 'ob', markersize=10)
    for (ox, oy, size) in obstacle_list:
        circle = plt.Circle((ox, oy), size, color='gray')
        plt.gca().add_patch(circle)

    plt.xlabel('X')
    plt.ylabel('Y')
    plt.grid(True)
    plt.axis('equal')
    plt.show()
else:
    print("경로를 찾지 못했습니다.")