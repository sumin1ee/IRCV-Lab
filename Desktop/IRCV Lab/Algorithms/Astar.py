import math
import heapq

# 유클리드 거리를 구하는 함수
def get_euclidean_distance(point1, point2):
    x1, y1 = point1
    x2, y2 = point2
    return math.sqrt((x1 - x2)**2 + (y1 - y2)**2)

# BFS 를 위해 사용할 dx, dy 리스트 정의 (우 하 좌 상 시계방향으로 확인)
dx_list = [1, 0, -1, 0]
dy_list = [0, -1, 0, 1]

def a_star(array, start, destination):
    # 무한대 정의 및 전역변수 불러오기
    INF = float("inf")
    global dx_list
    global dy_list
    
    height = len(road_list)
    width = len(road_list[0])

    # 반환할 Heuristic Cost 를 INF로 정렬
    heuristic_cost = [[INF for _ in range(width)] for _ in range(height)]

    # 갈 수 있는 길의 heuristic_cost 계산하고 행렬에 넣기
    for row in range(height):
        for col in range(width):
            if array[row][col]:
                heuristic_cost[row][col] = round(get_euclidean_distance((row, col), destination))
    
    #start, dest 좌표, 총 비용, camefrom_list, vis_list 초기화 및 지정
    row, col = start
    row_dest , col_dest = destination
    total_cost = 0
    camefrom_list = []
    vis_list = [[False for _ in range(width)] for _ in range(height)]

    heap = []
    heapq.heappush(heap, (heuristic_cost[row][col], row, col))
    
    while len(heap) and (row, col) != (row_dest, col_dest):
        total_cost, row, col = heapq.heappop(heap)
        
        #f(n) = g(n) + h(n)  에서, g(n)을 구하기 위해 h(n) 을 빼준다.
        depth = total_cost - heuristic_cost[row][col]

        #방문 처리
        vis_list[row][col] = True

        for i in range(4):
            nx = col + dx_list[i]
            ny = row + dy_list[i]
            if nx < 0 or nx >= len(array[0]) or ny < 0 or ny >= len(array): continue
            if vis_list[ny][nx] or (not road_list[ny][nx] == 1): continue
            total_cost = heuristic_cost[ny][nx] + depth + 1
            camefrom_list.append(((row, col), (ny, nx)))
            heapq.heappush(heap, (total_cost, ny, nx))
    
    from_y, from_x = camefrom_list[-1][0]
    paths = []

    for i in range(len(camefrom_list) - 1 , -1, -1):
        from_coord, to_coord = camefrom_list[i]
        to_y, to_x = to_coord

        if from_y == to_y and from_x == to_x:
            from_y, from_x = from_coord
            paths.insert(0, to_coord)

    return total_cost, paths, vis_list, heuristic_cost

def print_cost(cost):
    h = len(cost)
    w = len(cost[0])

    print(" - Heuristic Cost - ")
    for i in range(h):
        for j in range(w):
            print("." if math.isinf(cost[i][j]) else cost[i][j], end=" ")
        print()
    print()

def print_path(path, start, destination):
    h = len(path)
    w = len(path[0])

    print(" - Paths - ")
    for i in range(h):
        for j in range(w):
            if (i, j) == start:
                print("S", end=" ")
            elif (i, j) == destination:
                print("G", end=" ")
            else:
                print("0" if path[i][j] else ".", end=" ")
        print()
    print()

def print_shortest_distance(start, destination, total_cost):
    print(start , "->", destination)
    print("최단 거리 : ", total_cost)
    print()

def print_shortest_path(road, paths, start, destination):
    h = len(road)
    w = len(road[0])
    matrix = [["." for _ in range(w)] for _ in range(h)]

    for i in range(h):
        for j in range(w):
            if (i, j) == start:
                matrix[i][j] = "S"
    prev_y, prev_x = start
    for path in paths:
        cur_y, cur_x = path
        if prev_y < cur_y:
            matrix[cur_y][cur_x] = "↓"
        elif prev_y > cur_y:
            matrix[cur_y][cur_x] = "↑"
        elif prev_x < cur_x:
            matrix[cur_y][cur_x] = "→"
        elif prev_x > cur_x:
            matrix[cur_y][cur_x] = "←"
        prev_y, prev_x = cur_y, cur_x
    
    matrix[destination[0]][destination[1]] = "G"

    print("-- Shortest Path --")
    for i in range(h):
        for j in range(w):
            print(matrix[i][j], end=" ")
        print()
    print()




'''#길 리스트
road_list = [
    [True, True, True, False, False, False, False],
    [True, False, True, False, False, False, False],
    [True, False, True, True, True, True, True],
    [True, False, True, False, False, False, True],
    [True, False, True, False, True, True, True],
    [True, False, True, False, True, False, False],
    [True, True, True, True, True, True, True],
]'''

road_list = [
    [False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False],
    [False, True,  True,  False,  False,  False,  True,  False,  True,  True,  True,  True,  True,  True,  True,  True,  True,  True,  True,  False],
    [False, True,  True,  False,  True,  False,  True,  False,  False,  False,  False,  False,  False,  False,  False,  False,  False,  False,  True,  False],
    [False, True,  True,  False,  True,  False,  True,  True,  True,  True,  True,  True,  True,  False,  True,  True,  True,  True,  True,  False],
    [False, True,  False,  False,  True,  False,  True,  False,  False,  False,  False,  False,  True,  False,  True,  False,  False,  False,  False,  False],
    [False, True,  True,  True,  True,  True,  True,  True,  True,  True,  True,  True,  True,  False,  True,  False,  True,  True,  True,  False],
    [False, True,  False,  False,  False,  False,  True,  False,  False,  False,  False,  False,  True,  False,  True,  False,  False,  False,  False,  False],
    [False, True,  True,  True,  True,  False,  True,  True,  True,  True,  True,  True,  True,  False,  True,  False,  True,  True,  True,  False],
    [False, False, False, False, False, False, False, False, False, False, False, False,  True,  False,  True,  False,  False,  False,  False,  False],
    [False,  True,  True,  True,  True,  True,  True,  True,  True,  True,  True,  True,  True,  False,  True,  True,  True,  True,  True,  False],
    [False,  True,  False,  False,  False,  False,  True,  False,  False,  False,  False,  False,  True,  False,  True,  False,  False,  False,  False,  False],
    [False,  True,  True,  True,  True,  True,  True,  True,  True,  True,  True,  True,  True,  True,  True,  True,  True,  True,  True,  False],
    [False,  True,  False,  False,  False,  False,  False,  False,  False,  False,  False,  False,  False,  False,  False,  False,  False,  False,  True,  False],
    [False,  True,  True,  True,  True,  True,  True,  True,  True,  True,  True,  True,  True,  True,  True,  True,  True,  True,  True,  False],
    [False,  True,  False,  False,  False,  False,  False,  False,  False,  False,  False,  False,  False,  False,  False,  False,  False,  False,  False,  False],
    [False,  True,  True,  True,  True,  True,  True,  True,  True,  True,  True,  True,  True,  True,  True,  True,  True,  True,  True,  False],
    [False,  True,  False,  False,  False,  False,  False,  False,  False,  False,  False,  False,  False,  False,  False,  False,  False,  False,  False,  False],
    [False,  True,  True,  True,  True,  True,  True,  True,  True,  True,  True,  True,  True,  True,  True,  True,  True,  True,  True,  False],
    [False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False]
]

#시작점과 도착점
start = (1, 1)
destination = (17, 18)

total_cost, paths, visit_list, heuristic_cost = a_star(road_list, start, destination)

print_path(road_list, start, destination)
print_cost(heuristic_cost)
print_shortest_distance(start, destination, total_cost)
print_shortest_path(road_list, paths, start, destination)






