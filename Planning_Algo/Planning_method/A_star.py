import os
import sys
import math
import heapq

sys.path.append(os.path.dirname(os.path.abspath(__file__)) +
                "/../../Planning_Algo/")

from Planning_method import env,plotting

class A_star():
    
    def __init__(self, start_point, goal, h_type):
        self.start_point = start_point
        self.goal = goal
        self.h_type = h_type
 
        self.Env = env.Env() # Env class 

        self.u_set = self.Env.motions # action set
        self.obs = self.Env.obs # obstacles

        self.OPEN = [] # priority queue (앞으로 방문할 노드)
        self.CLOSED = [] # visited node (방문 완료)
        self.PARENT = dict() # parent node
        self.g = dict() # cost to come

    def searching(self):
        self.PARENT[self.start_point] = self.start_point # parent node에 시작노드 설정
        self.g[self.start_point] = 0 # start point의 cost = 0
        self.g[self.goal] = math.inf # goal point의 cost = inf

        heapq.heappush(self.OPEN, (self.g[self.start_point], self.start_point)) # start point를 priority queue에 추가

        while self.OPEN:
            _, current_node = heapq.heappop(self.OPEN) # 가장 cost가 낮은 node 가져옴
            self.CLOSED.append(current_node) # 방문한 노드에 추가

            if current_node == self.goal: # 선택한 node가 goal이면 break
                break

            neighbors = self.get_neighbor(current_node) # 현재 node의 neighbor들을 가져옴

            for neighbor in neighbors:
                new_cost = self.g[current_node] + self.cost(current_node, neighbor) # 이웃 노드에 대한 새로운 비용 계산

                # neighbor not in g
                if neighbor not in self.g:
                    self.g[neighbor] = math.inf
                
                # new_cost < g[neighbor] -> update , parent node update
                if new_cost < self.g[neighbor]:
                    self.g[neighbor] = new_cost
                    self.PARENT[neighbor] = current_node

                    heapq.heappush(self.OPEN, (self.f_value(neighbor), neighbor))

        return self.extract_path(self.PARENT), self.CLOSED

    def get_neighbor(self, current_node):
        
        return [(current_node[0] + u[0], current_node[1] + u[1]) for u in self.u_set]

    def cost(self, start_point, goal):
        '''
        Cost function
        start_point : current node or start node
        goal : goal point
        return : cost between start_point and goal
        '''

        # start point와 goal point 사이에 충돌이 있으면 cost = inf
        if self.is_collision(start_point, goal):
            return math.inf
        
        # start point와 goal point 사이의 거리
        return math.hypot(goal[0]- start_point[0], goal[1]- start_point[1]) 

    def is_collision(self, start_point, goal):
        '''
        collision check
        start_point : current node or start node
        goal : goal point
        return : True or False
        '''

        # 시작 노드나 끝 노드가 obs에 있으면 return True
        if start_point in self.obs or goal in self.obs:
            return True
        
        # 시작 노드와 끝 노드 사이에 장애물이 있으면 return True
        if start_point[0] != goal[0] and start_point[1] != goal[1]:
            if goal[0] - start_point[0] == start_point[1] - goal[1]:
                s1 = (min(start_point[0], goal[0]), min(start_point[1], goal[1]))
                s2 = (max(start_point[0], goal[0]), max(start_point[1], goal[1]))
            else:
                s1 = (min(start_point[0], goal[0]), min(start_point[1], goal[1]))
                s2 = (max(start_point[0], goal[0]), max(start_point[1], goal[1]))
            
            if s1 in self.obs or s2 in self.obs:
                return True
        
        # 장애물이 없으면 return False
        return False
        

    def extract_path(self, PARENT):
        '''
        parent node를 사용하여 path 추출
        '''

        path = [self.goal]
        s = self.goal

        while True:
            s = PARENT[s]
            path.append(s)

            if s == self.start_point:
                break

        return list(path)

    def f_value(self, current_node):
        return self.g[current_node] + self.heuristic(current_node)

    def heuristic(self, current_node):
        '''
        select heristic function
        '''
        h_type = self.h_type
        goal = self.goal

        if h_type == 'manhattan':
            return abs(goal[0] - current_node[0]) + abs(goal[1] - current_node[1])
        else:
            return math.hypot(goal[0] - current_node[0], goal[1] - current_node[1])

def main():
    start_point = (5,5)
    goal = (45, 35)

    astar = A_star(start_point, goal, 'euclidean')
    plot = plotting.Plotting(start_point, goal)

    path, visited_nodes = astar.searching()
    plot.animation(path, visited_nodes, 'A_star')


if __name__ == '__main__':
    main()