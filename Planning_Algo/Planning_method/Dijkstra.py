import os
import sys
import math
import heapq

sys.path.append(os.path.dirname(os.path.abspath(__file__)) +
                "/../../Planning_Algo/")

from Planning_method import env,plotting

class Dijikstra():
    
    def __init__(self, start_point, goal):
        self.start_point = start_point
        self.goal = goal
 
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

                # neighbor가 g에 없으면 
                if neighbor not in self.g:
                    self.g[neighbor] = math.inf
            
                # new_cost가 기존의 cost보다 적으면 update하고, parent node update
                if new_cost < self.g[neighbor]:
                    self.g[neighbor] = new_cost
                    self.PARENT[neighbor] = current_node

                    heapq.heappush(self.OPEN, (self.g[neighbor], neighbor))

        return self.extract_path(self.PARENT), self.CLOSED
    


    def get_neighbor(self, s):
        '''
        s : current node
        u_set : action set
        '''

        return [(s[0] + u[0], s[1] + u[1]) for u in self.u_set]

    def cost(self, start_point, goal):
        '''
        Cost 계산
        start_point = 시작노드
        goal = 끝 노드
        return = Cost from start_point to goal
        '''

        if self.is_collision(start_point, goal):
            return math.inf
        
        # math.hypot 함수는 삼각형의 빗변의 길이를 바로 return한다. == 두 점사이의 distance
        return math.hypot(goal[0] - start_point[0], goal[1] - start_point[1])

    def is_collision(self, start_point, goal):
        '''
        충돌 감지
        start_point = start node
        goal = end_node
        return : True (is collision) / False (not collistion) 
        '''

        # 시작 노드나 끝 노드가 obs에 있으면 return True
        if start_point in self.obs or goal in self.obs:
            return True
        
        # 노드 사이의 경로 중간에 장애물이 있나 확인
        if start_point[0] != goal[0] and start_point[1] != goal[1]:
            if goal[0] - start_point[0] == start_point[1] - goal[1]:
                s1 = (min(start_point[0], goal[0]), min(start_point[1], goal[1]))
                s2 = (max(start_point[0], goal[0]), max(start_point[1], goal[1]))
            else:
                s1 = (min(start_point[0], goal[0]), min(start_point[1], goal[1]))
                s2 = (max(start_point[0], goal[0]), max(start_point[1], goal[1]))
            
            if s1 in self.obs or s2 in self.obs:
                return True
        
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
        

def main():
    start_point = (5,5)
    goal = (45, 35)

    dijkstra = Dijikstra(start_point, goal)
    plot = plotting.Plotting(start_point, goal)

    path, visited = dijkstra.searching()
    plot.animation(path, visited, 'Dijkstra')    


if __name__ == "__main__":
    main()