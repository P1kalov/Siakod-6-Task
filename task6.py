import random
from collections import deque

class MinCostMaxFlow:
    def __init__(self, n):
        self.n = n
        self.capacity = [[0] * n for _ in range(n)]
        self.cost = [[0] * n for a in range(n)]
        self.flow = [[0] * n for a in range(n)]
        self.graph = [[] for a in range(n)]

    def add_edge(self, u, v, cap, cost):
        self.graph[u].append(v)
        self.graph[v].append(u)
        self.capacity[u][v] = cap
        self.cost[u][v] = cost
        self.cost[v][u] = -cost

    def find_augmenting_path(self, source, sink):
        n = self.n
        parent = [-1] * n
        dist = [float('inf')] * n
        dist[source] = 0
        in_queue = [False] * n
        queue = deque([source])
        in_queue[source] = True

        while queue:
            u = queue.popleft()
            in_queue[u] = False
            for v in self.graph[u]:
                if self.capacity[u][v] - self.flow[u][v] > 0 and dist[v] > dist[u] + self.cost[u][v]:
                    dist[v] = dist[u] + self.cost[u][v]
                    parent[v] = u
                    if not in_queue[v]:
                        queue.append(v)
                        in_queue[v] = True

        if dist[sink] == float('inf'):
            return None

        path = []
        v = sink
        while v != source:
            u = parent[v]
            path.append((u, v))
            v = u
        path.reverse()
        return path

    def min_cost_max_flow(self, source, sink):
        flow = 0
        cost = 0
        while True:
            path = self.find_augmenting_path(source, sink)
            if not path:
                break

            increment = float('inf')
            for u, v in path:
                increment = min(increment, self.capacity[u][v] - self.flow[u][v])

            for u, v in path:
                self.flow[u][v] += increment
                self.flow[v][u] -= increment
                cost += increment * self.cost[u][v]

            flow += increment

        return flow, cost

def generate_assignment_problem(n):
    return [[random.randint(1, 100) for i in range(n)] for i in range(n)]

def assignment_to_flow(matrix):
    n = len(matrix)
    size = 2 * n + 2
    source = 2 * n
    sink = 2 * n + 1
    flow_network = MinCostMaxFlow(size)

    for i in range(n):
        flow_network.add_edge(source, i, 1, 0)
        flow_network.add_edge(n + i, sink, 1, 0)
        for j in range(n):
            flow_network.add_edge(i, n + j, 1, matrix[i][j])

    return flow_network, source, sink

n = 4
cost_matrix = generate_assignment_problem(n)
print("стоимость матрицы:")
for row in cost_matrix:
    print(row)

flow_network, source, sink = assignment_to_flow(cost_matrix)
flow, cost = flow_network.min_cost_max_flow(source, sink)

print(f"\n максимальный поток: {flow}")
print(f"минимальная стоимость: {cost}")
