# Autonomous Delivery Agent Demo (Refactored & Complete)
# ------------------------------------------------------
# - 2D grid with static and moving obstacles
# - Terrain costs ≥1
# - Movement: 4-connected (up/down/left/right)
# - Search: BFS, Dijkstra, A*, Local Search Repair
# - Dynamic replanning for moving obstacles
# - Structured output with best planner per map

import heapq
from collections import deque
import random
import time
import math
import pandas as pd
import copy

# ---------------- Grid ----------------
class GridWorld:
    def __init__(self, width, height, terrain=None, walls=None):
        self.width = width
        self.height = height
        self.terrain = [[1 for _ in range(width)] for _ in range(height)]
        if terrain:
            for (r, c), cost in terrain.items():
                self.terrain[r][c] = max(1, cost)
        self.walls = set(walls) if walls else set()

    def in_bounds(self, r, c):
        return 0 <= r < self.height and 0 <= c < self.width

    def passable(self, r, c):
        return (r, c) not in self.walls

    def neighbors(self, r, c):
        for dr, dc in [(0,1),(1,0),(0,-1),(-1,0)]:  # 4-connected
            nr, nc = r+dr, c+dc
            if self.in_bounds(nr,nc) and self.passable(nr,nc):
                yield (nr,nc)

    def cost(self, r, c):
        return self.terrain[r][c]

# ---------------- Moving Obstacles ----------------
class MovingObstacle:
    def __init__(self, path):
        self.path = path[:]

    def position_at(self, t):
        if not self.path: return None
        return self.path[t % len(self.path)]

def get_dynamic_positions(obstacles, horizon=200):
    positions = {}
    for t in range(horizon):
        for obs in obstacles:
            positions[t] = obs.position_at(t)
    return positions

# ---------------- Search Algorithms ----------------
class Stats:
    def __init__(self):
        self.expanded = 0

def bfs_search(grid, start, goal, dynamic_positions=None, start_time=0):
    stats = Stats()
    frontier = deque([start])
    came_from = {start: None}
    t = start_time
    while frontier:
        current = frontier.popleft()
        stats.expanded += 1
        if current == goal:
            break
        for n in grid.neighbors(*current):
            if n not in came_from:
                if dynamic_positions and dynamic_positions.get(t+1)==n:
                    continue
                came_from[n] = current
                frontier.append(n)
        t += 1
    if goal not in came_from:
        return None, math.inf, stats
    path = []
    node = goal
    while node:
        path.append(node)
        node = came_from[node]
    path.reverse()
    cost = sum(grid.cost(r,c) for r,c in path)
    return path, cost, stats

def uniform_cost_search(grid, start, goal, dynamic_positions=None, start_time=0):
    stats = Stats()
    frontier = [(0, start, start_time)]
    heapq.heapify(frontier)
    came_from = {start: None}
    cost_so_far = {start: 0}

    while frontier:
        current_cost, current, t = heapq.heappop(frontier)
        stats.expanded += 1
        if current == goal:
            break
        for n in grid.neighbors(*current):
            new_t = t+1
            if dynamic_positions and dynamic_positions.get(new_t) == n:
                continue
            new_cost = cost_so_far[current] + grid.cost(*n)
            if n not in cost_so_far or new_cost < cost_so_far[n]:
                cost_so_far[n] = new_cost
                came_from[n] = current
                heapq.heappush(frontier, (new_cost, n, new_t))

    if goal not in came_from:
        return None, math.inf, stats
    path = []
    node = goal
    while node:
        path.append(node)
        node = came_from[node]
    path.reverse()
    cost = sum(grid.cost(r,c) for r,c in path)
    return path, cost, stats

def manhattan(a, b):
    return abs(a[0]-b[0]) + abs(a[1]-b[1])

def a_star_search(grid, start, goal, dynamic_positions=None, start_time=0):
    stats = Stats()
    frontier = [(manhattan(start,goal), 0, start, start_time)]
    heapq.heapify(frontier)
    came_from = {start: None}
    cost_so_far = {start: 0}

    while frontier:
        f, g, current, t = heapq.heappop(frontier)
        stats.expanded += 1
        if current == goal:
            break
        for n in grid.neighbors(*current):
            new_t = t+1
            if dynamic_positions and dynamic_positions.get(new_t) == n:
                continue
            new_g = cost_so_far[current] + grid.cost(*n)
            if n not in cost_so_far or new_g < cost_so_far[n]:
                cost_so_far[n] = new_g
                came_from[n] = current
                heapq.heappush(frontier, (new_g + manhattan(n,goal), new_g, n, new_t))

    if goal not in came_from:
        return None, math.inf, stats
    path = []
    node = goal
    while node:
        path.append(node)
        node = came_from[node]
    path.reverse()
    cost = sum(grid.cost(r,c) for r,c in path)
    return path, cost, stats

# ---------------- Local Search Repair ----------------
# Modify local_search_repair definition:
def local_search_repair(grid, start, goal, dynamic_positions=None, start_time=0, max_iters=100, restarts=2):
    best_path, best_cost, initial_stats = a_star_search(grid, start, goal, dynamic_positions, start_time)
    total_expanded = initial_stats.expanded
    if best_path is None:
        return None, math.inf, Stats()

    for _ in range(restarts):
        current_path = best_path[:]
        current_cost = best_cost
        for _ in range(max_iters):
            if len(current_path) < 4:
                break
            i = random.randint(0, len(current_path)-3)
            j = random.randint(i+2, len(current_path)-1)
            subpath, subcost, sub_stats = a_star_search(grid, current_path[i], current_path[j], dynamic_positions, start_time)
            total_expanded += sub_stats.expanded
            if subpath:
                candidate = current_path[:i] + subpath + current_path[j+1:]
                candidate = [candidate[0]] + [p for p in candidate[1:] if p != candidate[-1]]
                cost_candidate = sum(grid.cost(r,c) for r,c in candidate)
                if cost_candidate < current_cost:
                    current_path = candidate
                    current_cost = cost_candidate
        if current_cost < best_cost:
            best_path, best_cost = current_path, current_cost

    stats = Stats()
    stats.expanded = total_expanded
    return best_path, best_cost, stats


# ---------------- Simulation ----------------
def simulate(grid, start, goal, obstacles, planner_fn, planner_name, dynamic=True):
    time_step = 0
    total_nodes = 0
    pos = start
    path_executed = []

    dynamic_positions = get_dynamic_positions(obstacles) if dynamic else None
    plan, cost, stats = planner_fn(grid, start, goal, dynamic_positions, start_time=time_step)
    total_nodes += stats.expanded

    if plan is None:
        return {"planner": planner_name, "path_cost": math.inf, "nodes_expanded": total_nodes, "success": False}

    idx = 1
    while pos != goal:
        time_step += 1
        occ = get_dynamic_positions(obstacles).get(time_step) if dynamic else None
        next_cell = plan[idx] if idx < len(plan) else None
        collision = (next_cell is not None and occ == next_cell) if dynamic else False

        if collision or next_cell is None:
            plan_new, cost_new, stats_new = planner_fn(grid, pos, goal,
                                                       dynamic_positions=get_dynamic_positions(obstacles) if dynamic else None,
                                                       start_time=time_step)
            total_nodes += stats_new.expanded
            if plan_new is None:
                return {"planner": planner_name, "path_cost": math.inf, "nodes_expanded": total_nodes, "success": False}
            plan = [pos] + plan_new[1:]
            idx = 1
            next_cell = plan[idx] if idx < len(plan) else None

        pos = next_cell
        path_executed.append(pos)
        idx += 1
        if len(path_executed) > grid.width * grid.height * 4:
            break

    total_cost = sum(grid.cost(r,c) for r,c in [start]+path_executed)
    return {"planner": planner_name, "path_cost": total_cost, "nodes_expanded": total_nodes, "success": pos==goal}

# ---------------- Example Maps ----------------
def create_map1():
    walls = {(3,i) for i in range(1,9)} | {(6,i) for i in range(2,9)}
    walls.remove((3,4))
    terrain = {(r,c):3 for r in range(0,4) for c in range(6,9)}
    return GridWorld(10,10,terrain=terrain,walls=walls)

def create_map2():
    walls = {(i,5) for i in range(2,10)} | {(8,i) for i in range(2,11)}
    walls.remove((2,5))
    terrain = {(r,c):4 for r in range(5,9) for c in range(0,4)}
    return GridWorld(12,12,terrain=terrain,walls=walls)

def create_map3():
    walls = {(2,i) for i in range(1,10)} | {(5,i) for i in range(4,12)}
    terrain = {(r,c):2 for r in range(0,3) for c in range(8,13)}
    return GridWorld(14,8,terrain=terrain,walls=walls)

def create_map4():
    walls = {(i,3) for i in range(1,7)} | {(i,7) for i in range(3,9)}
    terrain = {(r,c):5 for r in range(0,5) for c in range(5,9)}
    return GridWorld(10,10,terrain=terrain,walls=walls)

# ---------------- Moving Obstacle Examples ----------------
def moving_obstacles_case1():
    return [MovingObstacle([(3,1),(3,2),(3,3),(3,2)])]

def moving_obstacles_case2():
    return [MovingObstacle([(5,5),(5,6),(5,7),(5,6)]), MovingObstacle([(8,2),(8,3),(8,4),(8,3)])]

def moving_obstacles_case3():
    return [MovingObstacle([(2,2),(2,3),(2,4),(2,3)])]

def moving_obstacles_case4():
    return [MovingObstacle([(1,1),(2,1),(3,1),(2,1)])]

# ---------------- Run Experiments ----------------
def run_demo():
    random.seed(42)
    maps = [
        (create_map1(), (0,0), (9,9), moving_obstacles_case1()),
        (create_map2(), (0,0), (11,11), moving_obstacles_case2()),
        (create_map3(), (0,0), (7,13), moving_obstacles_case3()),
        (create_map4(), (0,0), (9,9), moving_obstacles_case4())
    ]

    planners = [
    (bfs_search, "BFS"),
    (uniform_cost_search, "Uniform-Cost Search"),
    (a_star_search, "A*"),
    (local_search_repair, "Local Repair")
    ]

    for idx, (grid,start,goal,obs) in enumerate(maps, start=1):
        print(f"\n=== MAP {idx}: Start {start} -> Goal {goal} ===")
        for dynamic in [False, True]:
            print(f"\n-- {'Dynamic' if dynamic else 'Static'} Obstacles --")
            results = []
            for fn,name in planners:
                gcopy = copy.deepcopy(grid)
                res = simulate(gcopy,start,goal,obs,fn,name,dynamic)
                results.append(res)
            df = pd.DataFrame(results)[["planner","path_cost","nodes_expanded","success"]]
            print(df.to_string(index=False))
            successful = [r for r in results if r["success"]]
            if successful:
                best = min(successful, key=lambda x: (x["path_cost"], x["nodes_expanded"]))

                print(f"\n[BEST] {best['planner']} with cost {best['path_cost']}")
            else:
                print("\nNo planner succeeded.")

if __name__=="__main__":
    run_demo()
