Autonomous Delivery Agent Demo

This project demonstrates an autonomous delivery agent navigating a 2D grid world with static and moving obstacles, variable terrain costs, and dynamic replanning capabilities. It implements multiple pathfinding algorithms and compares their performance on several predefined grid maps.

Features
Grid World: 2D grid with customizable width, height, terrain costs (≥1), and walls.

Movement: 4-connected grid movement — up, down, left, right.

Obstacles:

Static obstacles (walls).

Moving obstacles with cyclic predefined paths.

Terrain Costs: Each cell can have different traversal costs, influencing pathfinding.

Pathfinding Algorithms:

Breadth-First Search (BFS)

Uniform-Cost Search (Dijkstra's algorithm)

A* Search (with Manhattan distance heuristic)

Local Search Repair (heuristic path improvement using repeated A* subplanning)

Dynamic Replanning: Agent replans paths dynamically to avoid collisions with moving obstacles.

Simulation: Executes plans step-by-step, simulates obstacle movements, and triggers replanning when needed.

Performance Metrics: Reports path cost, nodes expanded, and success status for each planner.

Multiple Map Scenarios: Four diverse maps with different obstacle layouts and terrain.

Usage
Run the demo with Python 3:

bash
python autonomous_delivery_agent.py
The demo runs four different grid maps, testing all planners on each with both static and dynamic obstacles, printing results in a structured table for comparison. It also identifies the best performing planner per map scenario.

Code Structure
GridWorld
Represents the 2D environment.

Handles terrain costs and walls.

Provides neighbor queries and cost lookups.

MovingObstacle
Defines cyclic movement paths for dynamic obstacles.

Positions update with time steps, used for collision checking.

Search Algorithms
bfs_search: Simple breadth-first search ignoring costs.

uniform_cost_search: Dijkstra's algorithm handling terrain costs.

a_star_search: Heuristic search with Manhattan distance.

local_search_repair: Improves initial A* solution via local repair using repeated sub-A* calls.

Simulation
Runs the agent through the grid step-by-step.

If collision with moving obstacles is imminent, triggers replanning.

Keeps track of total nodes expanded and path cost.

Maps and Moving Obstacles
Four example maps are created with unique wall and terrain layouts.

Each map has associated moving obstacle scenarios to test dynamic planning.

Dependencies
Python 3.x

pandas library (for tabulated output display)

Standard libraries: heapq, collections, random, math, copy, time

Install pandas if needed:

bash
pip install pandas
Insights
BFS works for uniform-cost grids with no heavy terrain but ignores path cost.

Uniform-cost search and A* handle terrain costs effectively.

Local search repair can improve A* solutions heuristically.

Dynamic replanning is essential in environments with moving obstacles.

Performance varies by map complexity and obstacle movement patterns.

License
This code is provided as-is for educational and demonstration purposes.
