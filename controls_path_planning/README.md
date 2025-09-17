# Path Planning Writeup

#### Note
I had issues displaying a plot of my paths and outputting a results_fig.png file with my paths when running `test_planner.py`, so unfortunately I only have `results.yaml` as output.
#### Introduction
This writeup documents my solution to the Risk-Aware Planning coding challenge. 
#### Initial Approach
My first idea after reading the problem statement was to apply breadth-first search (BFS). For each goal, I would run BFS starting from the initial position until reaching the goal. BFS guarantees the path with the fewest steps, so I was confident it would produce valid solutions. Since I was already familiar with BFS, I implemented it quickly.
```# BFS solution: not the most optimal, but always finds a valid path
def bfs(self, start: Coordinate, goal: Coordinate):
    q = deque([start])
    # For each coordinate, store the previous coordinate from which it was reached
    came_from = {start: start}

    while q:
        curr = q.popleft()
        if curr == goal:
            break

        for neighbor in self.neighbors(curr):
            if neighbor not in came_from:
                came_from[neighbor] = curr
                q.append(neighbor)

    if goal not in came_from:
        return []

    # Reconstruct path by backtracking from goal
    path = []
    curr = goal
    while curr != start:
        path.append(curr)
        curr = came_from[curr]
    path.append(start)
    path.reverse()
    return path

```
This method successfully reached all 10/10 sites:
```10/10 sites reached validly
    Flamboyance: (valid:  1, risk: 0, length: 18.07)
           Knot: (valid:  1, risk: 27, length: 50.00)
        Chatter: (valid:  1, risk: 5, length: 12.49)
         Deceit: (valid:  1, risk: 27, length: 38.24)
       Conclave: (valid:  1, risk: 22, length: 29.66)
         Mewing: (valid:  1, risk: 6, length: 13.66)
     Coronation: (valid:  1, risk: 28, length: 47.56)
           Bevy: (valid:  1, risk: 15, length: 26.38)
         Wisdom: (valid:  1, risk: 23, length: 48.77)
       Flotilla: (valid:  1, risk: 15, length: 26.80)
Total Risk: 168.0
Total Risk + Distance: 479.63

```
#### Drawbacks
Despite working, BFS had clear limitations:
- BFS only guarantees the optimal path if all step costs are equal. With Euclidean distance, diagonal steps cost √2 while cardinal steps cost 1.
- BFS provides no mechanism to optimize for risk.
To improve, I researched alternative algorithms and found A* and Dijkstra’s. After reviewing their descriptions on Wikipedia and asking clarifying questions to ChatGPT, I chose Dijkstra’s because it was easier to understand conceptually than A* (which relies on heuristics).
#### Implementing Dijkstra’s Algorithm
The core of Dijkstra’s algorithm is a min-priority queue that repeatedly selects the path with the lowest cost so far. The main challenge here was that each step had two costs: risk and distance. After consulting with ChatGPT, I resolved this by summing them, treating risk and distance equally (though weights could be applied if one needed more emphasis).
The resulting implementation was structurally similar to BFS, but used a priority queue instead of a standard queue:
```# Dijkstra's algorithm, modified to consider both risk and distance
def dijkstra(self, start: Coordinate, goal: Coordinate):
    # Priority queue: (combined cost, distance, risk, coordinate)
    pq = PriorityQueue()
    pq.put((0, 0, 0, start))

    best_cost = {start: 0}
    came_from = {start: start}

    while not pq.empty():
        combined, dist, risk, curr = pq.get()
        if curr == goal:
            break

        for neighbor in self.neighbors(curr):
            step_risk = self.map_info.risk_zones[neighbor.e][neighbor.n]
            step_dist = math.hypot(neighbor.e - curr.e, neighbor.n - curr.n)
            new_dist = dist + step_dist
            new_risk = risk + step_risk
            new_combined = new_dist + new_risk

            if (new_dist <= self.map_info.maximum_range and
                (neighbor not in best_cost or new_combined < best_cost[neighbor])):
                best_cost[neighbor] = new_combined
                came_from[neighbor] = curr
                pq.put((new_combined, new_dist, new_risk, neighbor))

    if goal not in came_from:
        return []

    # Reconstruct path
    path = []
    curr = goal
    while curr != start:
        path.append(curr)
        curr = came_from[curr]
    path.append(start)
    path.reverse()
    return path

```
#### Issues and a Quick Fix
Dijkstra’s reduced overall risk significantly, but I could only reach 7/10 sites validly—some paths exceeded maximum_range. After much debugging, I couldn’t find the exact issue. As a practical workaround, I used BFS as a fallback whenever Dijkstra’s failed. This hybrid approach worked well:
```10/10 sites reached validly
    Flamboyance: (valid:  1, risk: 0, length: 18.07)
           Knot: (valid:  1, risk: 27, length: 50.00)
        Chatter: (valid:  1, risk: 5, length: 12.49)
         Deceit: (valid:  1, risk: 0, length: 47.70)
       Conclave: (valid:  1, risk: 14, length: 36.28)
         Mewing: (valid:  1, risk: 2, length: 15.31)
     Coronation: (valid:  1, risk: 28, length: 47.56)
           Bevy: (valid:  1, risk: 15, length: 26.38)
         Wisdom: (valid:  1, risk: 23, length: 48.77)
       Flotilla: (valid:  1, risk: 15, length: 26.80)
Total Risk: 129.0
Total Risk + Distance: 458.36

```
#### Conclusion
Although the premise of this challenge seemed simple, the actual solution required deeper exploration of pathfinding algorithms. My initial BFS solution technically worked, but its inability to balance distance and risk motivated me to experiment with Dijkstra’s. While not perfect, combining Dijkstra’s with a BFS fallback yielded strong results. Overall, the challenge was both fun and educational, pushing me to strengthen my algorithmic understanding. 