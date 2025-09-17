"""
    This module is your primary workspace. Add whatever helper functions, classes, data structures, imports... etc here.

    We expect most results will utilize more than just dumping code into the plan_paths()
        function, that just serves as a meaningful entry point.

    In order for the rest of the scoring to work, you need to make sure you have correctly
        populated the Destination.path for each result you produce.
"""
import typing
import math
from queue import PriorityQueue
from collections import deque

import numpy as np
from typing import Dict

from map_info import Coordinate, Destination, MapInfo


class PathPlanner:
    def __init__(self, map_info: MapInfo, destinations: typing.List["Destination"]):
        self.map_info: MapInfo = map_info
        self.destinations: typing.List["Destination"] = destinations

    # Helper to generate neighbors of a coordinate
    def neighbors(self, coord: Coordinate):
        directions = [
            (1, 0), (-1, 0), (0, 1), (0, -1), # cardinals
            (1, 1), (1, -1), (-1, 1), (-1, -1) # diagonals
        ]
        width, height = self.map_info.risk_zones.shape
        for de, dn in directions: 
            newE, newN = coord.e + de, coord.n + dn 
            if 0 <= newE < width and 0 <= newN < height and self.map_info.risk_zones[newE][newN] != 2:
                yield Coordinate(newE, newN)

    # Dijkstra's algorithm, modified to consider two weights, risk and distance
    def dijkstra(self, start: Coordinate, goal: Coordinate):
        # Priority queue: (combined cost, risk, distance, coordinate)
        pq = PriorityQueue()
        pq.put((0, 0, 0, start))

        # For each coordinate, store the combined cost of the most optimal path (lowest cost) to it
        best_cost = {start : 0}
        # For each coordinate, store the previous coordinate of the most optimal path that we reach it from
        came_from = {start : start}

        while not pq.empty(): 
            combined, dist, risk, curr = pq.get()

            if curr == goal:
                break

            for neighbor in self.neighbors(curr):
                step_risk = self.map_info.risk_zones[neighbor.e][neighbor.n] # Risk value of neighbor
                step_dist = math.hypot(neighbor.e - curr.e, neighbor.n - curr.n) # Euclidean distance
                new_dist = dist + step_dist
                new_risk = risk + step_risk
                new_combined = new_dist + new_risk # Weigh distance and risk equally

                # If we have not reached maximum range for distance and the neighbor coordinate has not been seen yet,
                # or if we have found a better path to this coordinate, add it to best_cost, came_from, and pq
                if new_dist <= self.map_info.maximum_range and (neighbor not in best_cost or new_combined < best_cost[neighbor]):
                    best_cost[neighbor] = new_combined
                    came_from[neighbor] = curr
                    pq.put((new_combined, new_dist, new_risk, neighbor))
        
        # If we could not find a valid path using dijkstra, return empty
        if goal not in came_from:
            return []
        
        # Reconstruct our path by working our way back from the goal and continuously looking at came_from
        path = []
        curr = goal
        while curr != start:
            path.append(curr)
            curr = came_from[curr]
        path.append(start)
        # Reverse our path since we constructed it by backtracking
        path.reverse()
        return path
    
    # Bfs solution, not the most optimal but always finds a valid path
    def bfs(self, start: Coordinate, goal: Coordinate):
        q = deque([start])
        # For each coordinate, store the previous coordinate that we reached it from
        came_from = {start: start}

        while q:
            curr = q.popleft()

            if curr == goal:
                break

            for neighbor in self.neighbors(curr):
                # If we haven't seen neighbor, add it to came_from
                if neighbor not in came_from:
                    came_from[neighbor] = curr
                    q.append(neighbor)

        # If a valid path to goal was not found, return empty
        if goal not in came_from:
            return []
        
        # Reconstruct our path by working our way back from the goal and continuously looking at came_from
        path = []
        curr = goal
        while curr != start:
            path.append(curr)
            curr = came_from[curr]
        path.append(start)
        # Reverse our path since we constructed it by backtracking
        path.reverse()
        return path


    def plan_paths(self):
        """
        This is the function you should re-write. It is expected to mutate the list of
        destinations by calling each Destination's set_path() with the resulting
        path as an argument.

        The default construction shows this format, and should produce 10 invalid paths.
        """
        for site in self.destinations:
            # Try to use dijkstra. For some reason my dijkstra doesn't always find a valid path with length < maximum_range
            path_coords = self.dijkstra(self.map_info.start_coord, site.coord)
            if path_coords == []:
                # Fall back to bfs if dijkstra didn't work
                path_coords = self.bfs(self.map_info.start_coord, site.coord)
            site.set_path(path_coords)