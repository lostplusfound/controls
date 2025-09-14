"""
    This module is your primary workspace. Add whatever helper functions, classes, data structures, imports... etc here.

    We expect most results will utilize more than just dumping code into the plan_paths()
        function, that just serves as a meaningful entry point.

    In order for the rest of the scoring to work, you need to make sure you have correctly
        populated the Destination.path for each result you produce.
"""
import typing
from queue import PriorityQueue
from collections import deque

import numpy as np
from typing import Dict

from map_info import Coordinate, Destination, MapInfo


class PathPlanner:
    def __init__(self, map_info: MapInfo, destinations: typing.List["Destination"]):
        self.map_info: MapInfo = map_info
        self.destinations: typing.List["Destination"] = destinations

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

    def bfs(self, start: Coordinate, goal: Coordinate):
        q = deque([start])
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
        
        path = []
        curr = goal
        while curr != start:
            path.append(curr)
            curr = came_from[curr]
        path.append(start)
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
            path_coords = self.bfs(self.map_info.start_coord, site.coord)
            site.set_path(path_coords)