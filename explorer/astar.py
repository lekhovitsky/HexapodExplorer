"""A* maze path search implementation."""

import heapq
from typing import List, Dict, Tuple, Callable, Generic, TypeVar, Optional

from utils import euclidean_distance

T = TypeVar('T')


class PriorityQueue(Generic[T]):
    elements: List[T]

    def __init__(self, elements=None):
        self.elements = [] if elements is None else elements
        heapq.heapify(self.elements)

    def empty(self) -> bool:
        return not self.elements

    def push(self, item: T, priority: float):
        heapq.heappush(self.elements, (priority, item))

    def pop(self) -> T:
        return heapq.heappop(self.elements)[1]


class AStar:
    heuristic: Callable[[Tuple[int, int], Tuple[int, int]], float]

    def __init__(self, heuristic: Callable[[Tuple[int, int], Tuple[int, int]], float] = euclidean_distance):
        self.heuristic = heuristic

    def search(self,
               maze: List[List[int]],
               start: Tuple[int, int],
               goal: Tuple[int, int],
               neighborhood: str = "N8",
               radius: float = 0,
               ) -> List[Tuple[int, int]]:
        if not self._is_free_cell(start, maze):
            return []

        open_set: PriorityQueue[Tuple[int, int]] = PriorityQueue()
        open_set.push(start, 0)

        came_from: Dict[Tuple[int, int], Optional[Tuple[int, int]]] = {start: None}
        dist: Dict[Tuple[int, int], float] = {start: 0}
        while not open_set.empty():
            current = open_set.pop()

            if euclidean_distance(current, goal) <= radius:
                return self._reconstruct_path(current, came_from)

            for node in self._get_neighbors(current, maze, neighborhood):
                new_dist = dist[current] + euclidean_distance(current, node)
                if node not in dist or new_dist < dist[node]:
                    came_from[node], dist[node] = current, new_dist
                    open_set.push(node, new_dist + self.heuristic(node, goal))

        return []

    @staticmethod
    def _get_neighbors(pos: Tuple[int, int],
                       maze: List[List[int]],
                       neighborhood: str
                       ) -> List[Tuple[int, int]]:
        assert neighborhood in ("N4", "N8")
        if neighborhood == "N4":
            neighbors = [(0, -1), (0, 1), (-1, 0), (1, 0)]
        else:
            neighbors = [(0, -1), (0, 1), (-1, 0), (1, 0),
                         (-1, -1), (-1, 1), (1, -1), (1, 1)]

        result = []
        for dx, dy in neighbors:
            x, y = pos[0] + dx, pos[1] + dy
            if AStar._is_free_cell((x, y), maze):
                result.append((x, y))
        return result

    @staticmethod
    def _reconstruct_path(node: Tuple[int, int],
                          came_from: Dict[Tuple[int, int], Optional[Tuple[int, int]]]
                          ) -> List[Tuple[int, int]]:
        path = []
        while node is not None:
            path.insert(0, node)
            node = came_from[node]
        return path

    @staticmethod
    def _is_free_cell(pos: Tuple[int, int], maze: List[List[int]]) -> bool:
        x, y = pos
        height, width = len(maze), len(maze[0])
        return 0 <= x < width and 0 <= y < height and not maze[y][x]
