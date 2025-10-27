from typing import List, Deque
import logging
logger = logging.getLogger(__name__)

class Space:

    def __init__(self):
        pass

class ObstacleFreeContinuousSpace(Space):
    x_range: tuple[float, float]
    y_range: tuple[float, float]
    def __init__(self, x_range: tuple[float, float], y_range: tuple[float, float]):
        super().__init__()
        self.x_range = x_range
        self.y_range = y_range

    @classmethod
    def from_image(cls, img, height, width):
        return cls(height, width)
    


class StateNode:
    
    parent: "StateNode"
    children: List["StateNode"]
    value: float
    coordinates: tuple[float, float]
    
    def __repr__(self):
        return f"{self.coordinates}"
      
    def __init__(self, value: float, coordinates: tuple[float, float], parent: "StateNode", children: List["StateNode"]):
        self.parent = parent
        self.children = children
        self.value = value
        self.coordinates = coordinates
    
    def set_child(self, child: "StateNode"):
        self.children.append(child)
        child.parent = self

import random
import math
class SearchNode:
    
    parent: "SearchNode"
    children: List["SearchNode"]
    
    def __repr__(self):
        return f"{self.state.coordinates}"
      
    def __init__(self, state_node: StateNode, parent: "SearchNode", children: List["SearchNode"]):
        self.state = state_node
        self.parent = parent
        self.children = children
        

    def set_child(self, child: "SearchNode"):
        self.children.append(child)
        child.parent = self

### Space -> States Utilities
def from_uniform_distribution_over_obstacle_free_continuous_space(space: Space, n_nodes: int):
    nodes: List[StateNode] = []
    for n in range(n_nodes):
        x_coordinate = random.random() * space.x_range[1] - space.x_range[0]
        y_coordinate = random.random() * space.y_range[1] - space.y_range[0]
        nodes.append(StateNode(None, (x_coordinate, y_coordinate), None, []))
    logger.warning("Not a usable state space - unconnected state graph.")
    return nodes # Not a usable 

def from_grid_distribution_over_obstacle_free_continuous_space(space: Space, n_rows: int, n_columns: int):
    w = space.x_range[1] - space.x_range[0]
    h = space.y_range[1] - space.y_range[0]

    nodes: List[StateNode] = []
    for j in range(n_rows):
        for i in range(n_columns):
            x = space.x_range[0] + i * w/n_rows
            y = space.y_range[0] + j *h/n_columns
            nodes.append(StateNode(None, (x, y), None, []))
    for idx in range(0, len(nodes)):
        i = idx % len(nodes)/n_columns
        j = idx % len(nodes)/n_rows
        try:
            if i + 1 < n_columns:
                nodes[idx].set_child(nodes[idx + 1])
            if j + 1 < n_rows:
                nodes[idx].set_child(nodes[idx + n_columns])
        except IndexError:
            print(i, idx + 1, j, idx + n_columns)
            print(nodes)
            raise IndexError
    return nodes
### End Space -> State Utilities
"""
class PathPlanner:

    def __init__(self):
        self.path = None

    def createPath(self, space: Space, search_nodes: List[SearchNode], start_node: SearchNode, goal_node: SearchNode):
        search = Search(search_nodes, start_node, goal_node)
        reached = search.solve()
        Path.from_search_solution(reached)
"""
class Path:
    cost: float
    nodes: List[SearchNode]

    def __init__(self, nodes: List[SearchNode]):
        self.cost = None
        self.nodes = nodes

    @classmethod
    def from_search_solution(cls, reached: SearchNode):
        if not reached:
            logging.warning("Goal not reached by search!")
            return cls([])
        nodes = [reached]
        while nodes[-1].parent:
            nodes.append(nodes[-1].parent)
        nodes.reverse()
        return cls(nodes)
    
class Search:
    nodes: List[StateNode]
    start_node: SearchNode
    goal_node: SearchNode
    reached: SearchNode

    def __init__(self, nodes: List[StateNode], start_node: StateNode, goal_node: StateNode):
        self.nodes = nodes
        self.start_node = SearchNode(start_node, None, [])
        self.goal_node = goal_node
        
        

    def solve(self):
        logger.warning("No search strategy specified.")
        self.reached = self.start_node
        return self.start_node


from collections import deque
from time import time
class BreadthFirstSearch(Search):

    def __init__(self, nodes: List[StateNode], start_node: StateNode, goal_node: StateNode):
        super().__init__(nodes, start_node, goal_node)


        
    def solve(self):
        visited: List[StateNode] = []
        frontier: Deque[SearchNode] = deque()

        frontier.appendleft(self.start_node)
        
        t0 = time()
        
        while len(frontier) > 0:
            node = frontier.pop()
            if node.state in visited: # just node with cost
                continue
            visited.append(node.state)
            
            print(f"Visits: {len(visited)}", end="\r")
            if node.state.coordinates == self.goal_node.coordinates:
                logger.info(f"Goal reached after visiting {len(visited)} nodes in {time() - t0} seconds.")
                print(f"Visits: {len(visited)}")
                self.reached = node
                return node
            for state in node.state.children:
                if not state in visited:
                    child = SearchNode(state, node, [])
                    node.set_child(child)
                    frontier.appendleft(child)
        return None
"""
space1 = ObstacleFreeContinuousSpace((-100, 100), (-100, 100))
state_nodes = from_grid_distribution_over_obstacle_free_continuous_space(space1, 10, 10)
bfs = BreadthFirstSearch(state_nodes, state_nodes[0], state_nodes[-1])
bfs.solve()
path = Path.from_search_solution(bfs.reached)
logger.info(path)
"""


def main():
    space1 = ObstacleFreeContinuousSpace((-100, 100), (-100, 100))
    state_nodes = from_grid_distribution_over_obstacle_free_continuous_space(space1, 10, 10)
    goal_state = state_nodes[-25]
    bfs = BreadthFirstSearch(state_nodes, state_nodes[0], goal_state)
    reached = bfs.solve()
    path = Path.from_search_solution(bfs.reached)
    print(path.nodes)



if __name__ == '__main__':
    main()