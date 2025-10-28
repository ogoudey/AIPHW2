from typing import List, Deque, Any
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

    def show(self, state_nodes: List["StateNode"]=[], path: "Path"=None, show_state_connections:bool=False):
        fig, ax = plt.subplots()
        
        if len(state_nodes) > 0:
            for node in state_nodes:
                ax.add_patch(plt.Circle((node.coordinates[0], node.coordinates[1]), 0.1, color='black'))
            if show_state_connections:
                for n in state_nodes:
                    x1 = n.coordinates[0]
                    y1 = n.coordinates[1]
                    for child in n.children:
                        ax.plot((x1, child.coordinates[0]), (y1, child.coordinates[1]), color='black', linewidth=2, label='link', alpha=0.1)
        if path:
            # print path nodes
            first_node = path.nodes[0]
            xs = [n.state.coordinates[0] for n in path.nodes]
            ys = [n.state.coordinates[1] for n in path.nodes]
            robot_side_len = 2 # suppose
            ax.plot(xs, ys, color='green', linewidth=robot_side_len, label='path')

            for node in path.nodes[1:]:
                ax.add_patch(plt.Circle((node.state.coordinates[0], node.state.coordinates[1]), 0.2, color='green'))
        ax.set_aspect('equal', adjustable='box')
        ax.relim()
        ax.autoscale()
        plt.show()

import matplotlib.pyplot as plt 
from aabbtree import AABB

class ObstacleContinuousSpace(Space):
    x_range: tuple[float, float]
    y_range: tuple[float, float]
    obstacles: List[Any]
    robot: "Robot"

    def __init__(self, x_range: tuple[float, float], y_range: tuple[float, float]):
        super().__init__()
        self.x_range = x_range
        self.y_range = y_range
        self.obstacles = []

    def add(self, obstacle: "Obstacle"):
        if isinstance(obstacle, Robot):
            self.robot = obstacle
        else:
            self.obstacles.append(obstacle)

    def show(self, state_nodes: List["StateNode"]=[], path: "Path"=None, show_state_connections:bool=False):
        fig, ax = plt.subplots()

        for obstacle in self.obstacles:
            coords = (obstacle.shape[0][0], obstacle.shape[1][0])
            w = obstacle.shape[0][1] - obstacle.shape[0][0]
            h = obstacle.shape[1][1] - obstacle.shape[1][0]
            ax.add_patch(plt.Rectangle(coords, w, h, fill=True, color='red', alpha=0.5))
        if len(state_nodes) > 0:
            for node in state_nodes:
                ax.add_patch(plt.Circle((node.coordinates[0], node.coordinates[1]), 0.1, color='black'))
            if show_state_connections:
                for n in state_nodes:
                    x1 = n.coordinates[0]
                    y1 = n.coordinates[1]
                    for child in n.children:
                        ax.plot((x1, child.coordinates[0]), (y1, child.coordinates[1]), color='black', linewidth=2, label='link')
        if path:
            # print path nodes
            first_node = path.nodes[0]
            xs = [n.state.coordinates[0] for n in path.nodes]
            ys = [n.state.coordinates[1] for n in path.nodes]
            robot_side_len = self.robot.shape[0][1] - self.robot.shape[0][0]
            ax.plot(xs, ys, color='green', linewidth=robot_side_len, label='path')

            for node in path.nodes[1:]:
                ax.add_patch(plt.Circle((node.state.coordinates[0], node.state.coordinates[1]), 0.2, color='green'))
        ax.set_aspect('equal', adjustable='box')
        ax.relim()
        ax.autoscale()
        plt.show()

    @classmethod
    def from_image(cls, img, height, width):
        return cls(height, width)
    
    def robot_collide(self, coordinate_1: tuple[float, float], obstacle: "Obstacle", coordinate_2: tuple[float, float]):
        if not self.robot:
            raise Exception("No robot in space, but collision with robot requested...")
        n_stamps = 10
        x = coordinate_2[0] - coordinate_1[0]
        y = coordinate_2[1] - coordinate_1[1]
        obstacle_aabb = AABB(obstacle.shape)
        collisions = 0.0
        #print(f"Robot at {coordinate_1} with shape {self.robot.shape} towards {coordinate_2}")
        for i in range(n_stamps + 1):
            x_offset = (i) * x / n_stamps + coordinate_1[0]
            y_offset = (i) * y / n_stamps + coordinate_1[1]
            stamp = AABB([(self.robot.shape[0][0] + x_offset, self.robot.shape[0][1] + x_offset), (self.robot.shape[1][0] + y_offset, self.robot.shape[1][1] + y_offset)])
            if stamp.overlaps(obstacle_aabb):
                #print(f"Collision of stamp {i}:{stamp} with {obstacle_aabb}")
                collisions += 100.0
            else:
                pass
                #print(f"Obstacle {obstacle_aabb} not collide with stamp {stamp}")
        return collisions


            
    

import random
class StateNode:
    
    parent: "StateNode"
    children: List["StateNode"]
    space: Space | ObstacleFreeContinuousSpace | ObstacleContinuousSpace
    coordinates: tuple[float, float]
    
    def __repr__(self):
        return f"{self.coordinates}"
      
    def __init__(self, space: Space | ObstacleFreeContinuousSpace | ObstacleContinuousSpace, coordinates: tuple[float, float], parent: "StateNode", children: List["StateNode"]):
        self.parent = parent
        self.children = children
        self.space = space
        self.coordinates = coordinates
    
    def set_child(self, child: "StateNode"):
        self.children.append(child)
        child.parent = self
    
    def has_collision(self, end_state):

        #print("Yet to implement better cost function\r")
        if not end_state.space == self.space:
            return 0.0
        if type(self.space) == Space or isinstance(self.space, ObstacleFreeContinuousSpace):
            return 0.0
        else:
            cost = 0.1
            for obstacle in self.space.obstacles:
                cost += self.space.robot_collide(self.coordinates, obstacle, end_state.coordinates)
                    
                # See if beam from self.coords to end_state.coords hits the obstacle
                
            return cost

    
    
        

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

class CostlySearchNode:
    parent: "CostlySearchNode"
    children: dict["CostlySearchNode", float]

    def __repr__(self):
        return f"{self.state.coordinates}"

    def __init__(self, state_node: StateNode, parent:"CostlySearchNode", children: dict["CostlySearchNode", float]):
        self.state = state_node
        self.parent = parent
        self.children = children

    def set_child(self, child: "CostlySearchNode", cost: float=0.0):
        
        self.children[child] = cost

class Obstacle:
    def __init__(self, shape: List[tuple[float, float]]):
        """
        shape: List[tuple[float, float]] Example: [(-1, 1), (-1, 1)]
        """
        self.shape = shape

class Robot(Obstacle):
    def __init__(self, shape: List[tuple[float, float]]):
        super().__init__(shape)


### Space -> States Utilities
def from_uniform_distribution_over_continuous_space(space: ObstacleFreeContinuousSpace, n_nodes: int):
    nodes: List[StateNode] = []
    for n in range(n_nodes):
        x_coordinate = random.random() * space.x_range[1] - space.x_range[0]
        y_coordinate = random.random() * space.y_range[1] - space.y_range[0]
        nodes.append(StateNode(space, (x_coordinate, y_coordinate), None, []))
    logger.warning("Not a usable state space - unconnected state graph.")
    return nodes # Not a usable 




def from_grid_distribution_over_continuous_space(space: ObstacleContinuousSpace, n_rows: int, n_columns: int):
    w = space.x_range[1] - space.x_range[0]
    h = space.y_range[1] - space.y_range[0]
    initial_x_offset = w/n_columns
    initial_y_offset = h/n_rows
    nodes: List[StateNode] = []
    for j in range(n_rows):
        for i in range(n_columns):
            x = initial_x_offset + space.x_range[0] + i * w/n_rows
            y = initial_y_offset + space.y_range[0] + j *h/n_columns
            nodes.append(StateNode(space, (x, y), None, []))
    for idx in range(0, len(nodes)):
        i = idx % (len(nodes)/n_columns) + 1
        #j = idx+1 % (len(nodes)/n_rows)
        j = math.floor(idx/n_columns) + 1
        #print(f"{idx}: i:{i} j:{j} right:{i + 1} <= {n_columns}? up:{j + 1} <= {n_rows}?")
        try:
            if i + 1 <= n_columns:
                nodes[idx].set_child(nodes[idx + 1])
                #print(f"{idx} => {idx + 1}")
            if j + 1 <= n_rows:
                nodes[idx].set_child(nodes[idx + n_columns])
                #print(f"{idx} => {idx + n_columns}")

        except IndexError:
            print("---error---")
            print(i, idx + 1, j, idx + n_columns)
            print(nodes)
            raise IndexError
    return nodes



### End "Space -> State" Utilities
import math
### Heuristic functions ###
def manhattan_distance(state: StateNode, goal_state: StateNode):
    #print(f"Heuristic {state} -> {goal_state} = {abs(goal_state.coordinates[0] - state.coordinates[0]) + abs(goal_state.coordinates[1] - state.coordinates[1])}")
    return abs(goal_state.coordinates[0] - state.coordinates[0]) + abs(goal_state.coordinates[1] - state.coordinates[1])

### End heuristic functions ###


class Path:
    total_cost: float | None
    nodes: List[SearchNode] | List[CostlySearchNode]

    def __init__(self, nodes: List[SearchNode], total_cost: float=0.0):
        self.total_cost = total_cost
        self.nodes = nodes

    @classmethod
    def from_search_solution(cls, reached: SearchNode | CostlySearchNode):
        total_cost = 0.0
        if not reached:
            logging.warning("Goal not reached by search!")
            return cls([])
        nodes = [reached]
        while nodes[-1].parent:
            if isinstance(reached, CostlySearchNode):
                total_cost += nodes[-1].parent.children[nodes[-1]]
            nodes.append(nodes[-1].parent)
        nodes.reverse()
        if isinstance(reached, CostlySearchNode):
            return cls(nodes, total_cost)
        else:
            return cls(nodes)
    
class Search:
    nodes: List[StateNode]
    start_node: SearchNode
    goal_node: StateNode
    reached: SearchNode

    def __init__(self, nodes: List[StateNode], start_node: StateNode, goal_node: StateNode):
        self.nodes = nodes
        
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
        self.start_node = SearchNode(start_node, None, [])

        
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
                print(f"Goal reached after {len(visited)} visits in {time() - t0} seconds.")
                self.reached = node
                return node
            for state in node.state.children:
                if not state in visited:
                    child = SearchNode(state, node, [])
                    node.set_child(child)
                    frontier.appendleft(child)
        return None
import itertools
class CostlyBreadthFirstSearch(Search):

    def __init__(self, robot: Robot, nodes: List[StateNode], start_node: StateNode, goal_node: StateNode):
        super().__init__(nodes, start_node, goal_node)
        self.start_node = CostlySearchNode(start_node, None, dict())
        self.robot = robot
        
    def solve(self):
        visited: List[StateNode] = []
        frontier: Deque[CostlySearchNode] = deque()
        frontier.appendleft(self.start_node)
        
        t0 = time()
        
        while len(frontier) > 0:
            node = frontier.pop()
            if node.state in visited: # just node with cost
                continue
            visited.append(node.state)
            
            print(f"Visits: {len(visited)}", end="\r")
            if node.state.coordinates == self.goal_node.coordinates:
                print(f"Goal reached after {len(visited)} visits in {time() - t0} seconds.")
                self.reached = node
                return node
            for state in node.state.children:
                if not state in visited:
                    child = CostlySearchNode(state, node, dict())
                    node.set_child(child)
                    frontier.appendleft(child)
        return None


from queue import PriorityQueue

class A_Star_Search(Search):

    def __init__(self, robot: Robot, nodes: List[StateNode], start_node: StateNode, goal_node: StateNode):
        super().__init__(nodes, start_node, goal_node)
        self.start_node = CostlySearchNode(start_node, None, dict())
        self.counter = itertools.count() # For tiebreakers

        
    def solve(self, heuristic_function: Any=manhattan_distance):
        visited: List[StateNode] = []
        cost_cache: dict[CostlySearchNode, float] = dict()
        frontier: PriorityQueue[tuple[float, int, CostlySearchNode]] = PriorityQueue()

        initial_cost = 0.0
        cost_cache[self.start_node] = initial_cost
        frontier.put((heuristic_function(self.start_node.state, self.goal_node) + initial_cost, next(self.counter), self.start_node))
        t0 = time()
        
        while not frontier.empty():
            #print(frontier.queue)
            __, __, node = frontier.get()
            if node.state in visited: # just node with cost
                continue
            visited.append(node.state)
            
            print(f"Visits: {len(visited)}", end="\r")
            if node.state.coordinates == self.goal_node.coordinates:
                print(f"Goal reached after {len(visited)} visits in {time() - t0} seconds.")
                self.reached = node
                return node
            for state in node.state.children:
                if not state in visited:
                    h = heuristic_function(state, self.goal_node)
                    cost = node.state.has_collision(state)
                    child = CostlySearchNode(state, node, dict())
                    path_cost = cost_cache[node] + cost
                    cost_cache[child] = path_cost
                    node.set_child(child, cost)
                    priority = h + path_cost
                    #print(f"Putting {child} with cost {cost}")
                    frontier.put((priority, next(self.counter), child)) 
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
    def sequence1():
        print("--------BFS---------")
        space1 = ObstacleFreeContinuousSpace((-1000, 1000), (-1000, 1000))
        state_nodes = from_grid_distribution_over_continuous_space(space1, 100, 100)
        goal_state = state_nodes[-1]
        bfs = BreadthFirstSearch(state_nodes, state_nodes[0], goal_state)
        reached = bfs.solve()
        path = Path.from_search_solution(bfs.reached)
        print(path.nodes)
        #space1.show(state_nodes,path, show_state_connections=True)

    def sequence2():
        # for testing transition to A*
        print("-------- Costly (0 cost) ----------")
        space2 = ObstacleContinuousSpace((-100, 100), (-100, 100))
        state_nodes = from_grid_distribution_over_continuous_space(space2, 10, 10)
        goal_state = state_nodes[-1]
        robot = Robot([(-1,1),(-1,1)])
        bfs = CostlyBreadthFirstSearch(robot, state_nodes, state_nodes[0], goal_state)
        reached = bfs.solve()
        path = Path.from_search_solution(bfs.reached)
        print(path.nodes)
        print(len(path.nodes), path.total_cost)
        space2.show(state_nodes,path, show_state_connections=True)

    def faster():
        print("------- A star -------")
        space3 = ObstacleFreeContinuousSpace((-1000, 1000), (-1000, 1000))
        #space3.show()
        state_nodes = from_grid_distribution_over_continuous_space(space3, 100, 100)
        #space3.show(state_nodes)
        goal_state = state_nodes[-1]
        robot = Robot([(-1, 1), (-1, 1)])
        bfs = A_Star_Search(robot, state_nodes, state_nodes[0], goal_state)
        reached = bfs.solve(heuristic_function=manhattan_distance)
        path = Path.from_search_solution(bfs.reached)
        print(path.nodes)
        print(f"Path length {len(path.nodes)}, with cost {path.total_cost}")
        space3.show(state_nodes, path, show_state_connections=True)

    def sequence():
        print("------- A star -------")
        space3 = ObstacleContinuousSpace((-100, 100), (-100, 100))
        #space3.show()
        state_nodes = from_grid_distribution_over_continuous_space(space3, 10, 10)
        #space3.show(state_nodes)
        goal_state = state_nodes[-1]
        robot = Robot([(-1, 1), (-1, 1)])
        space3.add(robot)
        space3.add(Obstacle([(-10,100), (-10,10)]))
        space3.add(Obstacle([(-60,-20), (-20,60)]))
        space3.add(Obstacle([(30,40), (30,40)]))
        bfs = A_Star_Search(robot, state_nodes, state_nodes[0], goal_state)
        reached = bfs.solve(heuristic_function=manhattan_distance)
        path = Path.from_search_solution(bfs.reached)
        print(path.nodes)
        print(f"Path length {len(path.nodes)}, with cost {path.total_cost}")
        space3.show(state_nodes, path, show_state_connections=True)
    
    def all_or_nothing(robot_laden):
        print("------- A star -------")
        space3 = ObstacleContinuousSpace((-100, 100), (-100, 100))
        state_nodes = from_grid_distribution_over_continuous_space(space3, 10, 10)
        goal_state = state_nodes[-1]
        robot = Robot([(-1, 1), (-1, 1)]) if not robot_laden else Robot([(-10, 10), (-10, 10)])
        space3.add(robot)
        space3.add(Obstacle([(-10,10), (-80,-2)]))
        space3.add(Obstacle([(-40,10), (10,100)]))
        bfs = A_Star_Search(robot, state_nodes, state_nodes[0], goal_state)
        reached = bfs.solve(heuristic_function=manhattan_distance)
        path = Path.from_search_solution(bfs.reached)
        print(path.nodes)
        print(f"Path length {len(path.nodes)}, with cost {path.total_cost}")
        space3.show(state_nodes, path, show_state_connections=True)

    def test():
        space = ObstacleContinuousSpace((-10,10), (-10,10))
        state_nodes = from_grid_distribution_over_continuous_space(space, 2, 2)
        space.show(state_nodes, show_state_connections=True)

    def sequence3(robot_laden=False):
        print("------- A star -------")
        space3 = ObstacleContinuousSpace((-100, 100), (-100, 100))
        state_nodes = from_grid_distribution_over_continuous_space(space3, 10, 10)
        goal_state = state_nodes[-1]
        robot = Robot([(-1, 1), (-1, 1)]) if not robot_laden else Robot([(-10, 10), (-10, 10)])
        space3.add(robot)
        space3.add(Obstacle([(-10,10), (-80,-2)]))
        space3.add(Obstacle([(-10,10), (10,80)]))
        bfs = A_Star_Search(robot, state_nodes, state_nodes[0], goal_state)
        reached = bfs.solve(heuristic_function=manhattan_distance)
        path = Path.from_search_solution(bfs.reached)
        print(path.nodes)
        print(f"Path length {len(path.nodes)}, with cost {path.total_cost}")
        space3.show(state_nodes, path, show_state_connections=True)

    def test():
        space = ObstacleContinuousSpace((-10,10), (-10,10))
        state_nodes = from_grid_distribution_over_continuous_space(space, 2, 2)
        space.show(state_nodes, show_state_connections=True)

    #sequence1()
    #faster()
    sequence3(True)
    #all_or_nothing(True)
            


if __name__ == '__main__':
    main()