from typing import List, Deque, Any
import logging

logger = lambda x : print(x)

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
                if node.visited:
                    ax.add_patch(plt.Circle((node.coordinates[0], node.coordinates[1]), 0.25, color='purple'))
                else:
                    ax.add_patch(plt.Circle((node.coordinates[0], node.coordinates[1]), 0.1, color='black'))
            if show_state_connections:
                for n in state_nodes:
                    x1 = n.coordinates[0]
                    y1 = n.coordinates[1]
                    for child in n.children:
                        ax.plot((x1, child.coordinates[0]), (y1, child.coordinates[1]), color='black', linewidth=2, label='link', alpha=0.02)
        if path:
            # logger path nodes
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
    obstacles: dict[Any, float]
    robot: "Robot"

    def __init__(self, x_range: tuple[float, float], y_range: tuple[float, float]):
        super().__init__()
        self.x_range = x_range
        self.y_range = y_range
        self.obstacles = dict()

    def add(self, obstacle: "Obstacle", cost: float=100.0):
        if isinstance(obstacle, Robot):
            self.robot = obstacle
        else:
            self.obstacles[obstacle] = cost

    def get_cost(self, coordinates_a, coordinates_b):
        total_cost = 0.1
        for obstacle, cost in self.obstacles.items():
            total_cost += self.robot_collide(coordinates_a, obstacle, coordinates_b)
        return total_cost

    def get_obstacles_from_altitude(self, columns: dict[float, dict[float, float]], cost: float=1.0, condition = lambda altitude: altitude > 0.0):
        a_column = list(columns.values())[0]
        
        cell_size = (list(columns.keys())[1] - list(columns.keys())[0], list(columns.keys())[1] - list(columns.keys())[0]) # not too right...
        cell_size = (1, 1)
        log(cell_size)
        for c in columns:
            making_obstacle = False
            for y, z in sorted(list(columns[c].items()), key=lambda item: item[0]):
                if condition(columns[c][y]):
                    if not making_obstacle:
                        y0 = y
                        y1 = y
                        making_obstacle = True
                    else:
                        if y - y1 > cell_size[1]: #If these are not adjacent cells (they coindidently have the same x)
                            #log(f"{y} is different than {y1} by > {cell_size}")
                            self.obstacles[Obstacle([(c - cell_size[0]/2, c + cell_size[0]/2), (y0 - cell_size[1]/2, y1 + cell_size[1]/2)])] = cost
                            y0 = y
                        y1 = y
                else:
                    if making_obstacle:
                        self.obstacles[Obstacle([(c - cell_size[0]/2, c + cell_size[0]/2), (y0 - cell_size[1]/2, y1 + cell_size[1]/2)])] = cost
                        making_obstacle = False
                    else:
                        making_obstacle=False
            if making_obstacle:
                try:
                    self.obstacles[Obstacle([(c - cell_size[0]/2, c + cell_size[0]/2), (y0 - cell_size[1]/2, y1 + cell_size[1]/2)])] = cost
                except ValueError:
                    log("Bad obstacle")
                    raise ValueError
                
    def show(self, state_nodes: List["StateNode"]=[], path: "Path"=None, show_state_connections:bool=False, show: bool=False):
        fig, ax = plt.subplots()

        for obstacle, __ in self.obstacles.items():
            coords = (obstacle.shape[0][0], obstacle.shape[1][0])
            w = obstacle.shape[0][1] - obstacle.shape[0][0]
            h = obstacle.shape[1][1] - obstacle.shape[1][0]
            ax.add_patch(plt.Rectangle(coords, w, h, fill=True, color='red', alpha=0.5))
        if len(state_nodes) > 0:
            for node in state_nodes:
                if node.visited:
                    ax.add_patch(plt.Circle((node.coordinates[0], node.coordinates[1]), 0.2, color='purple'))
                else:
                    ax.add_patch(plt.Circle((node.coordinates[0], node.coordinates[1]), 0.1, color='black'))
                if node.is_start:
                    ax.add_patch(plt.Circle((node.coordinates[0], node.coordinates[1]), 2, color='brown'))
                elif node.is_goal:
                    ax.add_patch(plt.Circle((node.coordinates[0], node.coordinates[1]), 2, color='green'))
            if show_state_connections:
                for n in state_nodes:
                    x1 = n.coordinates[0]
                    y1 = n.coordinates[1]
                    for child in n.children:
                        ax.plot((x1, child.coordinates[0]), (y1, child.coordinates[1]), color='black', linewidth=2, label='link', alpha=0.1)
        if not path is None:
            # logger path nodes
            first_node = path.nodes[0]
            xs = [n.state.coordinates[0] for n in path.nodes]
            ys = [n.state.coordinates[1] for n in path.nodes]
            robot_side_len = self.robot.shape[0][1] - self.robot.shape[0][0]
            ax.plot(xs, ys, color='green', linewidth=robot_side_len, label='path')

            for node in path.nodes[1:]:
                ax.add_patch(plt.Circle((node.state.coordinates[0], node.state.coordinates[1]), robot_side_len, color='green'))
        ax.set_aspect('equal', adjustable='box')
        ax.relim()
        ax.autoscale()
        if show:
            plt.show()
        else:
            file_name = "navigation"
            plt.savefig(f"{file_name}.png")

    @classmethod
    def from_image(cls, img, height, width):
        return cls(height, width)
    
    def robot_collide(self, coordinate_1: tuple[float, float], obstacle: "Obstacle", coordinate_2: tuple[float, float]):
        if not self.robot:
            raise Exception("No robot in space, but collision with robot requested...")
        n_stamps = 10
        x = coordinate_2[0] - coordinate_1[0]
        y = coordinate_2[1] - coordinate_1[1]
        try:
            obstacle_aabb = AABB(obstacle.shape)    
        except ValueError:
            log(obstacle)
            log(obstacle.shape)
            raise ValueError
        collisions = 0.0
        #log(f"Robot at {coordinate_1} with shape {self.robot.shape} towards {coordinate_2}")
        for i in range(n_stamps + 1):
            x_offset = (i) * x / n_stamps + coordinate_1[0]
            y_offset = (i) * y / n_stamps + coordinate_1[1]
            stamp = AABB([(self.robot.shape[0][0] + x_offset, self.robot.shape[0][1] + x_offset), (self.robot.shape[1][0] + y_offset, self.robot.shape[1][1] + y_offset)])
            if stamp.overlaps(obstacle_aabb):
                #log(f"Collision of stamp {i}:{stamp} with {obstacle_aabb}")
                collisions += 100.0
            else:
                pass
                #log(f"Obstacle {obstacle_aabb} not collide with stamp {stamp}")
        return collisions


            
    
import math
import random
class StateNode:
    
    parent: "StateNode"
    children: List["StateNode"]
    space: Space | ObstacleFreeContinuousSpace | ObstacleContinuousSpace
    coordinates: tuple[float, float]
    visited: bool
    is_goal: bool
    is_start: bool
    def __repr__(self):
        return f"{self.coordinates}"
      
    def __init__(self, space: Space | ObstacleFreeContinuousSpace | ObstacleContinuousSpace, coordinates: tuple[float, float], parent: "StateNode", children: List["StateNode"]):
        self.parent = parent
        self.children = children
        self.space = space
        self.coordinates = coordinates
        self.visited = False
        self.is_goal = False
        self.is_start = False
    
    def set_child(self, child: "StateNode"):
        self.children.append(child)
        child.parent = self
    
    def has_collision(self, end_state):

        #log("Yet to implement better cost function\r")
        if not end_state.space == self.space:
            return 0.0
        if type(self.space) == Space or isinstance(self.space, ObstacleFreeContinuousSpace):
            return 0.0
        else:
            cost = self.space.get_cost(self.coordinates, end_state.coordinates)
            
                    
                # See if beam from self.coords to end_state.coords hits the obstacle
                
            return cost

    def distance(self, other: "StateNode"):
        d = math.sqrt(math.pow(self.coordinates[0] - other.coordinates[0], 2) + math.pow(self.coordinates[1] - other.coordinates[1], 2))
        return d
    
    def angle(self, other: "StateNode"):
        # return angle to coordinates of other node
        a = other.coordinates[0] - self.coordinates[0]
        o = other.coordinates[1] - self.coordinates[1]
        angle = math.degrees(math.atan2(o, a))
        return (angle + 180) % 360 # opposite...

    def get_nearest_among(self, nodes:List["StateNode"]):
        nearest = None
        for node in nodes:
            if nearest:
                if self.distance(node) < self.distance(nearest):
                    nearest = node
            else:
                nearest = node
        return nearest
    
    def get_nearest_among_w_condition(self, nodes:List["StateNode"], condition: Any):
        nearest = None
        for node in nodes:
            if nearest:
                if self.distance(node) < self.distance(nearest):
                    angle = self.angle(node)
                    okay_angle = condition(angle)
                    #log(f"{angle} is {okay_angle}")
                    if okay_angle:
                        nearest = node
            else:
                angle = self.angle(node)
                okay_angle = condition(angle)
                #log(f"{angle} is {okay_angle}")
                if okay_angle:
                    nearest = node
        return nearest

    def already_in(self, nodes:List["StateNode"]):
        for node in nodes:
            if self.distance(node) < 0.01:
                #log(f"{self.coordinates} too close to {node.coordinates}")
                return True
        return False
    
    @classmethod    
    def create_branch(cls, space, nodes, nearest_node, random_node, dT:float=2.0):
        
        nearest_x, nearest_y = nearest_node.coordinates[0], nearest_node.coordinates[1]
        random_x, random_y = random_node.coordinates[0], random_node.coordinates[1]
        d = nearest_node.distance(random_node)
        try:
            coordinates = (
                nearest_x + (random_x - nearest_x) / d * dT,
                nearest_y + (random_y - nearest_y) / d * dT,
            )
        except ZeroDivisionError:
            log(f"Division by zero for random node {random_node}")
            return None
        
        new_node = cls(space, coordinates, nearest_node, [])
        
        #nearest_node.set_child(new_node) # directed
        #return new_node
        if nearest_node.has_collision(new_node) == 0.1: # If it equals the default cost for any edge...
            if not new_node.already_in(nodes):
                nearest_node.set_child(new_node) # directed
                return new_node
            else:
                #log(f"Already in...")
                del new_node
                return None
        else:
            #log(f"Has collision...")
            del new_node
            return None

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

    def as_message(self):
        #log(f"Waypoint:{self.state.coordinates[0]} {self.state.coordinates[1]}")
        return f"{self.state.coordinates[0]} {self.state.coordinates[1]}"

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
    return nodes # Not a usable 

def from_grid_distribution_over_continuous_space(space: ObstacleContinuousSpace, n_rows: int, n_columns: int):
    # Now symmetric!
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
        #log(f"{idx}: i:{i} j:{j} right:{i + 1} <= {n_columns}? up:{j + 1} <= {n_rows}?")
        try:
            if i + 1 <= n_columns:
                nodes[idx].set_child(nodes[idx + 1])
                nodes[idx + 1].set_child(nodes[idx])
                #log(f"{idx} => {idx + 1}")
            if j + 1 <= n_rows:
                nodes[idx].set_child(nodes[idx + n_columns])
                nodes[idx + n_columns].set_child(nodes[idx])
                #log(f"{idx} => {idx + n_columns}")

        except IndexError:
            log("---error---")
            log(i, idx + 1, j, idx + n_columns)
            log(nodes)
            raise IndexError
    return nodes

def from_rrt(space: ObstacleContinuousSpace, start: StateNode, num_nodes, beta: float, dT: float=2.0, goal: StateNode | None = None):
    if not goal:
        raise Exception("Must provide a goal to use RRTs.")
    nodes: List[StateNode] = [start]
    for i in range(num_nodes):
        random_node = StateNode(space, (random.uniform(*space.x_range), random.uniform(*space.y_range)), None, [])
        
        #if random_node.distance(goal) < beta:
        #    random_node = goal
        def good_condition(theta):
            if theta > 0 and theta < 100:
                return False
            else:
                return True
        nearest_node = random_node.get_nearest_among_w_condition(nodes, good_condition)
        if not nearest_node:
            continue
            raise Exception(f"Nearest node to {random_node.coordinates} (is {nearest_node})")

        new_node_Q = StateNode.create_branch(space, nodes, nearest_node, random_node, dT)
        if new_node_Q:
            nodes.append(new_node_Q)
            if new_node_Q.distance(goal) < beta:
                new_node_Q.is_goal = True
                new_node_Q.coordinates = goal.coordinates
                log("Found goal, returning early...")
                return nodes
        #log(f"{random_node.coordinates} --- d to goal: {random_node.distance(goal)}, nearest_coords: {nearest_node.coordinates}, new_node: {new_node_Q}")
        #log(f"Nodes placed: {len(nodes)}", end="\r")
    if not goal.already_in(nodes):
        log("This RRT does not reach the goal...")
        nodes.append(goal)
    return nodes


### End "Space -> State" Utilities
import math
### Heuristic functions ###
def manhattan_distance(state: StateNode, goal_state: StateNode):
    #log(f"Heuristic {state} -> {goal_state} = {abs(goal_state.coordinates[0] - state.coordinates[0]) + abs(goal_state.coordinates[1] - state.coordinates[1])}")
    return abs(goal_state.coordinates[0] - state.coordinates[0]) + abs(goal_state.coordinates[1] - state.coordinates[1])

def euclidean_distance(state: StateNode, goal_state: StateNode):
    dx = goal_state.coordinates[0] - state.coordinates[0]
    dy = goal_state.coordinates[1] - state.coordinates[1]
    return (dx**2 + dy**2) ** 0.5

def euclidean_manhattan_combo(state: StateNode, goal_state: StateNode):
    return 0.5 * euclidean_distance(state, goal_state) + 0.5 * manhattan_distance(state, goal_state)
### End heuristic functions ###


class Path:
    total_cost: float | None
    nodes: List[SearchNode] | List[CostlySearchNode]


    def __init__(self, nodes: List[SearchNode], total_cost: float =0.0, costs_by_nodes: List[float] = []):
        self.total_cost = total_cost
        self.nodes = nodes
        self.costs_by_nodes = costs_by_nodes

    @classmethod
    def from_search_solution(cls, reached: SearchNode | CostlySearchNode):
        total_cost = 0.0
        if not reached:
            logging.warning("Goal not reached by search!")
            return cls([])
        costs_by_nodes = []
        nodes = [reached]
        while nodes[-1].parent:
            if isinstance(reached, CostlySearchNode):
                total_cost += nodes[-1].parent.children[nodes[-1]]
                costs_by_nodes.append(nodes[-1].parent.children[nodes[-1]])
            nodes.append(nodes[-1].parent)
        costs_by_nodes.reverse()    
        nodes.reverse()
        if isinstance(reached, CostlySearchNode):
            return cls(nodes, total_cost, costs_by_nodes)
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
            
            #log(f"Visits: {len(visited)} Size of frontier: {len(frontier)}", end="\r")
            if node.state.coordinates == self.goal_node.coordinates:
                log(f"Goal reached after {len(visited)} visits in {time() - t0} seconds.")
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
        self.reached = None

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
            
            #log(f"Visits: {len(visited)}", end="\r")
            if node.state.coordinates == self.goal_node.coordinates:
                log(f"Goal reached after {len(visited)} visits in {time() - t0} seconds.")
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
        self.reached = None
        
    def solve(self, heuristic_function: Any=manhattan_distance):
        visited: List[StateNode] = []
        cost_cache: dict[CostlySearchNode, float] = dict()
        frontier: PriorityQueue[tuple[float, int, CostlySearchNode]] = PriorityQueue()

        initial_cost = 0.0
        cost_cache[self.start_node] = initial_cost
        frontier.put((heuristic_function(self.start_node.state, self.goal_node) + initial_cost, next(self.counter), self.start_node))
        t0 = time()
        
        while not frontier.empty():
            #log(frontier.queue)
            __, __, node = frontier.get()
            if node.state in visited: # just node with cost
                continue
            visited.append(node.state)
            node.state.visited = True
            
            #log(f"Visits: {len(visited)} Size of frontier: {frontier.qsize()}", end="\r")
            
            if node.state.coordinates == self.goal_node.coordinates:
                log(f"Goal reached after {len(visited)} visits in {time() - t0} seconds.")
                self.reached = node
                return node
            for state in node.state.children:
                if not state in visited: # or not state.visited
                    h = heuristic_function(state, self.goal_node)
                    cost = node.state.has_collision(state)
                    child = CostlySearchNode(state, node, dict())
                    path_cost = cost_cache[node] + cost
                    cost_cache[child] = path_cost
                    node.set_child(child, cost)
                    priority = h + path_cost
                    #log(f"Putting {child} with cost {cost}")
                    frontier.put((priority, next(self.counter), child)) 
        log(f"Visits: {len(visited)}")
        return None

"""
space1 = ObstacleFreeContinuousSpace((-100, 100), (-100, 100))
state_nodes = from_grid_distribution_over_obstacle_free_continuous_space(space1, 10, 10)
bfs = BreadthFirstSearch(state_nodes, state_nodes[0], state_nodes[-1])
bfs.solve()
path = Path.from_search_solution(bfs.reached)
logger.info(path)
"""
USE_BPY = False
if USE_BPY:
    from app_interfaces import columns, x_range, y_range


from time import sleep

class UnityEnvironment:
    def __init__(self, boat, destinations, terrain):
        self.boat = boat
        self.destinations = destinations
        self.terrain = terrain

### Exposed sequences ###
def rrt_astar(unity_environment: UnityEnvironment, goal: str, num_nodes:int=1200, terrain_aabb:tuple=((-50,50), (-50, 50)), costly_altitude:float=0.58, logger:Any=lambda t: log(t)):
    global log
    log = logger
    log(f"Terrain is declared to be {terrain_aabb}")
    terrain = unity_environment.terrain
    log(f"Terrain data shape: {terrain.shape}")
    destinations = unity_environment.destinations
    boat = unity_environment.boat
    # get columns, range_x, range_y
    x_range, y_range = terrain_aabb[0], terrain_aabb[1] # (-50,50), (-50, 50) # overriding...

    side_offset = 0

    def scale(side):
        return side
        return side * 100 / 1025 - side_offset
    columns = {
        float(scale(col)): {float(scale(row)): float(terrain[row, col]) for row in range(terrain.shape[0])}
        for col in range(terrain.shape[1])
    }
    #log(columns)
    space = ObstacleContinuousSpace(x_range, y_range)
    space.get_obstacles_from_altitude(columns, cost=100.0, condition=lambda altitude: altitude > costly_altitude)
    #space.get_obstacles_from_altitude(columns, cost=100.0, condition=lambda altitude: altitude > 0.58)
    
    goal_coordinates = (min(max(destinations[goal]["position"]["x"], x_range[0]), x_range[1]), min(max(destinations[goal]["position"]["z"], y_range[0]), y_range[1]))
    goal_state = StateNode(space, goal_coordinates, None, [])
    
    goal_state.is_goal = True
    start_coordinates = (boat["position"]["x"], boat["position"]["z"])
    log(f"Start coordinates: {start_coordinates}, end coordinates: {goal_coordinates}")
    start_state = StateNode(space, start_coordinates, None, [])
    start_state.is_start = True
    robot = Robot([(-0.1, 0.1), (-0.1, 0.1)])
    space.add(robot)
    space.show([start_state, goal_state])
    state_nodes = from_rrt(space, start_state, num_nodes, beta=20.0, dT=20.0, goal=goal_state)
    #input("[enter] to visualize the state nodes")
    space.show(state_nodes, show_state_connections=True)
    
    bfs = A_Star_Search(robot, state_nodes, start_state, goal_state)
    reached = bfs.solve(heuristic_function=euclidean_manhattan_combo)
    path = Path.from_search_solution(bfs.reached)
    log(path.costs_by_nodes)
    log(f"Total cost: {path.total_cost}")
    #input("[enter] to visualize the path")
    try:
        space.show(state_nodes, path, True)
        pass
    except Exception as e:
        log(e)
    log(f"Path planning done.")
    return path

def main():
    def sequence():
        log("---------- A* Search ----------")    
        log("---- over artifial terrain ----")    
        #columns, x_range, y_range = load_grid("terrain.blend")
        space = ObstacleContinuousSpace(x_range, y_range)
        space.get_obstacles_from_altitude(columns, cost=100.0, condition=lambda altitude: altitude > 0.0)
        state_nodes = from_grid_distribution_over_continuous_space(space, 20, 20)
        goal_state = state_nodes[-2]
        robot = Robot([(-5, 5), (-5, 5)])
        space.add(robot)
        bfs = A_Star_Search(robot, state_nodes, state_nodes[19], goal_state)
        reached = bfs.solve()
        path = Path.from_search_solution(bfs.reached)
        log(path.costs_by_nodes)
        log(f"Total cost: {path.total_cost}")
        input("[enter] to visualize the path")
        space.show(state_nodes, path)

    def boston():
        log("----------------- A* Search -----------------")
        log("-- from Boston Harbor to home (Back Bay) ----")  
        #columns, x_range, y_range = load_grid("boston.blend")      
        space = ObstacleContinuousSpace(x_range, y_range)
        space.get_obstacles_from_altitude(columns, cost=100.0, condition=lambda altitude: altitude > 0.0)
        state_nodes = from_grid_distribution_over_continuous_space(space, 100, 100)
        
        goal_state = state_nodes[-5284] # could be an attribute of the mesh
        goal_state.is_goal = True
        
        start_state = state_nodes[99]
        start_state.is_start = True
        #space.show(state_nodes)
        robot = Robot([(-5, 5), (-5, 5)])
        space.add(robot)
        bfs = A_Star_Search(robot, state_nodes, start_state, goal_state)
        reached = bfs.solve(heuristic_function=euclidean_manhattan_combo)
        path = Path.from_search_solution(bfs.reached)
        log(path.costs_by_nodes)
        log(f"Total cost: {path.total_cost}")
        input("[enter] to visualize the path")
        


    boston()
    #sequence()
    

if __name__ == '__main__':
    main()
