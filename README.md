# AI Planning Homework 2 -- Path Planning

## Integration with [VLA*](https://github.com/ogoudey/VLA_Star)
This project is mainly used as a path planner for the VLA* project - as a tool for an LLM essentially.

## bpy Setup
Clone the repository:
```
git clone https://github.com/ogoudey/AIPHW2.git
```


Then set up an anaconda environment (makes python version control easier):
```
conda create -c bpy python=3.11
conda activate bpy
pip install matplotlib bpy aabbtree
```

## Creating objects of your own

Below I'll illustrate the sequence of things needed to run a path planner.

To make a new continuous space  that comes from (-100,-100) to (100,100), pass the `x_range` and `y_range`:
```
space = ObstacleContinuousSpace((-100, 100), (-100, 100))    
```
Add obstacles:
```
space.add(Obstacle([(-10,10), (-80,-2)]))
space.add(Obstacle([(-10,10), (10,80)]))
```
Then make state configurations from the space with utilities. The `from_grid_distribution_over_continuous_space` utility lays a grid of `n_rows` and `n_columns` evenly over the space, returning the state configurations (nodes):
```
state_nodes = from_grid_distribution_over_continuous_space(space, 10, 10)
```
Make a robot, giving it shape, assuming it only translates from up, down, left, right. To make the robot "carrying something", make it bigger.
```
robot = Robot([(-1, 1), (-1, 1)])
```
Add the robot into the space:
```
space.add(robot)
```
With state configurations, a path planner can be deployed with a search:
```
start_state = state_nodes[0]
goal_state = state_nodes[-1]
search = A_Star_Search(robot, state_nodes, start_state, goal_state)
```
Now start the search and get the path:
```
reached = search.solve(heuristic_function=manhattan_distance)
path = Path.from_search_solution(search.reached)
```

### Future Work
Multi-Agent planning:
Hierarchies based on - manueverability + size
At any point, visualize the environment (all the arguments are optional):
```
space.show(state_nodes, path, show_state_connections=True)
```

The Blender files contain meshes that are artificial and actual topographical data from Boston, respectively. Change the name of the file in `app_interfaces` to switch.

