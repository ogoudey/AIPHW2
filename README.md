# AI Planning Homework 2 -- Path Planning

## Setup
To setup the python environment and start the submission, do:
```
$ ./run.sh
```
### Alternatively,
```
python3 -m venv .venv
. .venv/bin/activate
pip install -r requirements.txt
python3 space.py
```

## Creating objects of your own
Below I'll illustrate the sequence of things needed to run a path planner.

To make a new continuous space  that comes from (-100,-100) to (100,100), pass the `x_range` and `y_range`:
```
space3 = ObstacleContinuousSpace((-100, 100), (-100, 100))    
```
Add obstacles:
```
space3.add(Obstacle([(-10,10), (-80,-2)]))
space3.add(Obstacle([(-10,10), (10,80)]))
```
Then make state configurations from the space with utilities. The `from_grid_distribution_over_continuous_space` utility lays a grid of `n_rows` and `n_columns` evenly over the space, returning the state configurations (nodes):
```
state_nodes = from_grid_distribution_over_continuous_space(space3, 10, 10)
```
Make a robot, giving it shape, assuming it only translates from up, down, left, right. To make the robot "carrying something", make it bigger.
```
robot = Robot([(-1, 1), (-1, 1)])
```
Add the robot into the space:
```
space3.add(robot)
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
At any point, visualize the environment (all the arguments are optional):
```
space3.show(state_nodes, path, show_state_connections=True)
```
