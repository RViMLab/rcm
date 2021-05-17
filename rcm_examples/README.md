# Remote Center of Motion - Examples
Implements an action client that sends RCM commands to the action server via the Python API. Launch an example via
```shell
cd h_rcm_vs_ws
catkin_make
source devel/setup.bash
roslaunch rcm_examples rcm_example_node.launch
```
This launches the robot, starts the action server and sends command to the action server as specified in the [python script](python/rcm_example_node.py).
