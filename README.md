# planner_bridge
This package is used to send waypoints from a planner to individual crazyflies

# Quickstart

1. Make sure to set up simulation as instructed in `crazyswarm2_application` README.md

2. Run the planner script
```
ros2 launch planner_bridge planner_bridge.launch.py
```

# Issues

1. How do we ensure that the crazyflie acknowledges the waypoint?
2. Do we really want to remove the inactive crazyflies or add them to an inactive list?
3. 'flight_state' reported from AgentStateFeedback is always in 'landing mode', how do we obtain information about the actual state of the crazyflie?


# TODO 
1. Write test cases