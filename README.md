# planner_bridge
This package is used to send waypoints from a planner to individual crazyflies

# Quickstart

1. Make sure to set up simulation as instructed in `crazyswarm2_application` README.md

2. Takeoff all drones
```
ros2 topic pub /user crazyswarm_application/msg/UserCommand \
'{cmd: 'takeoff_all', uav_id: [], goal: {x: 0.0, y: 0.0, z: 0.0}, yaw: 0.0}' --once
```

3. Run the planner script
```
ros2 run planner_bridge planner_bridge
```

# Parameters that can be tuned:
```
goal_tolerance: "Tolerance for distance from agent to drone for goal to be considered fulfilled"

```

# Issues

1. How do we ensure that the crazyflie acknowledges the waypoint?
2. Do we really want to remove the inactive crazyflies or add them to an inactive list?
3. 'flight_state' reported from AgentStateFeedback is always in 'landing mode'