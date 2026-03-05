# Swarm Generator

ROS1 package for multi-UAV sweeping missions, leader-follower coordination, and lat/lon waypoint execution with MRS services.

## What this package does

- Generates and sends sweep/infinity paths for a leader UAV.
- Publishes leader references and leader waypoint index for followers.
- Runs follower behavior in two modes:
  - Live leader index tracking.
  - Autonomous fallback when leader index communication is lost.
- Supports lat/lon waypoints via `transform_reference`.
- Supports Unity-based swarm following with action goals.

## Main launch files

- `launch/sweeping_generator.launch`
  - Leader node (`sweeping_generator.py`) + simple `followers.py`.
- `launch/swarm_sweeping_generator.launch`
  - Swarm sweeping using `swarm_sweeping.py`.
- `launch/swarm_sweeping_latlon.launch`
  - Swarm sweeping with lat/lon waypoints using `swarm_sweeping_latlon.py`.
- `launch/swarm_followers_unity.launch`
  - Unity/action-based follower behavior (`swarm_followers_unity.py`).

## Key scripts

- `scripts/swarm_sweeping_latlon.py`
  - Main lat/lon implementation with fallback/rejoin logic.
- `scripts/start_index_gate.sh`
  - Starts a gate node to forward leader index to a gated topic.
- `scripts/stop_index_gate.sh`
  - Stops the gate node (simulates communication loss for leader index).

## Basic startup

Start tmux environment:

```bash
./tmux/start.sh
```

Start mission (leader service):

```bash
rosservice call /$UAV_NAME/sweeping_generator/start 3.0
```

## Dependencies

Declared in `package.xml`:

- `rospy`
- `mrs_msgs`

The runtime also uses standard ROS message packages and MRS ecosystem services used by your stack.

## `swarm_sweeping_latlon.py` important parameters

Configuration is mainly in `config/sweeping_generator.yaml` plus launch arguments.

- `~octomap_planner_set` (bool)
  - Use octomap planner service for references.
- `~waypoint_list` (list)
  - Lat/lon/alt waypoints when `~automated_calculation` is false.
- `~automated_calculation` (bool)
  - Auto-generate waypoint lines from start/end lat/lon and swarm spacing.
- `~timer_comm/rate`, `~timer_comm/timeout`
  - Leader communication loss detection.
- `~fallback_goal_latch_timeout` (s)
  - Timeout to detect goal latch during fallback.
- `~fallback_goal_completion_timeout` (s)
  - Timeout to stop waiting forever for completion during fallback.
- `~rejoin_policy` (`return` | `keep_fallback` | `wait_sync`)
  - Behavior when leader index communication returns after loss.

### Rejoin policy behavior

- `return`
  - Follower immediately exits fallback and returns to leader index.
- `keep_fallback`
  - Follower ignores recovered leader index until fallback sequence finishes.
- `wait_sync`
  - Follower waits until leader index catches up to local fallback index, then rejoins.

## Simulate leader index communication loss/recovery

Use topic gating so leader node keeps running and only index communication is cut.

1. Start gate (default: `/uav1/leader_index` -> `/uav1/leader_index_gated`):

```bash
./scripts/start_index_gate.sh
```

2. Remap follower subscription to gated topic (example in launch command):

```bash
roslaunch $ROS_LAUNCH_PATH/swarm_sweeping_generator.launch /uav1/leader_index:=/uav1/leader_index_gated
```

3. Simulate communication loss:

```bash
./scripts/stop_index_gate.sh
```

4. Simulate communication recovery:

```bash
./scripts/start_index_gate.sh
```

Optional custom topics:

```bash
./scripts/start_index_gate.sh /uavX/leader_index /uavX/leader_index_gated my_gate_name
./scripts/stop_index_gate.sh my_gate_name
```
