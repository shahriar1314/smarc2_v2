# RescuePointServer

A ROS 2 action server that computes and publishes a multi-phase recovery trajectory for drone–SAM recovery missions. It receives GeoPoints as input and publishes the trajectory as `PoseStamped` setpoints.

---

## Inputs & Outputs

### Input

* Action Goal: `smarc_mission_msgs/action/BaseAction`
* Goal JSON (`std_msgs/String` inside the action):

  ```json
  {
    "rope_points": [
      {"latitude": <float>, "longitude": <float>, "altitude": <float>},
      {"latitude": <float>, "longitude": <float>, "altitude": <float>}
    ],
    "min_height_above_water": <float>,
    "swoop_vertical": <float>,
    "swoop_horizontal": <float>,
    "straight_before_rope": <float>,
    "straight_distance": <float>,
    "raise_horizontal": <float>,
    "raise_vertical": <float>
  }
  ```

### Output

* Waypoints: `geometry_msgs/PoseStamped` on `setpoint_topic` (frame: `/<robot_name>/odom`)
* Action Result: `success: bool`
* Logs: `INFO` messages for goal validation, phase changes, and completion

---

## Parameters — Needed to be Changed

Goal JSON (per mission; shown with sensible starting values):

* `swoop_horizontal` (m): lateral offset from rope midpoint (example: `12.0`)
* `swoop_vertical` (m): vertical offset above rope midpoint (example: `8.0`)
* `min_height_above_water` (m): safety margin at midpoint (example: `1.0`)
* `straight_before_rope` (m): switch distance to flat pass (example: `6.0`)
* `straight_distance` (m): length of straight pass after SAM (example: `10.0`)
* `raise_horizontal` (m), `raise_vertical` (m): inclined fly-out vector (examples: `20.0`, `10.0`)

ROS params:

* `setpoint_topic`: topic your controller subscribes to (default: `"move_to_setpoint"`)


---

## Parameters — Might Need to be Changed (tuning/validation)

* `dt` (s): timer/publish rate (default: `0.05`)
* `num_steps`: τ-law resolution, samples start→touchdown (default: `400`)
* `setpoint_tolerance` (m): “reached”/phase transition tolerance (default: `0.1`)
* `target_index_offset`: look-ahead along τ-law (default: `5`)
* `tau_trajectory_starting_threshold` (m): arrival threshold at τ start (default: `0.2`)
* `width_goal_threshold` (m): max SAM–buoy separation (default: `10.0`)
* `dist_goal_threshold` (m): max drone–SAM distance (default: `1000.0`)

---

## Parameters — Not Needed to be Changed (advanced)

* `initial_velocity` (m/s): τ timing scale (default: `5.0`)
* `tau_k`: τ shape parameter (default: `0.4`, range: `0.1` to `0.5`)
* `kd_alpha`: α-coupling exponent (default: `0.8`)

---

## Phase Overview

1. Go to Start (Pre-Approach Alignment)
   Build start point perpendicular to rope midpoint:
   `start = midpoint + (perp_xy * swoop_horizontal) + (ẑ * swoop_vertical)`

2. τ-law Trajectory
   Smooth curved approach from `start` to `touchdown` (midpoint + `min_height_above_water` in Z), with look-ahead tracking via `target_index_offset`.

3. Flat-Horizontal Phase 
   When within `straight_before_rope` of SAM, fly straight at constant altitude for `straight_distance`. This is where the hook/rope of the drone should make contact with the rope of the SAM

4. Inclined Fly-Out
   From end of flat pass, climb along vector defined by `raise_horizontal` and `raise_vertical`.
