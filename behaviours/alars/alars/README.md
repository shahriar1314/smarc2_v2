# README for recover_action_server

A ROS 2 action server that computes and publishes a multi-phase recovery trajectory for drone–SAM recovery missions. It receives GeoPoints for the **SAM** and the **buoy** as input and publishes the trajectory as `PoseStamped` setpoints.

---

## Inputs & Outputs

### Input

The server expects a JSON **dict** with the following fields:

```json
   {
   "object_position": {            // SAM head (GeoPoint)
      "latitude":  <float>,
      "longitude": <float>,
      "altitude":  <float>
   },
   "buoy_position": {              // Buoy (GeoPoint)
      "latitude":  <float>,
      "longitude": <float>,
      "altitude":  <float>
   },

   "min_height_above_water": <float>,  
   "swoop_vertical":         <float>,  // start-point height
   "swoop_horizontal":       <float>,  // start-point lateral distance (perpendicular to rope)
   "straight_before_rope":   <float>,  // switching distance to flat pass
   "straight_distance":      <float>,  // flat pass length after collecting SAM rope
   "raise_horizontal":       <float>,  // fly-out horizontal distance
   "raise_vertical":         <float>   // fly-out vertical distance
   }
```

### Output

- **Waypoints:** `geometry_msgs/PoseStamped` published on `dji_msgs/Topics.MOVE_TO_SETPOINT_TOPIC`  
- **Action result:** Success/failure logged
- **Logs:** INFO logs for validation, phase transitions, completion

---

## Parameters — Needed to be Changed/Adjusted

Goal JSON (per mission; shown with sensible starting values):

* `swoop_horizontal` (m): lateral offset from rope midpoint (example: `12.0`)
* `swoop_vertical` (m): vertical offset above rope midpoint (example: `8.0`)
* `min_height_above_water` (m): safety margin at midpoint (example: `3.0`)
* `straight_before_rope` (m): switch distance to flat pass (example: `6.0`)
* `straight_distance` (m): length of straight pass after SAM (example: `10.0`)
* `raise_horizontal` (m), `raise_vertical` (m): inclined fly-out vector (examples: `20.0`, `10.0`)

ROS params:

* `setpoint_topic`: topic your controller subscribes to (default: `dji_msgs/Topics.MOVE_TO_SETPOINT_TOPIC`)

---

## Parameters — Might Need to be Changed (tuning/validation)

* `setpoint_tolerance` (m): “reached”/phase transition tolerance (default: `0.2`)
* `num_steps`: τ-law resolution, samples start→touchdown (default: `100`)
* `target_index_offset`: look-ahead along τ-law (default: `5`)
* `tau_trajectory_starting_threshold` (m): arrival threshold at τ start (default: `0.2`)
* `width_goal_threshold` (m): max SAM–buoy separation (default: `10.0`)
* `dist_goal_threshold` (m): max drone–SAM distance (default: `1000.0`)

---

## Parameters — Not Needed to be Changed

* `initial_velocity` (m/s): τ timing scale (default: `5.0`)
* `tau_k`: τ shape parameter (default: `0.4`, range: `0.1` to `0.5`)
* `kd_alpha`: α-coupling exponent (default: `0.8`)

---

## Phase Overview

1. **Go to Start (Pre-Approach Alignment)**  
   Build start point perpendicular to rope midpoint:  
   `start = midpoint + (perp_xy * swoop_horizontal) + (ẑ * swoop_vertical)`

2. **τ-law Trajectory**  
   Smooth curved approach from `start` to `touchdown` (`midpoint + min_height_above_water` in Z),  
   with look-ahead tracking via `target_index_offset`.

3. **Flat-Horizontal Phase**  
   When within `straight_before_rope` of SAM, fly straight at constant altitude for `straight_distance`.  
   This is where the drone hook/rope should make contact with the SAM rope.

4. **Inclined Fly-Out**  
   From end of flat pass, climb along vector defined by `raise_horizontal` and `raise_vertical`.
