# Ackermann Simulink v1.0.2 — Obstacle-aware path generation

**Fix:** previously two distinct hardcoded reference paths existed: `c.route` in `ackermann_cfg.m`, and a duplicated `route = [...]` inside `function_scripts/tkqd_p.txt`. Editing obstacles changed neither path.

## Changes in v1.0.2

- `ackermann_global_route(c)`: obstacle-aware 8-neighbor A* grid route with disk-inflated robot safety envelope, then clear-segment smoothing. Caches by map configuration.
- `ackermann_cfg.m`: calls `c.route = ackermann_global_route(c)` instead of hardcoded waypoints.
- `function_scripts/tkqd_p.txt`: takes `route` through an input port; no hardcoded route nodes.
- `build_ackermann_giaotrinh.m`: connects `GlobalRoute_ASTAR` (Constant: `bookRoute`) to the third input of `TKQD_cho_P`, and adds `GlobalRouteLog`.
- `ackermann_map_bounds.m`: axes fit the actual obstacles, route, start and goal.

**Full runnable source ZIP:** delivered in the conversation as `Ackermann_GiaoTrinh_DWA_TEB_NE_v1.0.2_AStar.zip`. This GitHub directory holds the main changed files and the patch instructions, not a standalone replacement for the full project.

## Application steps

1. Use the v1.0.2 ZIP (recommended); unzip to a new folder.
2. Edit `c.obstacles` in `ackermann_cfg.m`.
3. Run in MATLAB:

```matlab
clear functions
rehash
c = ackermann_cfg();
disp(c.route);
build_ackermann_giaotrinh
out = run_ackermann_giaotrinh('DWA'); % or 'TEB'
```

Inspect `GlobalRoute_ASTAR` and `GlobalRouteLog` in the model.

## Model limitations

- A* yields a grid-based shortest-distance **guide**, **not** a proven optimal path subject to Ackermann minimum turning radius or dynamics.
- DWA and simplified TEB compute local motion against the updated path. Extremely tight maps may remain infeasible for a forward-only Ackermann vehicle.
- Rebuild/restart simulation after modifying obstacle positions. This does **not** support changing obstacles while a simulation is running.
- The source has passed static checks and an independent Python test of two maps, but has **not** been compiled or executed in MATLAB/Simulink here.
