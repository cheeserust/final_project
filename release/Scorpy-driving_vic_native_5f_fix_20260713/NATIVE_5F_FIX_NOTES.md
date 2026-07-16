# Native Nav 5F Fix

This package is based on the uploaded `Scorpy-driving_vic (1).zip` working
driving baseline. It does not require or launch `vicpinky_nav_adapter`.

## Preserved baseline behavior

- `task_servers.launch.py` starts the robot-side `nav_go_to_server` and it is
  the sole `/nav/go_to` provider.
- The original 4F/5F maps, `floor_markers.yaml`, `nav_points.yaml`, and Nav2
  parameters are preserved.
- `full_mission_test` still pauses for operator Enter input before manual
  elevator button transitions.
- Boarding marker ID 10 still uses the original 50 cm target.
- Exit calls without an override still use the original 60 cm target.

## Focused changes

- Floor landing-marker exits (target floors / marker IDs 4 and 5) request a
  70 cm target through `exit_target_distance_cm`.
- The first native Nav2 goal after each 5F/4F map switch requests a 3 second
  `start_delay_sec` and publishes `WAIT_MAP_SETTLE` feedback.
- Mission flow now uses `/elevator/exit -> /map/switch -> /nav/go_to` for both
  floor transitions.
- Native navigation accepts the legacy mission location aliases while still
  reading every coordinate from `vicpinky_task_servers/config/nav_points.yaml`.

## Expected 5F sequence

```text
WAIT_5F
-> EXIT_ELEVATOR_5F (rear landing marker ID 5, 70 cm)
-> SWITCH_5F_MAP
-> WAIT_MAP_SETTLE (3.0 s)
-> GO_TO_DELIVERY_LOCATION (native Nav2)
```

The 4F return applies the same exit/map-settle pattern with landing marker ID
4. The operator Enter prompts in `full_mission_test` are unchanged.

## Runtime ownership check

Do not launch a central `vicpinky_nav_adapter`. After starting the robot task
servers, `/nav/go_to` must have exactly one server and the only provider node
must be `nav_go_to_server`.

```bash
ros2 action info /nav/go_to -c
ros2 action info /nav/go_to -t
ros2 node list | grep -E 'nav_go_to_server|vicpinky_nav_adapter'
```

Use a clean workspace or remove old overlays before testing so an installed
adapter from another workspace cannot register a second action server.

## Field verification

Changing a landing-marker target from 60 cm to 70 cm changes the physical exit
endpoint by about 10 cm. Before a full run, pause after the 5F exit and verify
that the map, LaserScan, robot pose, and heading overlap correctly in RViz.
