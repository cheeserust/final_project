# Deprecated arm_task_server

This package is retained for compatibility tests only. Production launch files
use `roscue_arm_pick/task_executor_node`, which owns `/arm/execute`, `/arm/pick`,
`/arm/place`, `/arm/press_button`, and `/arm/homing`.

Do not run this package together with `roscue_arm_pick`; both expose overlapping
`/arm/*` action server names.
