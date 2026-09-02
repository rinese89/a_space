# trajectory_system_bringup_pkg

Global ROS 2 bringup for the trajectory chain:

1. `static_trajectory_manager`
2. `static_trajectory_conflict_manager`
3. `trajectory_server_pkg` (`trajectory_server_node`)

This package intentionally contains **only**:

- `launch/trajectory_system_bringup.launch.py`
- the three centralized YAML configuration files.

It does **not** duplicate the component launch files. The master launch obtains them
at runtime from the installed package shares:

- `static_trajectory_manager/launch/static_trajectory_manager.launch.py`
- `static_trajectory_conflict_manager/launch/static_trajectory_conflict_manager.launch.py`
- `trajectory_server_pkg/launch/trajectory_server_node.launch.py`

All three component launches must accept a launch argument named `config_file`.
The static trajectory manager launch already does. The conflict manager and
trajectory server launch files need the small compatibility change supplied
alongside this package if their installed versions still hard-code their own YAML.

## Launch

```bash
ros2 launch trajectory_system_bringup_pkg trajectory_system_bringup.launch.py
```

Optional YAML overrides:

```bash
ros2 launch trajectory_system_bringup_pkg trajectory_system_bringup.launch.py \
  static_trajectories_config_file:=/path/static_trajectories.yaml \
  conflict_manager_config_file:=/path/static_trajectory_conflict_manager.yaml \
  trajectory_server_config_file:=/path/trajectory_server_node.yaml
```
