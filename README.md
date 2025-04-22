# 🤖 BumperBot Workspace - Simulation & Localization

This ROS 2 project simulates a BumperBot robot with support for localization, navigation, and safety features. It uses Gazebo, RViz2, and custom ROS 2 nodes.

---

## 🚀 Launch the robot with `odometry_motion_model`

```bash
ros2 launch bumperbot_bringup simulated_robot.launch.py
ros2 run key_teleop key_teleop
ros2 run rviz2 rviz2
ros2 run bumperbot_localization odometry_motion_model --ros-args -p alpha3:=0.1
```

---

## 🧭 Display the robot in RViz

```bash
ros2 launch bumperbot_description display.launch.py
ros2 launch bumperbot_description gazebo.launch.py world_name:=small_house
```

---

## 🛑 Safety Stop (Put obstacles in Gazebo to test)

```bash
ros2 launch bumperbot_bringup simulated_robot.launch.py
ros2 run bumperbot_utils safety_stop
ros2 run key_teleop key_teleop
```

---

## 🗺️ Display the map (Global Localization)

```bash
ros2 launch bumperbot_localization global_localization.launch.py
rviz2
```

> In RViz:
> - Enable the Map display plugin
> - Set **Durability** to `Transient Local`

---

## 🧱 Generate Occupancy Grid (Mapping with known poses)

```bash
ros2 launch bumperbot_bringup simulated_robot.launch.py world_name:=small_house
ros2 run bumperbot_mapping mapping_with_known_poses
ros2 run key_teleop key_teleop
rviz2
```

---

## 📍 Localization using AMCL

```bash
ros2 launch bumperbot_bringup simulated_robot.launch.py world_name:=small_house
ros2 launch bumperbot_localization global_localization.launch.py
ros2 run key_teleop key_teleop
```

> In RViz:
> - Map plugin: set durability to `Transient Local`
> - `/particle_cloud`: set reliability to `Best Effort`
> - Frame: `map`
> - Provide an **initial pose estimation** to AMCL

---

## 📁 Project Structure (simplified)

```
bumperbot_ws/
├── src/
│   ├── bumperbot_bringup/
│   ├── bumperbot_localization/
│   ├── bumperbot_mapping/
│   └── bumperbot_utils/
├── install/ (ignored)
├── build/ (ignored)
├── log/ (ignored)
└── README.md
```

---