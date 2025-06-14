## 🚀 Nav2 Costmap 配置 Cheat Sheet（速查手册）

### 🔧 基本结构（local / global 通用）

```yaml
local_costmap:
  local_costmap:
    ros__parameters:
      global_frame: odom                 # Local costmap 推荐设为 odom
      robot_base_frame: base_link       # 一般与 URDF 中保持一致
      update_frequency: 5.0             # 地图更新频率（建议5Hz及以上）
      publish_frequency: 2.0            # 发布costmap的频率
      resolution: 0.05
      use_sim_time: true                # 仿真时必须设置为 true
      robot_radius: 0.22                # 或者配置 footprint
      rolling_window: true              # local costmap 必须为 true

      plugins: ["obstacle_layer", "inflation_layer"]  # 可选：static_layer, voxel_layer

      always_send_full_costmap: true
```

---

### 📡 激光配置（重点部分）

```yaml
obstacle_layer:
  plugin: "nav2_costmap_2d::ObstacleLayer"
  enabled: true
  observation_sources: scan
  scan:
    topic: /scan                     # ⚠️ 必须匹配 ros2 topic list 中的实际话题
    sensor_frame: LDS-01            # ⚠️ 必须匹配该话题 header.frame_id
    data_type: "LaserScan"          # 或 PointCloud2
    expected_update_rate: 0.3       # Hz, 如果太低会 warning，可设为 0.3（3Hz）
    marking: true
    clearing: true
    obstacle_range: 2.5
    raytrace_range: 3.0
```

---

### 🧱 膨胀层配置

```yaml
inflation_layer:
  plugin: "nav2_costmap_2d::InflationLayer"
  inflation_radius: 0.55            # 越大越保守（避障距离大）
  cost_scaling_factor: 3.0
```

---

### 🧊 体素层（可选，用于 3D 点云）

```yaml
voxel_layer:
  plugin: "nav2_costmap_2d::VoxelLayer"
  z_resolution: 0.05
  z_voxels: 16
  origin_z: 0.0
  max_obstacle_height: 2.0
  publish_voxel_map: true
  observation_sources: scan
  scan:
    topic: /scan
    sensor_frame: LDS-01
    data_type: "LaserScan"
    marking: true
    clearing: true
```

---

### 🗺 静态地图层（global costmap 专用）

```yaml
static_layer:
  plugin: "nav2_costmap_2d::StaticLayer"
  map_subscribe_transient_local: true
```

---

## ✅ 检查清单（你今后排错用得上）

| 项目                                       | 要求或建议                                |
| ---------------------------------------- | ------------------------------------ |
| `/scan` 能否收到消息？                          | `ros2 topic echo /scan`              |
| `header.frame_id` 与 `sensor_frame` 是否一致？ | `ros2 topic echo /scan -n 1`         |
| 该 frame 在 TF 中是否存在？                      | `ros2 run tf2_tools view_frames` 查看树 |
| `topic` 配置是否正确？                          | 和你监听的 scan 话题匹配                      |
| `expected_update_rate` 是否过低？             | 如果激光是 5Hz，设 0.3s 以上最稳妥               |
| 所有相关节点 `use_sim_time: true`？             | 统一时间源防止 timestamp 错误                 |
| 激光是否过稀疏？                                 | 射线数太少或视野太小可能导致 costmap 过空            |
| 是否多个 static\_transform\_publisher 重复广播？  | 会导致 TF 冲突、丢帧                         |


