# 机器人生成位置说明

## 默认设置

- **默认世界文件**: `leo_empty.sdf`
- **默认生成位置**: X=0.0, Y=0.0, Z=1.65 (米)
  - X: 前后方向（正X向前）
  - Y: 左右方向（正Y向左）
  - Z: 高度（1.65米是机器人底部距离地面的高度）

## 如何修改机器人生成位置

### 方法1: 通过Launch参数（推荐）

在启动时指定位置参数：

```bash
# 使用默认位置 (0, 0, 1.65)
ros2 launch leo_gz_bringup leo_gz_rviz.launch.py

# 指定X, Y, Z位置
ros2 launch leo_gz_bringup leo_gz_rviz.launch.py x:=2.0 y:=3.0 z:=1.65

# 只修改X位置
ros2 launch leo_gz_bringup leo_gz_rviz.launch.py x:=5.0

# 修改位置并指定世界文件
ros2 launch leo_gz_bringup leo_gz_rviz.launch.py \
    sim_world:=leo_with_obstacles.sdf \
    x:=2.0 y:=1.0 z:=1.65
```

### 方法2: 直接修改Launch文件

如果你想永久改变默认位置，可以编辑以下文件：

- `leo_gz_bringup/launch/leo_gz.launch.py` - 修改默认值
- `leo_gz_bringup/launch/leo_gz_rviz.launch.py` - 修改默认值

例如，将默认X位置改为2.0米：

```python
x_pos = DeclareLaunchArgument(
    "x",
    default_value="2.0",  # 改为2.0
    description="Initial X position of the robot (meters)",
)
```

## 重要说明

⚠️ **机器人生成位置不是在SDF世界文件中设置的！**

- SDF文件（如`leo_empty.sdf`）只定义世界环境（地面、障碍物等）
- 机器人的生成位置是在Launch文件中通过`ros_gz_sim create`命令设置的
- 修改SDF文件不会改变机器人的生成位置

## 坐标系说明

Gazebo使用的坐标系：
- **X轴**: 前后方向（正X向前）
- **Y轴**: 左右方向（正Y向左）
- **Z轴**: 上下方向（正Z向上）

地面在Z=0，所以Z=1.65表示机器人底部距离地面1.65米。

## 可用的Launch参数

所有launch文件都支持以下参数：

- `sim_world`: 世界文件路径（默认：`leo_empty.sdf`）
- `robot_ns`: 机器人命名空间（默认：空字符串）
- `x`: 初始X位置（默认：`0.0`）
- `y`: 初始Y位置（默认：`0.0`）
- `z`: 初始Z位置（默认：`1.65`）
- `use_rviz`: 是否启动RViz（仅leo_gz_rviz.launch.py，默认：`true`）
- `rviz_config`: RViz配置文件路径（仅leo_gz_rviz.launch.py）

## 示例场景

### 场景1: 在原点生成机器人
```bash
ros2 launch leo_gz_bringup leo_gz_rviz.launch.py x:=0.0 y:=0.0 z:=1.65
```

### 场景2: 在障碍物世界中的特定位置生成
```bash
ros2 launch leo_gz_bringup leo_gz_rviz.launch.py \
    sim_world:=leo_with_obstacles.sdf \
    x:=5.0 y:=0.0 z:=1.65
```

### 场景3: 生成多个机器人（不同位置）
```bash
# 第一个机器人
ros2 launch leo_gz_bringup leo_gz_rviz.launch.py \
    robot_ns:=leo1 x:=0.0 y:=0.0 z:=1.65

# 在另一个终端，添加第二个机器人
ros2 launch leo_gz_bringup spawn_robot.launch.py \
    robot_ns:=leo2 x:=3.0 y:=0.0 z:=1.65
```
