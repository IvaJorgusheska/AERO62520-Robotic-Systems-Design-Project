# 快速添加障碍物指南

## 快速开始

### 1. 使用示例世界文件

已创建一个包含多种障碍物示例的世界文件：`leo_with_obstacles.sdf`

运行示例：
```bash
ros2 launch leo_gz_bringup leo_gz_rviz.launch.py sim_world:=leo_with_obstacles.sdf
```

### 2. 在现有世界文件中添加障碍物

打开你的世界文件（如 `leo_empty.sdf`），在 `</world>` 标签之前添加障碍物模型。

#### 最简单的例子 - 添加一个红色盒子：

```xml
<model name="my_obstacle">
  <pose>3 0 0.5 0 0 0</pose>
  <static>true</static>
  <link name="link">
    <collision name="collision">
      <geometry>
        <box>
          <size>1 1 1</size>
        </box>
      </geometry>
    </collision>
    <visual name="visual">
      <geometry>
        <box>
          <size>1 1 1</size>
        </box>
      </geometry>
      <material>
        <ambient>0.8 0.2 0.2 1</ambient>
        <diffuse>0.8 0.2 0.2 1</diffuse>
      </material>
    </visual>
  </link>
</model>
```

## 常用障碍物模板

### 立方体（盒子）
```xml
<model name="box_obstacle">
  <pose>X Y Z 0 0 0</pose>
  <static>true</static>
  <link name="link">
    <collision name="collision">
      <geometry>
        <box>
          <size>长 宽 高</size>
        </box>
      </geometry>
    </collision>
    <visual name="visual">
      <geometry>
        <box>
          <size>长 宽 高</size>
        </box>
      </geometry>
      <material>
        <ambient>0.8 0.2 0.2 1</ambient>
        <diffuse>0.8 0.2 0.2 1</diffuse>
      </material>
    </visual>
  </link>
</model>
```

### 圆柱体
```xml
<model name="cylinder_obstacle">
  <pose>X Y Z 0 0 0</pose>
  <static>true</static>
  <link name="link">
    <collision name="collision">
      <geometry>
        <cylinder>
          <radius>0.5</radius>
          <length>1</length>
        </cylinder>
      </geometry>
    </collision>
    <visual name="visual">
      <geometry>
        <cylinder>
          <radius>0.5</radius>
          <length>1</length>
        </cylinder>
      </geometry>
      <material>
        <ambient>0.2 0.8 0.2 1</ambient>
        <diffuse>0.2 0.8 0.2 1</diffuse>
      </material>
    </visual>
  </link>
</model>
```

### 球体
```xml
<model name="sphere_obstacle">
  <pose>X Y Z 0 0 0</pose>
  <static>true</static>
  <link name="link">
    <collision name="collision">
      <geometry>
        <sphere>
          <radius>0.5</radius>
        </sphere>
      </geometry>
    </collision>
    <visual name="visual">
      <geometry>
        <sphere>
          <radius>0.5</radius>
        </sphere>
      </geometry>
      <material>
        <ambient>0.2 0.2 0.8 1</ambient>
        <diffuse>0.2 0.2 0.8 1</diffuse>
      </material>
    </visual>
  </link>
</model>
```

## 位置和姿态说明

`<pose>X Y Z ROLL PITCH YAW</pose>`

- **X, Y, Z**: 位置坐标（米）
  - X: 前后方向（正X向前）
  - Y: 左右方向（正Y向左）
  - Z: 上下方向（正Z向上）

- **ROLL, PITCH, YAW**: 旋转角度（弧度）
  - 0度 = 0
  - 90度 = 1.57 (π/2)
  - 180度 = 3.14 (π)

## 颜色参考

修改 `<ambient>` 和 `<diffuse>` 中的 RGBA 值（0-1范围）：
- 红色: `0.8 0.2 0.2 1`
- 绿色: `0.2 0.8 0.2 1`
- 蓝色: `0.2 0.2 0.8 1`
- 黄色: `0.8 0.8 0.2 1`
- 橙色: `0.8 0.4 0.2 1`
- 灰色: `0.5 0.5 0.5 1`

## 详细文档

更多详细信息请参考：`OBSTACLES_GUIDE.md`
