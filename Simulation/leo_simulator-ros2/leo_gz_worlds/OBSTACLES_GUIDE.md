# 在Gazebo世界中添加障碍物指南

本指南介绍如何在Gazebo世界中添加障碍物体，用于机器人仿真和导航测试。

## 方法1: 直接在SDF世界文件中添加简单几何体

这是最简单的方法，适合快速添加简单的障碍物。

### 添加立方体盒子

```xml
<model name="obstacle_box">
  <pose>X Y Z ROLL PITCH YAW</pose>  <!-- 位置和姿态 -->
  <static>true</static>  <!-- 静态物体，不会移动 -->
  <link name="link">
    <collision name="collision">
      <geometry>
        <box>
          <size>长 宽 高</size>  <!-- 单位：米 -->
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
        <ambient>R G B A</ambient>  <!-- 环境光颜色 -->
        <diffuse>R G B A</diffuse>  <!-- 漫反射颜色 -->
        <specular>R G B A</specular>  <!-- 镜面反射颜色 -->
      </material>
    </visual>
  </link>
</model>
```

### 添加圆柱体

```xml
<model name="obstacle_cylinder">
  <pose>X Y Z 0 0 0</pose>
  <static>true</static>
  <link name="link">
    <collision name="collision">
      <geometry>
        <cylinder>
          <radius>0.5</radius>  <!-- 半径（米） -->
          <length>1</length>     <!-- 高度（米） -->
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

### 添加球体

```xml
<model name="obstacle_sphere">
  <pose>X Y Z 0 0 0</pose>
  <static>true</static>
  <link name="link">
    <collision name="collision">
      <geometry>
        <sphere>
          <radius>0.5</radius>  <!-- 半径（米） -->
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

## 方法2: 创建自定义障碍物模型

对于需要重复使用的复杂障碍物，可以创建自定义模型。

### 步骤1: 创建模型目录结构

```bash
mkdir -p models/my_obstacle
cd models/my_obstacle
```

### 步骤2: 创建model.config文件

```xml
<?xml version="1.0"?>
<model>
  <name>My Obstacle</name>
  <version>1.0</version>
  <sdf version="1.6">model.sdf</sdf>
  <author>
    <name>Your Name</name>
  </author>
  <description>
    Description of your obstacle
  </description>
</model>
```

### 步骤3: 创建model.sdf文件

```xml
<?xml version="1.0"?>
<sdf version="1.6">
  <model name="my_obstacle">
    <static>true</static>
    <link name="link">
      <collision name="collision">
        <geometry>
          <!-- 你的几何体定义 -->
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
</sdf>
```

### 步骤4: 在世界文件中使用自定义模型

```xml
<model name="obstacle_instance_1">
  <include>
    <uri>model://my_obstacle</uri>
    <pose>X Y Z ROLL PITCH YAW</pose>
  </include>
</model>
```

## 方法3: 使用Gazebo内置模型库

Gazebo提供了一些内置模型，可以通过以下方式使用：

```xml
<model name="cinder_block">
  <include>
    <uri>https://fuel.gazebosim.org/1.0/OpenRobotics/models/Cinder Block</uri>
    <pose>X Y Z 0 0 0</pose>
  </include>
</model>
```

## 姿态（Pose）说明

`<pose>`标签定义物体的位置和姿态：
- 格式：`X Y Z ROLL PITCH YAW`
- X, Y, Z: 位置（米）
- ROLL, PITCH, YAW: 欧拉角（弧度）
  - ROLL: 绕X轴旋转
  - PITCH: 绕Y轴旋转
  - YAW: 绕Z轴旋转

示例：
- `<pose>3 0 0.5 0 0 0</pose>` - 在X=3m, Y=0m, Z=0.5m，无旋转
- `<pose>0 0 0.5 0 0 1.57</pose>` - 在原点上方0.5m，绕Z轴旋转90度

## 示例世界文件

参考 `leo_with_obstacles.sdf` 文件，其中包含了多种障碍物的示例。

## 使用示例世界

```bash
ros2 launch leo_gz_bringup leo_gz_rviz.launch.py sim_world:=leo_with_obstacles.sdf
```

## 注意事项

1. **静态vs动态**: 使用`<static>true</static>`标记静态障碍物，它们不会受物理引擎影响
2. **碰撞检测**: 确保同时定义了`<collision>`和`<visual>`，这样机器人才能正确检测到障碍物
3. **性能**: 过多的复杂障碍物可能影响仿真性能
4. **坐标系**: 默认坐标系中，Z轴向上，X轴向前，Y轴向左

## 常见问题

**Q: 如何调整障碍物的大小？**
A: 修改几何体的尺寸参数，如`<size>`或`<radius>`

**Q: 如何改变障碍物的颜色？**
A: 修改`<material>`中的`<ambient>`和`<diffuse>`颜色值（RGBA格式，0-1范围）

**Q: 如何让障碍物可以移动？**
A: 移除`<static>true</static>`标签，或设置为`false`

**Q: 如何添加多个相同的障碍物？**
A: 创建多个`<model>`标签，使用不同的名称和位置
