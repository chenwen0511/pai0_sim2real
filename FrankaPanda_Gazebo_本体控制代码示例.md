---
AIGC: {"Label": "1", "ContentProducer": "001191330101MA27WPYJ18xliu", "ProduceID": "c2378876-ef91-4f8f-96a2-004260a1c464", "ReserveCode1": "iflow", "ContentPropagator": "iflow", "PropagateID": "iflow", "ReserveCode2": "iflow"}
---

# Franka Panda Gazebo本体控制代码示例指南

## 一、Gazebo仿真环境准备
### 1.1 系统环境要求
- 操作系统：Ubuntu 20.04 LTS
- ROS版本：Melodic Morenia
- 依赖库：libfranka、libpoco-dev、libeigen3-dev

### 1.2 启动Gazebo仿真
```bash
roslaunch franka_gazebo franka_gazebo.launch payload_mass:=0.5  # 可配置负载质量
```

## 二、官方示例控制器使用
### 2.1 基础运动控制
```bash
roslaunch franka_control franka_control.launch robot_ip:=172.16.0.2 load_gripper:=true
roslaunch franka_example_controllers cartesian_pose_example_controller.launch robot_ip:=172.16.0.2
```

## 三、夹爪控制接口
### 3.1 初始化与归零
```bash
rostopic pub -1 /franka_gripper/homing/goal franka_gripper/HomingActionGoal '{}'
```

### 3.2 抓取与移动操作
```bash
rostopic pub -1 /franka_gripper/grasp/goal franka_gripper/GraspActionGoal '{
  goal: {
    width: 0.02,          # 目标抓取宽度(m)
    epsilon: {inner: 0.01, outer: 0.02},  # 允许偏差范围(m)
    speed: 0.1,           # 夹爪运动速度(m/s)
    force: 20            # 夹持力(N)
  }
}'
```

## 四、自定义控制程序开发
### 4.1 工作空间配置
```bash
mkdir -p franka_workspace/src
cd franka_workspace
catkin_make
cmake -DFranka_DIR:PATH=/opt/ros/melodic/include/libfranka
```

### 4.2 示例程序编译
```bash
cd libfranka/build/
./examples/generate_cartesian_velocity_motion 172.16.0.2
```