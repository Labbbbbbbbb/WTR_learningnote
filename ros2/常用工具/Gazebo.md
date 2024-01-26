# Gazebo是什么

Gazebo是一个**三维物理仿真平台**，用于模拟真实环境**生产数据** （与之相比，Rviz只是将数据可视化以另一种形式呈现，二者的功能本质是不同的）

Gazebo可以根据我们所提供的机器人模型文件，传感器配置参数，给机器人创造一个虚拟的环境，虚拟的电机和虚拟的传感器，并通过ROS/ROS2的相关功能包把传感器数据电机数据等发送出来（生产数据）。

**Gazebo 是一个独立的应用程序，可以独立于 ROS 或 ROS 2 使用。**

Gazebo与ROS 版本的集成是通过一组叫做 `gazebo_ros_pkgs`的包 完成的，`gazebo_ros_pkgs`将Gazebo和ROS2连接起来。

gazebo_ros_pkgs不是一个包，是一堆包如下：

* gazebo_dev：开发Gazebo插件可以用的API
* gazebo_msgs：定义的ROS2和Gazebo之间的接口（Topic/Service/Action）
* gazebo_ros：提供方便的 C++ 类和函数，可供其他插件使用，例如转换和测试实用程序。它还提供了一些通常有用的插件。gazebo_ros::Node
* gazebo_plugins：一系列 Gazebo 插件，将传感器和其他功能暴露给 ROS2 例如:
  1. `gazebo_ros_camera` 发布ROS2图像
  2. `gazebo_ros_diff_drive` 通过ROS2控制和获取两轮驱动机器人的接口

# 运用

因为ros2本身不会默认安装gazebo，所以需要先手动一下：

```
sudo apt install gazebo
```

(未完)
