# 关于动作的一些规则

话题适用于节点间单向频繁的数据传输，服务则适用于节点间双向的数据传递，而参数则用于动态调整节点的设置，那么动作则是基于以上几点的封装而形成的具有较完善的实时反馈机制，可用于实现完整动作流程控制的一种通信方式。

Action的通信双方是客户端节点与服务端节点；客户端节点中包含三个属性：Goal Service Client, Feedback Subscriber, Result Service Client;  服务端节点也包含三个属性：Goal Service Sever, Feedback Publisher, Result Service Sever.  

![1706184138967](image/node/1706184138967.png)

可以看见，Action有三大组成部分：目标(服务)，反馈(话题)，结果(服务)

* 目标：动作客户端告诉服务端要做什么，服务端对该目标要有响应，保证接收处理目标
* 反馈：动作服务端告诉客户端任务的进度，实现执行过程中的反馈
* 结果：动作服务端告诉客户端其执行结果，表示最终执行情况

# 使用CLI体验Action

同样启动小乌龟节点作为示例：

```
ros2 run turtlesim turtlesim_node
ros2 run turtlesim turtle_teleop_key
```

可以看见启动键盘之后有：

```
Use arrow keys to move the turtle.
Use G|B|V|C|D|E|R|T keys to rotate to absolute orientations. 'F' to cancel a rotation.
'Q' to quit.
```

其中 `arrow keys`是使用话题通信的（键盘节点turtle_teleop_key--->乌龟turtlesim_node）

而 `absolute orientations`(绝对旋转)是使用动作来控制的

可以看到按下G|B|V|C|D|E|R|T|F后收到反馈（来自服务端），而按方向键是没有这样的反馈的

```
zyt_brain@zhanyt:~$ ros2 run turtlesim turtlesim_node
QStandardPaths: wrong permissions on runtime directory /run/user/1000/, 0755 instead of 0700
[INFO] [1706181453.655375395] [turtlesim]: Starting turtlesim with node name /turtlesim
[INFO] [1706181453.660633298] [turtlesim]: Spawning turtle [turtle1] at x=[5.544445], y=[5.544445], theta=[0.000000]
[INFO] [1706185147.565410423] [turtlesim]: Rotation goal completed successfully
[INFO] [1706185156.621255383] [turtlesim]: Rotation goal completed successfully
[INFO] [1706185159.597759931] [turtlesim]: Rotation goal completed successfully
[INFO] [1706185165.567930794] [turtlesim]: Rotation goal completed successfully
[INFO] [1706185167.037427927] [turtlesim]: Rotation goal completed successfully
[INFO] [1706185168.749285652] [turtlesim]: Rotation goal canceled
[INFO] [1706185171.229237508] [turtlesim]: Rotation goal canceled
[INFO] [1706185174.797336300] [turtlesim]: Rotation goal canceled
[INFO] [1706185176.989191061] [turtlesim]: Rotation goal canceled
```

**下面来看一些常用的命令：**

查看目前的动作列表：

```
ros2 action list
```

```
zyt_brain@zhanyt:~$ ros2 action list
/turtle1/rotate_absolute
```

如果在list后加上-t，则可以看到action的类型（action自己也有接口类型）

```
ros2 action list -t
```

```
zyt_brain@zhanyt:~$ ros2 action list -t
/turtle1/rotate_absolute [turtlesim/action/RotateAbsolute]
```

已知接口类型，可以使用指令查看接口信息

```
ros2 interface show [type_of_interface]
```

```
zyt_brain@zhanyt:~$ ros2 interface show turtlesim/action/RotateAbsolute
# The desired heading in radians
float32 theta
---
# The angular displacement in radians to the starting position
float32 delta
---
# The remaining rotation in radians
float32 remaining
```

查看action信息：

```
ros2 action info <dir_of_action>
```

（ps .  `<dir_of_action>`是从action list命令得到的动作名称）

```
zyt_brain@zhanyt:~$ ros2 action info /turtle1/rotate_absolute
Action: /turtle1/rotate_absolute
Action clients: 1
    /teleop_turtle
Action servers: 1
    /turtlesim
```


**发送action请求到服务端**

```
ros2 action send_goal <dir_of_action> <typr_of_action> "{xxdataxx: val}"
```

```
zyt_brain@zhanyt:~$ ros2 action send_goal /turtle1/rotate_absolute turtlesim/action/RotateAbsolute "{theta: 0}"
Waiting for an action server to become available...
Sending goal:
     theta: 0.0

Goal accepted with ID: a57f21c93dc04685bd09d820484ae679

Result:
    delta: 2.3359999656677246

Goal finished with status: SUCCEEDED
```

*在上述命令后加上 ` --feedback`就可以收到任务执行过程中的实时反馈*



***ps.关于动作的编程实现此处暂不涉及，因为Action这种通信本身用得比较少，并且它目前还不在rclpy或rclcpp库当中；另外Action本身就是基于话题和服务的组合形成的，可以通过话题与服务理解动作的通信方式***
