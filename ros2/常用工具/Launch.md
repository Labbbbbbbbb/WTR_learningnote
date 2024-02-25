# Launch是什么

launch是ros2设计的具有完整语法和规则的类似于脚本的文件，用于同时启动和配置多个含有ros2节点的可执行文件，管理节点之间的依赖关系等等

Launch文件在ROS系统中出现的频次相当之高，它就像粘合剂一样，可以自由组装和配置各个节点

Launch的核心目的是 **启动节点** ，我们在命令行中输入的各种参数，在Launch文件中，通过类似这样的很多代码模版，也可以进行配置，甚至还可以使用Python原有的编程功能，大大丰富了启动过程中的多样化配置。

# Launch使用效果

尝试在终端中运行 `ros2 launch learning_launch simple.launch.py`

就可以看到它同时启动了两个节点（话题通信的双方）

```
zyt_brain@zhanyt:~$ ros2 launch learning_launch simple.launch.py
[INFO] [launch]: All log files can be found below /home/zyt_brain/.ros/log/2024-01-25-09-20-49-414516-zhanyt-288289
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [topic_helloworld_pub-1]: process started with pid [288290]
[INFO] [topic_helloworld_sub-2]: process started with pid [288292]
[topic_helloworld_pub-1] [INFO] [1706196050.678002007] [topic_helloworld_pub]: Publishing: "Hello World"
[topic_helloworld_sub-2] [INFO] [1706196050.678034496] [topic_helloworld_sub]: I heard: "Hello World"
[topic_helloworld_pub-1] [INFO] [1706196051.169136971] [topic_helloworld_pub]: Publishing: "Hello World"
[topic_helloworld_sub-2] [INFO] [1706196051.169517761] [topic_helloworld_sub]: I heard: "Hello World"
[topic_helloworld_pub-1] [INFO] [1706196051.669368623] [topic_helloworld_pub]: Publishing: "Hello World"
[topic_helloworld_sub-2] [INFO] [1706196051.670003510] [topic_helloworld_sub]: I heard: "Hello World"
```

可以来看一下这个launch文件的结构

```
from launch import LaunchDescription           # launch文件的描述类
from launch_ros.actions import Node            # 节点启动的描述类

def generate_launch_description():             # 自动生成launch文件的函数
    return LaunchDescription([                 # 返回launch文件的描述信息
        Node(                                  # 配置一个节点的启动
            package='learning_topic',          # 节点所在的功能包
            executable='topic_helloworld_pub', # 节点的可执行文件
        ),
        Node(                                  # 配置一个节点的启动
            package='learning_topic',          # 节点所在的功能包
            executable='topic_helloworld_sub', # 节点的可执行文件名
        ),
    ])

```

launch的原理就是把运行命令行是所输的命令和配置等都用python编写打包好放到一个文件里，直接运行这个文件即可完成所有命令


# 手搓ROS2的Launch文件

ros2的launch文件有python , xml , yaml 三种格式，其中python为官方推荐格式，所以此处也只（会）用python编写。

在功能包根目录(如village_li)下新建launch文件夹，在该文件夹下新建.launch.py文件，如village.launch.py，注意名字里要带上.launch，因为后面在setup.py里配置时使用通配符匹配的是.launch.py

代码逻辑主要有一下几部分

* 导入库
* 创建函数 `generate_launch_description()`（注意这里名字不能变，ros2会根据这个名字识别）
* 在函数内创建节点描述
* 生成 `LaunchDescription`对象并进行返回

```
#导入头文件
from launch import LaunchDescription
from launch_ros.actions import Node


 #launch内容描述函数,由ros2 launch 扫描调用
def generate_launch_description():  

    #创建Actions.Node对象lisi_node和wangwu_node
    lisi_node=Node(
          package="village_li",
          executable="lisi_node"
    )
    wangwu_node=Node(
          package="village_li",
          executable="wangwu_node"
     )
  
    #创建LaunchDescription对象launch_d,用于描述launch文件
    launch_d=LaunchDescription(lisi_node,wangwu_node)
    return launch_d
```

编写完成之后还需要***修改setup.py将launch文件拷贝到install目录下***

```
from setuptools import setup
from glob import glob
import os		#添加这两句

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')), #添加这一行，注意逗号
    ],
    },
)

```

然后就可以 `colcon build`啦，然后就可以在install目录下找到刚刚写的launch文件

当然如果是ament_cmake类型的功能包，配置方法也是不一样的，但是这里可以不用管噜

然后运行时可以直接用命令行运行这个封装好的launch文件，就可以优雅地跑节点啦


除此之外，如果想用launch文件配置参数，只需要在节点对象的 `executable=xxx`的下一行继续添加 `parameters=[{'<param_name>': param_val}]`即可修改参数值
