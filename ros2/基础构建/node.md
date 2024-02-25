# 节点---机器人的工作细胞

## **节点的本质 copyfrom guyue**

完整的机器人系统可能并不是一个物理上的整体，比如这样一个的机器人：

![image-20220526231417594](https://book.guyuehome.com/ROS2/2.%E6%A0%B8%E5%BF%83%E6%A6%82%E5%BF%B5/image/2.3_%E8%8A%82%E7%82%B9/image-20220526231417594.png)

在机器人身体里搭载了一台计算机A，它可以通过机器人的眼睛——摄像头，获取外界环境的信息，也可以控制机器人的腿——轮子，让机器人移动到想要去的地方。除此之外，可能还会有另外一台计算机B，放在你的桌子上，它可以远程监控机器人看到的信息，也可以远程配置机器人的速度和某些参数，还可以连接一个摇杆，人为控制机器人前后左右运动。

**这些功能虽然位于不同的计算机中，但都是这款机器人的工作细胞，也就是节点，他们共同组成了一个完整的机器人系统。**

* 节点在机器人系统中的职责就是 **执行某些具体的任务** ，从计算机操作系统的角度来看，也叫做进程；
* 每个节点都是一个可以 **独立运行的可执行文件** ，比如执行某一个python程序，或者执行C++编译生成的结果，都算是运行了一个节点；
* 既然每个节点都是独立的执行文件，那自然就可以想到，得到这个执行文件的 **编程语言可以是不同的** ，比如C++、Python，乃至Java、Ruby等更多语言。
* 这些节点是功能各不相同的细胞，根据系统设计的不同，可能位于计算机A，也可能位于计算机B，还有可能运行在云端，这叫做 **分布式** ，也就是可以分布在不同的硬件载体上；
* 每一个节点都需要有 **唯一的命名** ，当我们想要去找到某一个节点的时候，或者想要查询某一个节点的状态时，可以通过节点的名称来做查询。

节点也可以比喻是一个一个的工人，分别完成不同的任务，他们有的在一线厂房工作，有的在后勤部门提供保障，他们互相可能并不认识，但却一起推动机器人这座“工厂”，完成更为复杂的任务。

## 代码架构（python）

以node_helloworld的例程为例，代码如下：

```
#!/usr/bin/env python3 
# -*- coding: utf-8 -*-

"""
@作者: 古月居(www.guyuehome.com)
@说明: ROS2节点示例-发布“Hello World”日志信息, 使用面向过程的实现方式
"""

import rclpy                                     # ROS2 Python接口库
from rclpy.node import Node                      # ROS2 节点类
import time

def main(args=None):                             # ROS2节点主入口main函数
    rclpy.init(args=args)                        # ROS2 Python接口初始化
    node = Node("node_helloworld")               # 创建ROS2节点对象并进行初始化
  
    while rclpy.ok():                            # ROS2系统是否正常运行
        node.get_logger().info("Hello World")    # ROS2日志输出
        time.sleep(0.5)                          # 休眠控制循环时间
  
    node.destroy_node()                          # 销毁节点对象  
    rclpy.shutdown()                             # 关闭ROS2 Python接口

```

除此之外，为了能通过ros2 run命令运行代码，还需要再setup.py中配好程序入口：

```
from setuptools import setup

package_name = 'learning_node'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Hu Chunxu',
    maintainer_email='huchunxu@guyuehome.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
         'node_helloworld       = learning_node.node_helloworld:main',  #就是这一句，表示入口在learning						  
                                                                        #_node这个文件下node_helloworld
									#这个文件的main函数中
         'node_helloworld_class = learning_node.node_helloworld_class:main',
         'node_object            = learning_node.node_object:main',
         'node_object_webcam     = learning_node.node_object_webcam:main',
        ],
    },
)

```

> 以上是面向过程的编程，还有面向对象思想编程，但它们的编码流程都为：

* 编程接口初始化
* 创建节点并初始化
* 实现节点功能
* 销毁节点并关闭接口

## 节点运行

```
ros2 run <package_name> <executable_name>
```

指令意义：xx启动包下的xx节点

如 ros2 run learning_node node_helloworld 是learning_node文件夹下的node_helloworld.py文件所写的节点

典中典之体验小乌龟模拟器

```
ros2 run turtlesim turtlesim_node
```

再打开一个终端启动键盘

```
ros2 run turtlesim turtle_teleop_key
```


## 节点间的通信

**节点间交互的方式包括话题（topics），服务（services），动作（action），参数（parameter），其中最重要的就是话题**

### 话题：节点间传递数据的桥梁 cpfrom guyue

以两个机器人节点为例。A节点的功能是驱动相机这个硬件设备，获取得到相机拍摄的图像信息，B节点的功能是视频监控，将相机拍摄到的图像实时显示给用户查看。

![image-20220524141342311](https://book.guyuehome.com/ROS2/2.%E6%A0%B8%E5%BF%83%E6%A6%82%E5%BF%B5/image/2.4_%E8%AF%9D%E9%A2%98/image-20220524141342311.png)

大家可以想一下，这两个节点是不是必然存在某种关系？没错，节点A要将获取的图像数据传输给节点B，有了数据，节点B才能做这样可视化的渲染。

此时从节点A到节点B传递图像数据的方式，在ROS中，我们就称之为 **话题** ，它作为一个桥梁，实现了节点之间某一个方向上的数据传输。

#### **发布/订阅模型**

从话题本身的实现角度来看，使用了基于DDS的 **发布/订阅模型** ，什么叫发布和订阅呢？

![image8](https://book.guyuehome.com/ROS2/2.%E6%A0%B8%E5%BF%83%E6%A6%82%E5%BF%B5/image/2.4_%E8%AF%9D%E9%A2%98/image8.gif)

话题数据传输的特性是从一个节点到另外一个节点，发送数据的对象称之为 **发布者** ，接收数据的对象称之为 **订阅者** ，每一个话题都需要有一个名字，传输的数据也需要有固定的数据类型。

![image-20220527224626092](https://book.guyuehome.com/ROS2/2.%E6%A0%B8%E5%BF%83%E6%A6%82%E5%BF%B5/image/2.4_%E8%AF%9D%E9%A2%98/image-20220527224626092.png)

打一个比方，大家平时应该也会看微信公众号，比如有一个公众号，它的名字叫做“古月居”，这个古月居就是话题名称，公众号的发布者是古月居的小编，他会把组织好的机器人知识排版成要求格式的公众号文章，发布出去，这个文章格式，就是话题的数据类型。如果大家对这个话题感兴趣，就可以订阅“古月居”，成为订阅者之后自然就可以收到古月居的公众号文章，没有订阅的话，也就无法收到。

类似这样的发布/订阅模型在生活中随处可见，比如订阅报纸、订阅杂志等等。

#### **多对多通信**

![image9](https://book.guyuehome.com/ROS2/2.%E6%A0%B8%E5%BF%83%E6%A6%82%E5%BF%B5/image/2.4_%E8%AF%9D%E9%A2%98/image9.gif)

大家再仔细想下这些可以订阅的东西，是不是并不是唯一的，我们每个人可以订阅很多公众号、报纸、杂志，这些公众号、报纸、杂志也可以被很多人订阅，没错，ROS里的话题也是一样，发布者和订阅者的数量并不是唯一的，可以称之为是多对多的通信模型。

因为话题是多对多的模型，发布控制指令的摇杆可以有一个，也可以有2个、3个，订阅控制指令的机器人可以有1个，也可以有2个、3个，如果存在多个发送指令的节点，建议大家要 **注意区分优先级** ，不然机器人可能不知道该听谁的了。

#### **异步通信**

话题通信还有一个特性，那就是异步。所谓异步，只要是指发布者发出数据后，并不知道订阅者什么时候可以收到，类似古月居公众号发布一篇文章，你什么时候阅读的，古月居根本不知道，报社发出一份报纸，你什么时候收到，报社也是不知道的。这就叫做异步。

异步的特性也让话题更适合用于一些周期发布的数据，比如传感器的数据，运动控制的指令等等，如果某些逻辑性较强的指令，比如修改某一个参数，用话题传输就不太合适了。

#### **消息---话题中数据的统一描述格式**

最后，既然是数据传输，发布者和订阅者就得统一数据的描述格式，不能一个说英文，一个理解成了中文。在ROS中，**话题通信数据的描述格式称之为消息**，对应编程语言中数据结构的概念。比如这里的一个图像数据，就会包含图像的长宽像素值、每个像素的RGB等等，在ROS中都有标准定义。

 **消息是ROS中的一种接口定义方式** ，与编程语言无关，我们也可以通过.msg后缀的文件自行定义，有了这样的接口，各种节点就像积木块一样，通过各种各样的接口进行拼接，组成复杂的机器人系统。

---

#### **案例一：Hello World话题通信**

了解了话题的基本原理，接下来我们就要开始编写代码啦。

![image-20220524141514506](https://book.guyuehome.com/ROS2/2.%E6%A0%B8%E5%BF%83%E6%A6%82%E5%BF%B5/image/2.4_%E8%AF%9D%E9%A2%98/image-20220524141514506.png)

还是从Hello World例程开始，我们来创建一个发布者，发布话题“chatter”，周期发送“Hello World”这个字符串，消息类型是ROS中标准定义的String，再创建一个订阅者，订阅“chatter”这个话题，从而接收到“Hello World”这个字符串。

##### **运行效果**

启动第一个终端，运行话题的发布者节点：

```
$ ros2 run learning_topic topic_helloworld_pub
```

![image-20220524141606868](https://book.guyuehome.com/ROS2/2.%E6%A0%B8%E5%BF%83%E6%A6%82%E5%BF%B5/image/2.4_%E8%AF%9D%E9%A2%98/image-20220524141606868.png)

启动第二个终端，运行话题的订阅者节点：

```
$ ros2 run learning_topic topic_helloworld_sub
```

![image-20220524141614633](https://book.guyuehome.com/ROS2/2.%E6%A0%B8%E5%BF%83%E6%A6%82%E5%BF%B5/image/2.4_%E8%AF%9D%E9%A2%98/image-20220524141614633.png)

可以看到发布者循环发布“Hello World”字符串消息，订阅者也以几乎同样的频率收到该话题的消息数据。

##### **发布者代码解析**

我们来看下发布者的实现方法。

###### 程序实现（面向对象）

learning_topic/topic_helloworld_pub.py

```
#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
@作者: 古月居(www.guyuehome.com)
@说明: ROS2话题示例-发布“Hello World”话题
"""

import rclpy                                     # ROS2 Python接口库
from rclpy.node import Node                      # ROS2 节点类
from std_msgs.msg import String                  # 字符串消息类型

"""
创建一个发布者节点
"""
class PublisherNode(Node):

    def __init__(self, name):
        super().__init__(name)                                    # ROS2节点父类初始化
        self.pub = self.create_publisher(String, "chatter", 10)   # 创建发布者对象（消息类型、话题名、队列长度）
        self.timer = self.create_timer(0.5, self.timer_callback)  # 创建一个定时器（单位为秒的周期，定时执行的回调函数）

    def timer_callback(self):                                     # 创建定时器周期执行的回调函数
        msg = String()                                            # 创建一个String类型的消息对象
        msg.data = 'Hello World'                                  # 填充消息对象中的消息数据
        self.pub.publish(msg)                                     # 发布话题消息
        self.get_logger().info('Publishing: "%s"' % msg.data)     # 输出日志信息，提示已经完成话题发布

def main(args=None):                                 # ROS2节点主入口main函数
    rclpy.init(args=args)                            # ROS2 Python接口初始化
    node = PublisherNode("topic_helloworld_pub")     # 创建ROS2节点对象并进行初始化
    rclpy.spin(node)                                 # 循环等待ROS2退出
    node.destroy_node()                              # 销毁节点对象
    rclpy.shutdown()                                 # 关闭ROS2 Python接口
```

完成代码的编写后需要设置功能包的编译选项，让系统知道Python程序的入口，打开功能包的setup.py文件，加入如下入口点的配置：

```
    entry_points={
        'console_scripts': [
         'topic_helloworld_pub  = learning_topic.topic_helloworld_pub:main',
        ],
    },
```

###### 流程总结

对以上程序进行分析，如果我们想要实现一个发布者，流程如下：

* 编程接口初始化
* 创建节点并初始化
* 创建发布者对象
* 创建并填充话题消息
* 发布话题消息
* 销毁节点并关闭接口

##### **订阅者代码解析**

我们再来看下订阅者的实现方法。

###### 程序实现（面向对象）

learning_topic/topic_helloworld_sub.py

```
#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
@作者: 古月居(www.guyuehome.com)
@说明: ROS2话题示例-订阅“Hello World”话题消息
"""

import rclpy                      # ROS2 Python接口库
from rclpy.node   import Node     # ROS2 节点类
from std_msgs.msg import String   # ROS2标准定义的String消息
				#向该话题的.msg中载入String类型
"""
创建一个订阅者节点
"""
class SubscriberNode(Node):

    def __init__(self, name):
        super().__init__(name)                             # ROS2节点父类初始化
        self.sub = self.create_subscription(\
            String, "chatter", self.listener_callback, 10) # 创建订阅者对象（消息类型、话题名、订阅者回调函数、队列长度）

    def listener_callback(self, msg):                      # 创建回调函数，执行收到话题消息后对数据的处理
        self.get_logger().info('I heard: "%s"' % msg.data) # 输出日志信息，提示订阅收到的话题消息

def main(args=None):                               # ROS2节点主入口main函数
    rclpy.init(args=args)                          # ROS2 Python接口初始化
    node = SubscriberNode("topic_helloworld_sub")  # 创建ROS2节点对象并进行初始化
    rclpy.spin(node)                               # 循环等待ROS2退出
    node.destroy_node()                            # 销毁节点对象
    rclpy.shutdown()                               # 关闭ROS2 Python接口
```

完成代码的编写后需要设置功能包的编译选项，让系统知道Python程序的入口，打开功能包的setup.py文件，加入如下入口点的配置：

```
    entry_points={
        'console_scripts': [
         'topic_helloworld_pub  = learning_topic.topic_helloworld_pub:main',
         'topic_helloworld_sub  = learning_topic.topic_helloworld_sub:main',
        ],
    },
```

###### 流程总结

对以上程序进行分析，如果我们想要实现一个订阅者，流程如下：

* 编程接口初始化
* 创建节点并初始化
* 创建订阅者对象
* 回调函数处理话题数据
* 销毁节点并关闭接口

### 服务：节点间的你问我答

在之前的课程中，我们通过一个节点驱动相机，发布图像话题，另外一个节点订阅图像话题，并实现对其中红色物体的识别，此时我们可以按照图像识别的频率，周期得到物体的位置。

![image-20220527232959464](https://book.guyuehome.com/ROS2/2.%E6%A0%B8%E5%BF%83%E6%A6%82%E5%BF%B5/image/2.5_%E6%9C%8D%E5%8A%A1/image-20220527232959464.png)

这个位置信息可以继续发给机器人的上层应用使用，比如可以跟随目标运动，或者运动到目标位置附近。此时，我们并不需要这么高的频率一直订阅物体的位置，而是更希望在需要这个数据的时候，发一个查询的请求，然后尽快得到此时目标的最新位置。

这样的通信模型和话题单向传输有所不同，变成了发送一个请求，反馈一个应答的形式，好像是你问我答一样，这种通信机制在ROS中成为 **服务，Service** 。

#### **客户端/服务器模型**

从服务的实现机制上来看，这种你问我答的形式叫做 **客户端/服务器模型** ，简称为CS模型，客户端在需要某些数据的时候，针对某个具体的服务，发送请求信息，服务器端收到请求之后，就会进行处理并反馈应答信息。

![image8](https://book.guyuehome.com/ROS2/2.%E6%A0%B8%E5%BF%83%E6%A6%82%E5%BF%B5/image/2.5_%E6%9C%8D%E5%8A%A1/image8.gif)

这种通信机制在生活中也很常见，比如我们经常浏览的各种网页，此时你的电脑浏览器就是客户端，通过域名或者各种操作，向网站服务器发送请求，服务器收到之后返回需要展现的页面数据。

#### **同步通信**

这个过程一般要求越快越好，假设服务器半天没有反应，你的浏览器一直转圈圈，那有可能是服务器宕机了，或者是网络不好，所以相比话题通信，在服务通信中，客户端可以通过接收到的应答信息，判断服务器端的状态，我们也称之为同步通信。

#### **一对多通信**

![image9](https://book.guyuehome.com/ROS2/2.%E6%A0%B8%E5%BF%83%E6%A6%82%E5%BF%B5/image/2.5_%E6%9C%8D%E5%8A%A1/image9.gif)

比如古月居这个网站，服务器是唯一存在的，并没有多个完全一样的古月居网站，但是可以访问古月居网站的客户端是不唯一的，大家每一个人都可以看到同样的界面。所以服务通信模型中，服务器端唯一，但客户端可以不唯一。

#### **服务接口**

和话题通信类似，服务通信的核心还是要传递数据，数据变成了两个部分，一个 **请求的数据** ，比如请求苹果位置的命令，还有一个 **反馈的数据** ，比如反馈苹果坐标位置的数据，这些数据和话题消息一样，在ROS中也是要标准定义的，话题使用.msg文件定义，服务使用的是.srv文件定义，后续我们会给大家介绍定义的方法。

#### **案例一：加法求解器**

大家现在对ROS服务通信应该有了基本了解，接下来我们就要开始编写代码啦。还是从一个相对简单的例程开始，也是ROS官方的一个例程，通过服务实现一个加法求解器的功能。

![image-20220527233716400](https://book.guyuehome.com/ROS2/2.%E6%A0%B8%E5%BF%83%E6%A6%82%E5%BF%B5/image/2.5_%E6%9C%8D%E5%8A%A1/image-20220527233716400.png)

当我们需要计算两个加数的求和结果时，就通过客户端节点，将两个加数封装成请求数据，针对服务“add_two_ints”发送出去，提供这个服务的服务器端节点，收到请求数据后，开始进行加法计算，并将求和结果封装成应答数据，反馈给客户端，之后客户端就可以得到想要的结果啦。

##### **运行效果**

我们一起操作下这个例程，并且看下代码的实现原理。

启动两个终端，并运行如下节点，第一个节点是服务端，等待请求数据并提供求和功能，第二个节点是客户端，发送传入的两个加数并等待求和结果。

```
$ ros2 run learning_service service_adder_server
$ ros2 run learning_service service_adder_client 2 3
```

![image-20220527233928009](https://book.guyuehome.com/ROS2/2.%E6%A0%B8%E5%BF%83%E6%A6%82%E5%BF%B5/image/2.5_%E6%9C%8D%E5%8A%A1/image-20220527233928009.png)

![image-20220527233916665](https://book.guyuehome.com/ROS2/2.%E6%A0%B8%E5%BF%83%E6%A6%82%E5%BF%B5/image/2.5_%E6%9C%8D%E5%8A%A1/image-20220527233916665.png)

##### **客户端代码解析**

我们来看下客户端的实现方法。

###### **程序实现**

learning_service/service_adder_client.py

```
#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
@作者: 古月居(www.guyuehome.com)
@说明: ROS2服务示例-发送两个加数，请求加法器计算
"""

import sys

import rclpy                                  # ROS2 Python接口库
from rclpy.node   import Node                 # ROS2 节点类
from learning_interface.srv import AddTwoInts # 自定义的服务接口

class adderClient(Node):
    def __init__(self, name):
        super().__init__(name)                                       # ROS2节点父类初始化
        self.client = self.create_client(AddTwoInts, 'add_two_ints') # 创建服务客户端对象（服务接口类型，服务名）
        while not self.client.wait_for_service(timeout_sec=1.0):     # 循环等待服务器端成功启动
            self.get_logger().info('service not available, waiting again...') 
        self.request = AddTwoInts.Request()                          # 创建服务请求的数据对象

    def send_request(self):                                          # 创建一个发送服务请求的函数
        self.request.a = int(sys.argv[1])
        self.request.b = int(sys.argv[2])
        self.future = self.client.call_async(self.request)           # 异步方式发送服务请求

def main(args=None):
    rclpy.init(args=args)                        # ROS2 Python接口初始化
    node = adderClient("service_adder_client")   # 创建ROS2节点对象并进行初始化
    node.send_request()                          # 发送服务请求

    while rclpy.ok():                            # ROS2系统正常运行
        rclpy.spin_once(node)                    # 循环执行一次节点

        if node.future.done():                   # 数据是否处理完成
            try:
                response = node.future.result()  # 接收服务器端的反馈数据
            except Exception as e:
                node.get_logger().info(
                    'Service call failed %r' % (e,))
            else:
                node.get_logger().info(          # 将收到的反馈信息打印输出
                    'Result of add_two_ints: for %d + %d = %d' % 
                    (node.request.a, node.request.b, response.sum))
            break

    node.destroy_node()                          # 销毁节点对象
    rclpy.shutdown()                             # 关闭ROS2 Python接口
```

完成代码的编写后需要设置功能包的编译选项，让系统知道Python程序的入口，打开功能包的setup.py文件，加入如下入口点的配置：

```
    entry_points={
        'console_scripts': [
         'service_adder_client  = learning_service.service_adder_client:main',
        ],
    },
```

###### **流程总结**

对以上程序进行分析，如果我们想要实现一个客户端，流程如下：

* 编程接口初始化
* 创建节点并初始化
* 创建客户端对象
* 创建并发送请求数据
* 等待服务器端应答数据
* 销毁节点并关闭接口

##### **服务端代码解析**

至于服务器端的实现，有点类似话题通信中的订阅者，并不知道请求数据什么时间出现，也用到了回调函数机制。

###### **程序实现**

learning_service/service_adder_server.py

```
#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
@作者: 古月居(www.guyuehome.com)
@说明: ROS2服务示例-提供加法器的服务器处理功能
"""

import rclpy                                     # ROS2 Python接口库
from rclpy.node   import Node                    # ROS2 节点类
from learning_interface.srv import AddTwoInts    # 自定义的服务接口

class adderServer(Node):
    def __init__(self, name):
        super().__init__(name)                                                           # ROS2节点父类初始化
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.adder_callback)  # 创建服务器对象（接口类型、服务名、服务器回调函数）

    def adder_callback(self, request, response):   # 创建回调函数，执行收到请求后对数据的处理
        response.sum = request.a + request.b       # 完成加法求和计算，将结果放到反馈的数据中
        self.get_logger().info('Incoming request\na: %d b: %d' % (request.a, request.b))   # 输出日志信息，提示已经完成加法求和计算
        return response                          # 反馈应答信息

def main(args=None):                             # ROS2节点主入口main函数
    rclpy.init(args=args)                        # ROS2 Python接口初始化
    node = adderServer("service_adder_server")   # 创建ROS2节点对象并进行初始化
    rclpy.spin(node)                             # 循环等待ROS2退出
    node.destroy_node()                          # 销毁节点对象
    rclpy.shutdown()                             # 关闭ROS2 Python接口
```

完成代码的编写后需要设置功能包的编译选项，让系统知道Python程序的入口，打开功能包的setup.py文件，加入如下入口点的配置：

```
    entry_points={
        'console_scripts': [
         'service_adder_client  = learning_service.service_adder_client:main',
         'service_adder_server  = learning_service.service_adder_server:main',
        ],
    },
```

###### **流程总结**

对以上程序进行分析，如果我们想要实现一个服务端，流程如下：

* 编程接口初始化
* 创建节点并初始化
* 创建服务器端对象
* 通过回调函数处进行服务
* 向客户端反馈应答结果
* 销毁节点并关闭接口

### 动作：完整行为的流程管理

机器人是一个复杂的智能系统，并不仅仅是键盘遥控运动、识别某个目标这么简单，我们需要实现的是送餐、送货、分拣等满足具体场景需求的机器人。

在这些应用功能的实现中，另外一种ROS通信机制也会被常常用到——那就是 **动作** 。从这个名字上就可以很好理解这个概念的含义，这种通信机制的目的就是便于 **对机器人某一完整行为的流程进行管理** 。

#### **通信模型**

举个例子，比如我们想让机器人转个圈，这肯定不是一下就可以完成的，机器人得一点一点旋转，直到360度才能结束，假设机器人并不在我们眼前，发出指令后，我们根本不知道机器人到底有没有开始转圈，转到哪里了？

![image-20220528005012082](https://book.guyuehome.com/ROS2/2.%E6%A0%B8%E5%BF%83%E6%A6%82%E5%BF%B5/image/2.7_%E5%8A%A8%E4%BD%9C/image-20220528005012082.png)

OK，现在我们需要的是一个反馈，比如每隔1s，告诉我们当前转到多少度了，10度、20度、30度，一段时间之后，到了360度，再发送一个信息，表示动作执行完成。

这样一个需要执行一段时间的行为，使用动作的通信机制就更为合适，就像装了一个进度条，我们可以随时把控进度，如果运动过程当中，我们还可以随时发送一个取消运动的命令。

##### **客户端/服务器模型**

动作和服务类似，使用的也是客户端和服务器模型，客户端发送动作的目标，想让机器人干什么，服务器端执行动作过程， 控制机器人达到运动的目标，同时周期反馈动作执行过程中的状态。

![1706184138967](image/node/1706184138967.png)

客户端发送一个运动的目标，想让机器人动起来，服务器端收到之后，就开始控制机器人运动，一边运动，一边反馈当前的状态，如果是一个导航动作，这个反馈可能是当前所处的坐标，如果是机械臂抓取，这个反馈可能又是机械臂的实时姿态。当运动执行结束后，服务器再反馈一个动作结束的信息。整个通信过程就此结束。

##### **一对多通信**

和服务一样，动作通信中的客户端可以有多个，大家都可以发送运动命令，但是服务器端只能有一个，毕竟只有一个机器人，先执行完成一个动作，才能执行下一个动作。

##### **同步通信**

既然有反馈，那动作也是一种同步通信机制，之前我们也介绍过，动作过程中的数据通信接口，使用.action文件进行定义。

##### **由服务和话题合成**

大家再仔细看下上边的动图，是不是还会发现一个隐藏的秘密。

动作的三个通信模块，竟然有两个是服务，一个是话题，当客户端发送运动目标时，使用的是服务的请求调用，服务器端也会反馈一个应带，表示收到命令。动作的反馈过程，其实就是一个话题的周期发布，服务器端是发布者，客户端是订阅者。

没错，动作是一种应用层的通信机制，其底层就是基于话题和服务来实现的。

#### **案例一：小海龟的动作**

我们再用小海龟的案例加深对动作概念的理解。

按照以下命令启动小海龟仿真器，接下来使用action命令控制小海龟的动作，可以让海龟运动到某一指定的姿态：

```
$ ros2 run turtlesim turtlesim_node
$ ros2 run turtlesim turtle_teleop_key
$ ros2 action info /turtle1/rotate_absolute
$ ros2 action send_goal /turtle1/rotate_absolute turtlesim/action/RotateAbsolute "{theta: -1.57}"
$ ros2 action send_goal /turtle1/rotate_absolute turtlesim/action/RotateAbsolute "{theta: -1.57}" --feedback
```

![image-20220528005909408](https://book.guyuehome.com/ROS2/2.%E6%A0%B8%E5%BF%83%E6%A6%82%E5%BF%B5/image/2.7_%E5%8A%A8%E4%BD%9C/image-20220528005909408.png)

![image-20220528005935269](https://book.guyuehome.com/ROS2/2.%E6%A0%B8%E5%BF%83%E6%A6%82%E5%BF%B5/image/2.7_%E5%8A%A8%E4%BD%9C/image-20220528005935269.png)

#### **案例二：机器人画圆**

如何通过代码来实现动作的编程呢？

动作虽然是基于话题和服务实现的，但在实际使用中，并不会直接使用话题和服务的编程方法，而是有一套针对动作特性封装好的编程接口，接下来我们就一起试一试。

![image-20220528010032315](https://book.guyuehome.com/ROS2/2.%E6%A0%B8%E5%BF%83%E6%A6%82%E5%BF%B5/image/2.7_%E5%8A%A8%E4%BD%9C/image-20220528010032315.png)

假设我们有一个机器人，我们希望通过动作的通信方法，让机器人转个圈，请编程实现动作通信中，客户端和服务器端的实现过程。

##### **运行效果**

启动两个终端，分别运行一下命令，启动动作示例的服务端和客户端：

```
$ ros2 run learning_action action_move_server 
$ ros2 run learning_action action_move_client
```

![image-20220528010905277](https://book.guyuehome.com/ROS2/2.%E6%A0%B8%E5%BF%83%E6%A6%82%E5%BF%B5/image/2.7_%E5%8A%A8%E4%BD%9C/image-20220528010905277.png)

![image-20220528010845943](https://book.guyuehome.com/ROS2/2.%E6%A0%B8%E5%BF%83%E6%A6%82%E5%BF%B5/image/2.7_%E5%8A%A8%E4%BD%9C/image-20220528010845943.png)

终端中，我们可以看到客户端发送动作目标之后，服务器端开始模拟机器人运动，每30度发送一次反馈信息，最终完成运动，并反馈结束运动的信息。

接下来我们就分析下这个例程实现背后的原理。

##### **接口定义**

例程使用的动作并不是ROS中的标准定义，我们通过MoveCircle.action进行自定义：

learning_interface/action/MoveCircle.action

```
bool enable# 定义动作的目标，表示动作开始的指令
---
bool finish# 定义动作的结果，表示是否成功执行
---
int32 state# 定义动作的反馈，表示当前执行到的位置
```

包含三个部分：

* 第一块是 **动作的目标** ，enable为true时，表示开始运动；
* 第二块是 **动作的执行结果** ，finish为true，表示动作执行完成；
* 第三块是 **动作的周期反馈** ，表示当前机器人旋转到的角度。

完成定义后，还需要在功能包的CMakeLists.txt中配置编译选项，让编译器在编译过程中，根据接口定义，自动生成不同语言的代码：

```
...

find_package(rosidl_default_generatorsREQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
"action/MoveCircle.action"
)

...
```

##### **通信模型**

通信模型就是这样，客户端发送给一个动作目标，服务器控制机器人开始运动，并周期反馈，结束后反馈结束信息。

![image-20220528010217043](https://book.guyuehome.com/ROS2/2.%E6%A0%B8%E5%BF%83%E6%A6%82%E5%BF%B5/image/2.7_%E5%8A%A8%E4%BD%9C/image-20220528010217043.png)

思路理清楚，接下来开始写代码。相比之前话题和服务的程序，动作通信的例程相对较长，我们一起来运行并分析一下。

##### **服务端代码解析**

learning_action/action_move_server.py

```
#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
@作者: 古月居(www.guyuehome.com)
@说明: ROS2动作示例-负责执行圆周运动动作的服务端
"""

import time

import rclpy                                      # ROS2 Python接口库
from rclpy.node   import Node                     # ROS2 节点类
from rclpy.action import ActionServer             # ROS2 动作服务器类
from learning_interface.action import MoveCircle  # 自定义的圆周运动接口

class MoveCircleActionServer(Node):
    def __init__(self, name):
        super().__init__(name)                   # ROS2节点父类初始化
        self._action_server = ActionServer(      # 创建动作服务器（接口类型、动作名、回调函数）
            self,
            MoveCircle,
            'move_circle',
            self.execute_callback)

    def execute_callback(self, goal_handle):            # 执行收到动作目标之后的处理函数
        self.get_logger().info('Moving circle...')
        feedback_msg = MoveCircle.Feedback()            # 创建一个动作反馈信息的消息

        for i in range(0, 360, 30):                     # 从0到360度，执行圆周运动，并周期反馈信息
            feedback_msg.state = i                      # 创建反馈信息，表示当前执行到的角度
            self.get_logger().info('Publishing feedback: %d' % feedback_msg.state)
            goal_handle.publish_feedback(feedback_msg)  # 发布反馈信息
            time.sleep(0.5)

        goal_handle.succeed()                           # 动作执行成功
        result = MoveCircle.Result()                    # 创建结果消息
        result.finish = True                        
        return result                                   # 反馈最终动作执行的结果

def main(args=None):                                    # ROS2节点主入口main函数
    rclpy.init(args=args)                               # ROS2 Python接口初始化
    node = MoveCircleActionServer("action_move_server") # 创建ROS2节点对象并进行初始化
    rclpy.spin(node)                                    # 循环等待ROS2退出
    node.destroy_node()                                 # 销毁节点对象
    rclpy.shutdown()                                    # 关闭ROS2 Python接口
```

完成代码的编写后需要设置功能包的编译选项，让系统知道Python程序的入口，打开功能包的setup.py文件，加入如下入口点的配置：

```
    entry_points={
        'console_scripts': [
         'action_move_server    = learning_action.action_move_server:main',
        ],
    },
```

##### **客户端代码解析**

learning_action/action_move_client.py

```
#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
@作者: 古月居(www.guyuehome.com)
@说明: ROS2动作示例-请求执行圆周运动动作的客户端
"""

import rclpy                                      # ROS2 Python接口库
from rclpy.node   import Node                     # ROS2 节点类
from rclpy.action import ActionClient             # ROS2 动作客户端类

from learning_interface.action import MoveCircle  # 自定义的圆周运动接口

class MoveCircleActionClient(Node):
    def __init__(self, name):
        super().__init__(name)                   # ROS2节点父类初始化
        self._action_client = ActionClient(      # 创建动作客户端（接口类型、动作名）
            self, MoveCircle, 'move_circle') 

    def send_goal(self, enable):                 # 创建一个发送动作目标的函数
        goal_msg = MoveCircle.Goal()             # 创建一个动作目标的消息
        goal_msg.enable = enable                 # 设置动作目标为使能，希望机器人开始运动

        self._action_client.wait_for_server()    # 等待动作的服务器端启动
        self._send_goal_future = self._action_client.send_goal_async(   # 异步方式发送动作的目标
            goal_msg,                                                   # 动作目标
            feedback_callback=self.feedback_callback)                   # 处理周期反馈消息的回调函数

        self._send_goal_future.add_done_callback(self.goal_response_callback) # 设置一个服务器收到目标之后反馈时的回调函数

    def goal_response_callback(self, future):           # 创建一个服务器收到目标之后反馈时的回调函数
        goal_handle = future.result()                   # 接收动作的结果
        if not goal_handle.accepted:                    # 如果动作被拒绝执行
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')                            # 动作被顺利执行

        self._get_result_future = goal_handle.get_result_async()              # 异步获取动作最终执行的结果反馈
        self._get_result_future.add_done_callback(self.get_result_callback)   # 设置一个收到最终结果的回调函数 

    def get_result_callback(self, future):                                    # 创建一个收到最终结果的回调函数
        result = future.result().result                                       # 读取动作执行的结果
        self.get_logger().info('Result: {%d}' % result.finish)                # 日志输出执行结果

    def feedback_callback(self, feedback_msg):                                # 创建处理周期反馈消息的回调函数
        feedback = feedback_msg.feedback                                      # 读取反馈的数据
        self.get_logger().info('Received feedback: {%d}' % feedback.state) 

def main(args=None):                                       # ROS2节点主入口main函数
    rclpy.init(args=args)                                  # ROS2 Python接口初始化
    node = MoveCircleActionClient("action_move_client")    # 创建ROS2节点对象并进行初始化
    node.send_goal(True)                                   # 发送动作目标
    rclpy.spin(node)                                       # 循环等待ROS2退出
    node.destroy_node()                                    # 销毁节点对象
    rclpy.shutdown()                                       # 关闭ROS2 Python接口
```

完成代码的编写后需要设置功能包的编译选项，让系统知道Python程序的入口，打开功能包的setup.py文件，加入如下入口点的配置：

```
    entry_points={
        'console_scripts': [
         'action_move_client    = learning_action.action_move_client:main',
         'action_move_server    = learning_action.action_move_server:main',
        ],
    },
```

#### **动作命令行操作**

动作命令的常用操作如下：

```
$ ros2 action list                  # 查看服务列表
$ ros2 action info <action_name>    # 查看服务数据类型
$ ros2 action send_goal <action_name> <action_type> <action_data>   # 发送服务请求
```

![image-20220528005935269](https://book.guyuehome.com/ROS2/2.%E6%A0%B8%E5%BF%83%E6%A6%82%E5%BF%B5/image/2.7_%E5%8A%A8%E4%BD%9C/image-20220528005935269.png)

### 参数：机器人系统的全局字典

类似C++编程中的全局变量，可以便于在多个程序中共享某些数据，**参数是ROS机器人系统中的全局字典，可以运行多个节点中共享数据。**

#### **通信模型**

比如在机器视觉识别的时候，有很多参数都会影响视觉识别的效果。

![image-20220528013439656](https://book.guyuehome.com/ROS2/2.%E6%A0%B8%E5%BF%83%E6%A6%82%E5%BF%B5/image/2.8_%E5%8F%82%E6%95%B0/image-20220528013439656.png)

在NodeA相机驱动节点中，就需要考虑很多问题，相机连接到哪个usb端口，使用的图像分辨率是多少，曝光度和编码格式分别是什么，这些都可以通过参数设置，我们可以通过配置文件或者程序进行设置。

NodeB节点中也是一样，图像识别使用的阈值是多少，整个图像面积很大，那个部分是我们关注的核心区域，识别过程是否需要美颜等等，就像我们使用美颜相机一样，我们可以通过滑动条或者输入框设置很多参数，不同参数设置后，都会改变执行功能的一些效果。

这就是参数的作用。

##### **全局字典**

在ROS系统中，参数是以**全局字典**的形态存在的，什么叫字典？就像真实的字典一样，由名称和数值组成，也叫做键和值，合成 **键值** 。或者我们也可以理解为，就像编程中的参数一样，有一个参数名 ，然后跟一个等号，后边就是参数值了，在使用的时候，访问这个参数名即可。

##### **可动态监控**

在ROS2中，参数的特性非常丰富，比如某一个节点共享了一个参数，其他节点都可以访问，如果某一个节点对参数进行了修改，其他节点也有办法立刻知道，从而获取最新的数值。这在**参数的高级编程**中，大家都可能会用到。

#### **案例一：小海龟例程中的参数**

在小海龟的例程中，仿真器也提供了不少参数，我们一起来通过这个例程，熟悉下参数的含义和命令行的使用方法。

启动两个终端，分别运行小海龟仿真器和键盘控制节点：

```
$ ros2 run turtlesim turtlesim_node
$ ros2 run turtlesim turtle_teleop_key
```

![image-20220528013639558](https://book.guyuehome.com/ROS2/2.%E6%A0%B8%E5%BF%83%E6%A6%82%E5%BF%B5/image/2.8_%E5%8F%82%E6%95%B0/image-20220528013639558.png)

##### **查看参数列表**

当前系统中有哪些参数呢？我们可以启动一个终端，并使用如下命令查询：

```
$ ros2 param list
```

![image-20220528013700730](https://book.guyuehome.com/ROS2/2.%E6%A0%B8%E5%BF%83%E6%A6%82%E5%BF%B5/image/2.8_%E5%8F%82%E6%95%B0/image-20220528013700730-16536730268421.png)

##### **参数查询与修改**

如果想要查询或者修改某个参数的值，可以在param命令后边跟get或者set子命令：

```
$ ros2 param describe turtlesim background_b   # 查看某个参数的描述信息
$ ros2 param get turtlesim background_b        # 查询某个参数的值
$ ros2 param set turtlesim background_b 10     # 修改某个参数的值
```

##### **参数文件保存与加载**

一个一个查询/修改参数太麻烦了，不如试一试参数文件，ROS中的参数文件使用yaml格式，可以在param命令后边跟dump子命令，将某个节点的参数都保存到文件中，或者通过load命令一次性加载某个参数文件中的所有内容：

```
$ ros2 param dump turtlesim >> turtlesim.yaml  # 将某个节点的参数保存到参数文件中
#dump命令会把节点中所有的参数进行保存，>>用于重定向存放到某个文件中（不加>>以后的内容的话会打印到终端上）
$ ros2 param load turtlesim turtlesim.yaml     # 一次性加载某一个文件中的所有参数（相当于刷新）
```

![image-20220528013752068](https://book.guyuehome.com/ROS2/2.%E6%A0%B8%E5%BF%83%E6%A6%82%E5%BF%B5/image/2.8_%E5%8F%82%E6%95%B0/image-20220528013752068.png)

#### **案例二：参数编程**

接下来就要开始写程序了，在程序中设置参数和读取参数都比较简单，一两句函数就可以实现，我们先来体验一下这几个函数的使用方法。

##### **运行效果**

启动一个终端，先运行第一句指令，启动param_declare节点，终端中可以看到循环打印的日志信息，其中的“mbot”就是我们设置的一个参数值，参数名称是“robot_name”，通过命令行修改这个参数，看下终端中会发生什么？

```
$ ros2 run learning_parameter param_declare
$ ros2 param set param_declare robot_name turtle
```

![image-20220528014153014](https://book.guyuehome.com/ROS2/2.%E6%A0%B8%E5%BF%83%E6%A6%82%E5%BF%B5/image/2.8_%E5%8F%82%E6%95%B0/image-20220528014153014.png)

##### **代码解析**

我们来看下在代码中，如何声明、创建、修改一个参数的值。

learning_parameter/param_declare.py

```
#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
@作者: 古月居(www.guyuehome.com)
@说明: ROS2参数示例-创建、读取、修改参数
"""

import rclpy                                     # ROS2 Python接口库
from rclpy.node   import Node                    # ROS2 节点类

class ParameterNode(Node):
    def __init__(self, name):
        super().__init__(name)                                    # ROS2节点父类初始化
        self.timer = self.create_timer(2, self.timer_callback)    # 创建一个定时器（单位为秒的周期，定时执行的回调函数）
        self.declare_parameter('robot_name', 'mbot')              # 创建一个参数，并设置参数的默认值

    def timer_callback(self):                                      # 创建定时器周期执行的回调函数
        robot_name_param = self.get_parameter('robot_name').get_parameter_value().string_value   # 从ROS2系统中读取参数的值

        self.get_logger().info('Hello %s!' % robot_name_param)     # 输出日志信息，打印读取到的参数值

        new_name_param = rclpy.parameter.Parameter('robot_name',   # 重新将参数值设置为指定值
                            rclpy.Parameter.Type.STRING, 'mbot')
        all_new_parameters = [new_name_param]
        self.set_parameters(all_new_parameters)                    # 将重新创建的参数列表发送给ROS2系统

def main(args=None):                                 # ROS2节点主入口main函数
    rclpy.init(args=args)                            # ROS2 Python接口初始化
    node = ParameterNode("param_declare")            # 创建ROS2节点对象并进行初始化
    rclpy.spin(node)                                 # 循环等待ROS2退出
    node.destroy_node()                              # 销毁节点对象
    rclpy.shutdown()                                 # 关闭ROS2 Python接口
```

完成代码的编写后需要设置功能包的编译选项，让系统知道Python程序的入口，打开功能包的setup.py文件，加入如下入口点的配置：

```
entry_points={
    'console_scripts': [
     'param_declare          = learning_parameter.param_declare:main',
    ],
},
```

#### **案例三：机器视觉应用**

参数大家已经会使用了，如何在机器人中应用呢？

继续优化机器视觉的示例，物体识别对光线比较敏感，不同的环境大家使用的阈值也是不同的，每次在代码中修改阈值还挺麻烦，不如我们就把阈值提炼成参数，运行过程中就可以动态设置，不是大大提高了程序的易用性么？

说干就干，我们先来看下效果如何，再看下代码中的变化。

##### 运行效果

启动三个终端，分别运行：

1. 相机驱动节点
2. 视觉识别节点
3. 修改红色阈值

```
$ ros2 run usb_cam usb_cam_node_exe 
$ ros2 run learning_parameter param_object_detect
$ ros2 param set param_object_detect red_h_upper 180
```

在启动的视觉识别节点中，我们故意将视觉识别中红色阈值的上限设置为0，如果不修改参数，将无法实现目标识别。

![image-20220530160304929](https://book.guyuehome.com/ROS2/2.%E6%A0%B8%E5%BF%83%E6%A6%82%E5%BF%B5/image/2.8_%E5%8F%82%E6%95%B0/image-20220530160304929.png)

为了便于调整阈值，我们在节点中将红色阈值的限位修改为了ROS参数，通过命令行修改该参数的值，就可以实现视觉识别啦。

![image-20220528014323915](https://book.guyuehome.com/ROS2/2.%E6%A0%B8%E5%BF%83%E6%A6%82%E5%BF%B5/image/2.8_%E5%8F%82%E6%95%B0/image-20220528014323915.png)

![image-20220530160505538](https://book.guyuehome.com/ROS2/2.%E6%A0%B8%E5%BF%83%E6%A6%82%E5%BF%B5/image/2.8_%E5%8F%82%E6%95%B0/image-20220530160505538.png)

##### 代码解析

我们来看下在视觉识别的代码中，是如何通过参数来设置阈值的。

learning_parameter/param_object_detect.py

```
#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
@作者: 古月居(www.guyuehome.com)
@说明: ROS2参数示例-设置目标识别的颜色阈值参数
"""

import rclpy                      # ROS2 Python接口库
from rclpy.node import Node       # ROS2 节点类
from sensor_msgs.msg import Image # 图像消息类型
from cv_bridge import CvBridge    # ROS与OpenCV图像转换类
import cv2                        # Opencv图像处理库
import numpy as np                # Python数值计算库

lower_red = np.array([0, 90, 128])     # 红色的HSV阈值下限
upper_red = np.array([180, 255, 255])  # 红色的HSV阈值上限

"""
创建一个订阅者节点
"""
class ImageSubscriber(Node):
  def __init__(self, name):
    super().__init__(name)                                  # ROS2节点父类初始化  
    self.sub = self.create_subscription(Image,              # 创建订阅者对象（消息类型、话题名、订阅者回调函数、队列长度）   
                  'image_raw', self.listener_callback, 10) 
    self.cv_bridge = CvBridge()                             # 创建一个图像转换对象，用于OpenCV图像与ROS的图像消息的互相转换

    self.declare_parameter('red_h_upper', 0)                # 创建一个参数，表示阈值上限
    self.declare_parameter('red_h_lower', 0)                # 创建一个参数，表示阈值下限

  def object_detect(self, image):
    upper_red[0] = self.get_parameter('red_h_upper').get_parameter_value().integer_value    # 读取阈值上限的参数值
    lower_red[0] = self.get_parameter('red_h_lower').get_parameter_value().integer_value    # 读取阈值下限的参数值
    self.get_logger().info('Get Red H Upper: %d, Lower: %d' % (upper_red[0], lower_red[0])) # 通过日志打印读取到的参数值

    hsv_img = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)                                        # 图像从BGR颜色模型转换为HSV模型
    mask_red = cv2.inRange(hsv_img, lower_red, upper_red)                                   # 图像二值化
    contours, hierarchy = cv2.findContours(mask_red, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)  # 图像中轮廓检测
    for cnt in contours:                                                                    # 去除一些轮廓面积太小的噪声
        if cnt.shape[0] < 150:
            continue

        (x, y, w, h) = cv2.boundingRect(cnt)                            # 得到苹果所在轮廓的左上角xy像素坐标及轮廓范围的宽和高
        cv2.drawContours(image, [cnt], -1, (0, 255, 0), 2)              # 将苹果的轮廓勾勒出来
        cv2.circle(image, (int(x+w/2), int(y+h/2)), 5, (0, 255, 0), -1) # 将苹果的图像中心点画出来

    cv2.imshow("object", image)                                         # 使用OpenCV显示处理后的图像效果
    cv2.waitKey(50)

  def listener_callback(self, data):
    self.get_logger().info('Receiving video frame')     # 输出日志信息，提示已进入回调函数
    image = self.cv_bridge.imgmsg_to_cv2(data, "bgr8")  # 将ROS的图像消息转化成OpenCV图像
    self.object_detect(image)                            # 苹果检测

def main(args=None):                                    # ROS2节点主入口main函数
    rclpy.init(args=args)                               # ROS2 Python接口初始化
    node = ImageSubscriber("param_object_detect")       # 创建ROS2节点对象并进行初始化
    rclpy.spin(node)                                    # 循环等待ROS2退出
    node.destroy_node()                                 # 销毁节点对象
    rclpy.shutdown()                                    # 关闭ROS2 Python接口
```

完成代码的编写后需要设置功能包的编译选项，让系统知道Python程序的入口，打开功能包的setup.py文件，加入如下入口点的配置：

```
entry_points={
    'console_scripts': [
     'param_declare          = learning_parameter.param_declare:main',
     'param_object_detect    = learning_parameter.param_object_detect:main',
    ],
},
```
