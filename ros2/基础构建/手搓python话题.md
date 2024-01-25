# 关于话题的一些规则

* Topics模型是一种发布订阅模型，该模型下节点分为发布者节点(publisher)和订阅者节点(subscriber)，一个话题发布者和订阅者的数量都是不限的。
* 话题的名字是关键，发布订阅的接口类型要相同，收发的数据类型也要相同
* 同一个节点可以同时订阅多个话题，同时发布多个话题

# 一个强大的工具：RqtGraph

启动话题中的订阅和发布者节点之后，通过 `rqt_graph`在新的终端中打开程序，即可看见数据的流向，搞清楚每一个节点的输入输出。

# ros2话题相关的CLI工具

```
ros2 topic list                  #返回系统中当前活动的所有话题的列表
ros2 topic list -t		 #增加显示消息类型

ros2 topic echo /[name of topic] #打印实时话题内容

ros2 topic info /[name of topic] #查看话题信息，返回消息类型，发布者和订阅者数量

ros2 interface show [type of message]  
#从上一条指令可以得到消息类型，比如std_msgs/msg/string，借助本条指令可以详询该消息类型的接口方式等

ros2 topic pub /[name of topic] [type of msg] '[name]:"[message]"'
#在无发布者运行时可以手动发布消息；其中type of message是上述info查出的类型，name是interface show查出的接口变
#量的名字，message是所要发布的内容
#也可以用yaml格式：
ros2 topic pub /[name of topic] [type of msg] "{[name]: [message]}"  
#注意[message]前有空格
```

# 编写话题发布者

编写话题发布者的通用流程：

* 导入消息类型
* 声明并创建发布者
* 编写发布逻辑来发布数据

继续在李家村这个包中编写，李四写了艳娘传奇，通过话题发布，并被王二订阅

```
import rclpy
from rclpy.node import Node
from std_msgs.msg import String             #导入消息类型

class WriterNode(Node):
    def __init__(self,name):
        super().__init__(name)
        self.get_logger().info("大家好，我是作家%s." % name)
	#声明并创建发布者；参数：消息类型，话题名称，大小。可以在vc中打开，右键查询函数的用法或查看官方api
	self.pub_novel=self.create_publisher(String,"sexy_girl",10)  

	#self.pub_novel.publish()	本条已经可以发布数据了，但是只发送一次不满足需求--->改为周期发送

	self.count=0
	self.timer_period=5		#定义发送周期 5秒钟
	self.timer=self.create_timer(self.timer_period,self.timer_callback)

    def timer_callback(self):		#创建回调函数，并传入create_timer函数中，意为每5秒调用一次回调函数
	msg=String()
	msg.data="第%d回，潋滟湖第%d次偶遇胡艳娘"%(self.count,self.count)
	self.pub_novel.publish(msg)	#让发布者发布消息
	self.get_logger().info("发布了一章小说，内容是：%s"%msg.data)
	self.count+=1

def main(args=None):
    '''发布者节点本身也是一个节点，所以也遵循节点编写的框架'''
    rclpy.init(args=args)
    lisi_node=WriterNode("lisi")
    rclpy.spin(lisi_node)
    rclpy.shutdown
  

```

> 寄 不知道为什么出现'Publisher' object is not callable的报错，节点无法正常运行。。地球什么时候爆炸。很急。----------->后续：破案了 ，callback函数中的self.pub_novel.后面的publish刚开始没加（小鱼的视频没有加），导致Publisher没有正确实例化。

# 编写话题订阅者

创建订阅者的一般流程：

* 导入订阅的话题接口类型
* 创建订阅回调函数
* 声明并创建订阅者
* 编写订阅回调处理逻辑

一个节点可以看成机器人系统中的"社会关系"的总和，每一个节点可以发布/订阅不定数量的话题，注意理清其中的逻辑联系与信息流向

## 李四去订阅其他话题

艳娘传奇发布后，作家为了过生活，订阅了一个收钱话题sexy_girl_money，用于收取稿费（仍然是在李四这个节点，只是在新的话题下他的身份变成了订阅者）

在原有代码的基础添加了创建订阅者的函数：

```
self.create_subscription(UInt32,"sexy_girl_money",self.recv_money_callback,10)
```

参数含义：消息类型为UInt32，话题名称为sexy_girl_money，接收中断调用recv_money_callback进行存钱

```
import rclpy
from rclpy.node import Node
from std_msgs.msg import String,UInt32             #导入消息类型,此处还添加了新的消息类型UInt32

class WriterNode(Node):
    def __init__(self,name):
        super().__init__(name)
        self.get_logger().info("大家好，我是作家%s." % name)
	#声明并创建发布者
	self.pub_novel=self.create_publisher(String,"sexy_girl",10)  

	#创建定时器成员属性timer
	self.count=0
	self.timer_period=5
	self.timer=self.create_timer(self.timer_period,self.timer_callback)

	#定义属性account表示账户余额
	self.account=80
	#创建并初始化订阅者成员属性sub_money
	self.sub_momey=self.create_subscription(UInt32,"sexy_girl_money",self.recv_money_callback,10)

    def timer_callback(self):		#创建回调函数，并传入create_timer函数中，意为每5秒调用一次回调函数
	msg=String()
	msg.data="第%d回，潋滟湖第%d次偶遇胡艳娘"%(self.count,self.count)
	self.pub_novel.publish(msg)	#让发布者发布消息
	self.get_logger().info("发布了一章小说，内容是：%s"%msg.data)
	self.count+=1
  
    def recv_money_callback(self,money):
	#编写订阅回调处理逻辑
	self.account+=money.data
	self.get_logger().info('李四：我已经收到了%d的稿费'%self.account)

def main(args=None):
    '''发布者节点本身也是一个节点，所以也遵循节点编写的框架'''
    rclpy.init(args=args)
    lisi_node=WriterNode("lisi")
    rclpy.spin(lisi_node)
    rclpy.shutdown
  

```

至此李四就已经完成了订阅。由于money话题暂时没有发布者，我们可以通过命令 手动向李四发布消息，来观察该节点订阅消息的状态

```
ros2 topic pub /sexy_girl_money std_msgs/msg/UInt32 "{data: 10}"     (默认一秒钟一次)
ros2 topic pub /sexy_girl_money std_msgs/msg/UInt32 "{data: 10}" -1  (只发一次)
```

## 其他人来订阅李四发布的小说话题

一个王五来订阅了李四的小说：

```
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class NodeSubscribe02(Node):
    def __init__(self,name):
        super().__init__(name)
        self.get_logger().info("大家好，我是%s!" % name)
        # 创建订阅者
        self.command_subscribe_ = self.create_subscription(String,"sexy_girl",self.command_callback,10)

    def command_callback(self,msg):
      
        self.get_logger().info(f'看到了[{msg.data}]章节')

def main(args=None):
    rclpy.init(args=args) # 初始化rclpy
    wangwu_node = NodeSubscribe02("wangwu")  # 新建一个节点
    rclpy.spin(wangwu_node) # 保持节点运行，检测是否收到退出指令（Ctrl+C）
    rclpy.shutdown() # 关闭rclpy
```

***注意创建了新节点之后要在setup.py中添加指引：***

```
entry_points={
        'console_scripts': [
            "lisi_node=village_li.lisi:main",     ###千万注意这里有一个逗号！！！
            "wangwu_node=village_li.wangwu:main"
        ],
    },
```

然后build&souce

至此就可以同时run两个节点，观察话题的发布和订阅啦!
