# 关于服务的一些规则

* 服务是应答式的，也就是说消息是双向流动的（Request&Response）
* 同一个服务（名称相同）有且只能有一个节点来提供，也就是只能有一个服务端
* 同一个服务可以有多个客户端

# 关于服务的一些CLI工具

输入以下命令运行系统提供的服务例程（两数相加）

```
ros2 run examples_rclpy_minimal_service service  				#运行服务节点
ros2 service call /add_two_ints example_interfaces/srv/AddTwoInts "{a: 1,b: 5}" #手动调用服务（在另一个终端
```

其中add_two_ints是服务名称，example_interfaces/srv/AddTwoInts是消息类型

查看已经启动的服务列表

```
ros2 service list
```

查看服务的接口类型

```
ros2 service type /add_two_ints
```

add_two_ints是服务名称，此命令返回`example_interfaces/srv/AddTwoInts`

查找使用某一接口的服务

```
ros2 service find example_interfaces/srv/AddTwoInts
```

此功能与上一条命令的功能恰好相对，一个是已知服务找类型，一个是已知类型找服务

# 代码实现

## 从编写自定义服务接口开始

话题通信是单向的，因此话题接口只需要定义传过去的数据类型即可，而服务是双向的，所以需要定义来回两种数据类型。

服务接口格式：`xxx.srv`  以add_two_ints为例

```
int64 a
int64 b
---
int64 sum
```

中间的三个横杠---是分界线，上方是客户端发送请求的数据结构定义，下方是服务端响应结果的数据结构定义

创建服务接口的一般步骤：

* 在原工作空间的src目录下创建接口功能包，在功能包根目录下新建 `srv` 文件，并在文件夹下新建 `xxx.srv`
* 在 `xxx.srv` 下编写服务接口内容并保存
* 在 `CmakeLists.txt`添加依赖和srv文件目录
* 在 `package.xml` 中添加 `xxx.srv`所需的依赖
* 编译功能包即可生成python与c++头文件

### 创建并编写.srv文件

cd到src之后

```
ros2 pkg create village_interfaces --dependencies rosidl_default_generators
```

创建新的功能包，buildtype不写时默认为ament_cmake类型，后面的 `rosidl_default_generators`依赖是必加的，某些情况下可能会有其他的依赖需要添加

在village_interfaces目录下新建srv文件夹，建立borrow_money.srv文件（用于借钱服务）

```
#request
string name
uint32 money
---
#response
bool success
uint32 money
```

接口应定义哪些数据类型应事先根据实际应用规划好

### 修改CmakeLists.txt

在CmakeLists.txt中可以看到这样一段

```
# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)       #这一句就是创建功能包时添加的依赖
```

在下一行加上：

```
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/borrow_money.srv"
)
```

功能包中如果有其他类型的接口文件以及依赖也是放在这里，举例：

```
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/RobotPose.msg"
  "msg/RobotStatus.msg"
  "srv/MoveRobot.srv"
  DEPENDENCIES geometry_msgs     #依赖；因为borrow_money.srv中定义的都是基本的数据类型，不需要添加其他的依赖
)
```

### 修改 package.xml

```
<buildtool_depend>ament_cmake</buildtool_depend>

  <depend>rosidl_default_generators</depend>

  <member_of_group>rosidl_interface_packages</member_of_group>  #添加这一行

  <test_depend>ament_lint_auto</test_depend>
  <test_depend>ament_lint_common</test_depend>
```



然后就可以编译噜 

```
colcon build --packages-select village_interfaces
```

编译完成之后再source一下，然后就可以通过 `ros2 interface list`找到自己创建的接口包啦


## Python服务端代码

基本步骤

* 导入服务接口
* 创建服务端回调函数
* 声明并创建服务端
* 编写回调函数逻辑处理请求

李三要找李四借钱，则李三节点发布请求，是客户端，李四节点接受请求并提供借钱服务，是服务端。先来看服务端节点（lisi_node)

首先要导入服务接口时因为引用了另一个功能包的接口，需要添加新的依赖：在village_li功能包的 `package.xml`中有一句 `<depend>rclpy</depend> `在它底下加入一句 `<depend>village_interfaces</depend>`

代码：

```
import rclpy
from rclpy.node import Node
from std_msgs.msg import String,UInt32             #导入消息类型,此处还添加了新的消息类型UInt32

from village_interfaces.srv import brow_money      #导入服务接口

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

        #声明并创建服务端属性
        self.borrow_sever=self.create_service(brow_money,"borrow_money",self.borrow_money_callback)      

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

    def borrow_money_callback(self,request,response):            #编写回调函数逻辑处理请求  
        '''
        request:来自客户端的请求数据
        reponse:来自服务端的响应数据
        '''
        self.get_logger().info("收到来自%s的借钱请求，账户内现有%d原"%(request.name,self.account))
        if request.money<=self.account*0.1:
            response.success=True
            response.money=request.money
            self.account=self.account-request.money
            self.get_logger().info("借钱成功")
        else:
            response.success=False
            response.money=0
            self.get_logger().info("借钱失败")
        return response


def main(args=None):
    '''发布者节点本身也是一个节点，所以也遵循节点编写的框架'''
    rclpy.init(args=args)
    lisi_node=WriterNode("lisi")
    rclpy.spin(lisi_node)
    rclpy.shutdown
  

```


## Python客户端代码
