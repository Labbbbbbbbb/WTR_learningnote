*ps.这里是按fishros的流程，与古月的不要混着来*

# 创建工作空间与功能包

因为节点需要存在于功能包中，功能包需要存在于工作空间中，所以要想创建节点，就应该先创建工作空间，再创建功能包

## 创建工作空间

工作空间就是文件夹，所以很简单：  （在home目录下）

```
mkdir -p dev_ws/src
cd dev_ws/
code ./             #在工作空间中打开vscode，后续操作在vscode中进行
```

ps.(!)如果是wsl也可以自行开启windows的vc打开远程链接，选择相应的文件夹（工作空间）

在vscode中右键src文件夹，选择“在集成终端中打开”(open in integrated terminal)，即可在vc中打开终端。其实这和ubuntu的终端是一样的，只是打开位置不同。

## 创建功能包

在src目录下，，建立一个名为village_li的功能包

```
ros2 pkg create village_li --build-type ament_python --dependencies rclpy
```

其中--dependencies是可选项，指这个功能包的依赖，这里给的是ros2的python客户端接口rclpy

创建后可以通过tree看到这样的目录

![1705998570002](image/手搓python节点/1705998570002.png)

***tree不是ubuntu自带的，可以借助指令下载：***

```
sudo apt  install tree  # version 2.0.2-1
```



## 创建节点文件

在__init__.py同级别的目录下创建一个叫lisi.py的文件（用cmd或是直接在vc中右键创建均可）效果如图：

![1705999689646](image/手搓python节点/1705999689646.png)


# 开始编写程序

**编写ros2节点的一般步骤**：

1. 导入库文件
2. 初始化客户端库
3. 新建节点
4. spin循环节点
5. 关闭客户端库

**几种编程思想：OOP(面向对象，如c++)，POP(面向过程，如c语言)，FP(函数式思想)**

## 使用非oop方法编写第一个节点并测试

### 在lisi.py文件中编写代码：

框架：：：

```
import rclpy
from rclpy.node import Node    #一二两句即导入库文件（ros2&python接口库）

def main(args=None):
  
    rclpy.init(args=args)       #初始化客户端
    node=Node("lisi")           #新建一个节点(此处的起节点名即是后面用node list查询时会看到的)

    rclpy.spin(node)            #保持节点运行直到收到退出指令（ctrl+c）
    rclpy.shutdown()            #关闭rclpy
'''注意！node=Node("lisi")和spin(node)中的node都只是一个代称，具体要换成每个节点的名字，如lisi_node'''
```

```
#加入节点内容
import rclpy
from rclpy.node import Node  

def main(args=None):
  
    rclpy.init(args=args)   
    lisi_node=Node("lisi")   
    lisi_node.get_logger().info("大家好，我是李四")   #填充节点内容，实现节点功能
    rclpy.spin(lisi_node)  
    rclpy.shutdown()  
```



### 配置

编写完成后，要想成功编译运行代码，还应进行一些配置，告诉ros2功能包，节点和入口函数的位置：

在功能包下的setup.py文件中的末尾有

```
entry_points={
        'console_scripts': [
        ],
    },
```

加入路径配置：

```
entry_points={
        'console_scripts': [
	    "lisi_node=village_li.lisi:main"
        ],
    },
```


### 编译

如果已经下载了colcon，则在***工作空间根目录下(一定要注意！***)直接输入 `colcon build`

如果还没有：

```
sudo apt-get install python3-colcon-common-extensions 
colcon build
```

> ps.此处ros2官方可能会自带一些bug，如果出现 `SetuptoolsDeprecationWarning: setup.py install is deprecated. Use build and pip and other standards-based tools. `可能是因为setuptools版本过高，可以尝试回到home目录输入pythpn3，按如下步骤查看版本
>
> ![1706010524892](image/手搓python节点/1706010524892.png)
>
> 版本过高时使用命令 `pip uninstall setuptools-[自己的版本号]`将高版本setuptools卸载；
>
> 使用命令`pip install setuptools==58.2.0`安装指定较低版本setuptools
>
> 重复步骤一可查看是否重新安装，成功后***回到工作空间根目录colcon build***
>
> 当然my说不重下也可以，如果能顺利编译生成可执行文件让节点跑起来就忽略这一段



最后要记得***source***一下，让系统能找到包和文件 （！）

```
source install/setup.bash
```

### 运行

编译配置等工作都完成了就可以来运行这个节点噜 

```
ros2 run village_li lisi_node
```

节点跑起来后按ctrl+c可以退出，在退出以前都可以用list指令查看到该节点：

按ctrl+shift+5可以在vc中切出新终端，`ros2 node list`即可看到已经启动的节点

然后通过 `ros2 node info /lisi`可以查看节点的一些具体信息，如话题，服务，动作，参数等等（要注意加上前面的斜杠/）,效果如图

```
zyt_brain@zhanyt:~/dev_ws02$ ros2 node info /lisi
/lisi
  Subscribers:

  Publishers:
    /parameter_events: rcl_interfaces/msg/ParameterEvent
    /rosout: rcl_interfaces/msg/Log
  Service Servers:
    /lisi/describe_parameters: rcl_interfaces/srv/DescribeParameters
    /lisi/get_parameter_types: rcl_interfaces/srv/GetParameterTypes
    /lisi/get_parameters: rcl_interfaces/srv/GetParameters
    /lisi/list_parameters: rcl_interfaces/srv/ListParameters
    /lisi/set_parameters: rcl_interfaces/srv/SetParameters
    /lisi/set_parameters_atomically: rcl_interfaces/srv/SetParametersAtomically
  Service Clients:

  Action Servers:

  Action Clients:
```

## 使用OOP思想进行编程

### oop介绍

从“面向过程”到“面向对象”的思想转变，可以简单地理解为将原有的一个个步骤看作一个对象的属性与行为的组合，即先将“对象”所具有的参数先封装成一个整体，而过程的实现是通过对对象的调用来完成的。

一对重要概念：类与对象，类是对象的抽象化，对象是类中的个体，是类的具体化

三个重要特性：封装，继承与多态。其中封装指将属性和行为封装在一起，而对象=属性+行为。

面向对象的编程更加工程化，更具移植性与拓展性

### 使用oop思想重构节点代码

```
import rclpy
from rclpy.node import Node     #同样，导入接口库

class WriterNode(Node):         #设李四是一个作家，则可以创建一个作家节点的类
    def __init__(self,name):    #定义初始化函数
        super().__init__(name)  #super表示父类的初始化
        self.get_logger().info("大家好，我是一名作家")  #self代指自身，即这个类所创建出的节点

def main(args=None):
  
    rclpy.init(args=args)   
    lisi_node=WriterNode("lisi")   #新建类型不再是一般的节点，而是刚定义好的类
    rclpy.spin(lisi_node)  
    rclpy.shutdown()  
```

Done!	如果是初始编写，也不要忘记在setup.py里指路和source！
