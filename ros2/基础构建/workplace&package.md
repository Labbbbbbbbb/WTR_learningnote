# 工作空间与功能包

## 工作空间

> 工作空间是一个存放项目开发相关文件的文件夹，是开发过程的大本营

工作空间中一般含有src，install，build，log三个文件夹

* src，代码空间，存放编写的代码或脚本，绝大部分的工作都是在该文件下进行
* install，安装空间，存放编译成功后的可执行文件或脚本
* build，编译空间，编译过程中产生的一些我们并不需要了解的东西会被自动放入这个文件夹中
* log，日志空间，编译过程及后续产生的警告，错误等记录会被保存到这个文件夹中

### 创建工作空间

##### **第一步：创建文件夹并复制好源码**

**1.若有可视化桌面，可以在Documents中直接右键创建新的文件夹，开辟工作空间**

**2.使用命令行--->把下面这一串one by one 地输入终端就好啦 （）**

```
mkdir -p ~/dev_ws01/src
$ cd ~/dev_ws01/src
$ git clone https://gitee.com/guyuehome/ros2_21_tutorials.git  #也就是说此处就可以把功能包什么的放进来了
```

##### 第二步：自动安装依赖（在src下）

```
sudo apt install -y python3-pip
sudo pip3 install rosdepc        #借助鱼香肉丝的一键安装
sudo rosdepc init
rosdepc update
cd ..                            #回到工作空间的根目录(dev_ws01)
rosdepc install -i --from-path src --rosdistro humble -y
```

by the way 如果遇到了 ` Unmet dependencies. Try 'apt --fix-broken install' with no packages (or specify a solution).`，则按照它说的试一下 `sudo apt --fix-broken install `不行再百度就好啦

##### 第三步：编译工作空间

```
sudo apt install python3-colcon-ros
cd ~/dev_ws01/
colcon build
```

然后再dev_ws01下ls即可看见上述的四个文件夹啦

***colcon:***

> colcon是一个功能包构建工具，简单的说是用来编译代码的。安装ros时并没有自动安装colcon，所以需要手动安装：(在home目录下)
>
> 但是！如果走了古月的流程就已经下好colcon了！不用再运行这一句！
>
>  `sudo apt-get install python3-colcon-common-extensions `


##### 第四步：设置环境变量

（在工作空间的根目录，dev_ws01/)

```
source install/local_setup.sh # 仅在当前终端生效
echo " source ~/dev_ws01/install/local_setup.sh" >> ~/.bashrc # 所有终端均生效（记得改文件名）
```

### 功能包

功能包相当于封装完好的函数，把各种功能封装在功能包中，降低各部分代码块的耦合关系，可以提高代码的移植性

#### 安装/下载/创建功能包

##### 安装获取

如小乌龟功能包这些是ros2自带且已经帮我们安装好的（一般无需再次安装）。对于有作者已经编写好并打包可执行文件上传到系统中的功能包，我们可以直接安装获取，可以借助命令：

```
sudo apt install ros-<version>-package_name
```

其中version是指ros2的版本，此处为humble

即：sudo apt install ros-humble-turtlesim

安装所得会自动放入系统目录 opt/ros/humble，不用再次手动source

##### 下载源码并手动编译

若只是下载了其他作者的源码（而不是可执行文件），如古月居的github源码ros2_21_tutorials，又或者是对源码进行了一定的修改，都需要手动编译

以ros2_21_tutorials为例下载源码：(在src中)

```
 git clone https://gitee.com/guyuehome/ros2_21_tutorials.git
```

下载源码和自行建包都需要走下面的手动编译流程

##### 自行创建和编写并手动编译

使用指令：

```
ros2 pkg create --build-type <build-type> <package_name>
```

ros2命令中：

* **pkg** ：表示功能包相关的功能；
* **create** ：表示创建功能包；
* **build-type** ：表示新创建的功能包是C++还是Python的，如果使用C++或者C，那这里就跟ament_cmake，如果使用Python，就跟ament_python；
* **package_name** ：新建功能包的名字。

比如在终端中分别创建C++和Python版本的功能包：

```
cd ~/dev_ws01/src
ros2 pkg create --build-type ament_cmake learning_pkg_c               # C++
ros2 pkg create --build-type ament_python learning_pkg_python         # Python（记得改文件名！！！）
```

#### **编译功能包**

在创建好的功能包中，我们可以继续完成代码的编写，之后需要编译和配置环境变量，才能正常运行：

```
$ cd ~/dev_ws01
$ colcon build   # 编译工作空间所有功能包
$ source install/local_setup.bash
```

#### 其他cli命令

列出所有可执行文件

```
ros2 pkg executables
```

列出某个功能包可执行文件

```
ros2 pkg executables [packagename]
```

输出某个包所在路径的前缀

```
ros2 pkg prefix [packagename]
```

列出包的清单描述文件(每个功能包都有标配的.xml文件，用于记录包的名字，构建工具，编译信息等)

```
ros2 pkg xml turtlesim
```

其他colcon常用指令：

```
colcon test --packages-select [packagename]      #只编译一个包（否则会编译工作空间中所有的包）
colcon test --packages-select [packagename] --cmake-args -DBUILD_TESTING=0   #不编译测试单元
colcon build --symlink-install            #允许通过更改src下的部分文件来改变install（重！）
'''因为python是解释型语言，每次使用colcon build时只是将src的文件拷贝到了install中而不建立实时链接，所以如果只是用colcon build的话，每一次修改完src的代码之后就需要重新build（否则实际执行的文件不会改变）。而如果加上以上后缀，则可以建立与install中文件的链接，从而允许通过更改src下的部分文件来改变install


```

#### 功能包的结构

功能包不是普通的文件夹，其中有一些特定的结构，由于我们所用主要是python类，这里主要说明python功能包的结构

python功能包一定含有**package.xml**和**setup.py**两个文件

package.xml文件包含功能包的版权描述，和各种依赖的声明；setup.py文件里边也包含一些版权信息，除此之外，还有“entry_points”配置的程序入口

（若是c++则会有cmake文件来负责C语言的编译）
