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

**2.使用命令行--->把下面这一串one by one 地输入终端就好啦 （from 古月居）**

```
mkdir -p ~/dev_ws01/src
$ cd ~/dev_ws01/src
$ git clone https://gitee.com/guyuehome/ros2_21_tutorials.git
```

##### 第二步：自动安装依赖

```
sudo apt install -y python3-pip
sudo pip3 install rosdepc        #借助鱼香肉丝的一键安装
sudo rosdepc init
rosdepc update
cd ..
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

##### 第四步：设置环境变量

```
source install/local_setup.sh # 仅在当前终端生效
echo " source ~/dev_ws01/install/local_setup.sh" >> ~/.bashrc # 所有终端均生效
```

### 功能包

功能包相当于封装完好的函数，把各种功能封装在功能包中，降低各部分代码块的耦合关系，可以提高代码的移植性

#### 创建功能包

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

#### 功能包的结构

功能包不是普通的文件夹，其中有一些特定的结构，由于我们所用主要是python类，这里主要说明python功能包的结构

python功能包一定含有**package.xml**和**setup.py**两个文件

package.xml文件包含功能包的版权描述，和各种依赖的声明；setup.py文件里边也包含一些版权信息，除此之外，还有“entry_points”配置的程序入口

（若是c++则会有cmake文件来负责C语言的编译）
