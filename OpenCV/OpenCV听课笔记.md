opencv听课笔记，只是一个听课笔记，详情见[SSC202学长的github](https://github.com/SSC202/OpenCV/tree/main/OpenCV-Python/Note)

## 安装

#### Conda环境

Windows可以去老登西的github找anaconda下载方法，ubuntu直接下miniconda：

[官网点我](https://docs.anaconda.com/free/miniconda/)

命令行下载：（ps这里的第二三行是同一条命令来的只是显示问题，wget那一条非常长)

```
mkdir -p ~/miniconda3
wget https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh -O ~/miniconda3/miniconda.sh
bash ~/miniconda3/miniconda.sh -b -u -p ~/miniconda3
rm -rf ~/miniconda3/miniconda.sh
```

> Let me explain what each command does:
>
> 1. `mkdir -p ~/miniconda3`: This command creates a directory named `miniconda3` in your home (`~`) directory. The `-p` option ensures that the command creates parent directories if they do not exist.
> 2. `wget https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh -O ~/miniconda3/miniconda.sh`: This command downloads the latest Miniconda installer script for Linux and saves it as `miniconda.sh` in the previously created `miniconda3` directory.
> 3. `bash ~/miniconda3/miniconda.sh -b -u -p ~/miniconda3`: This command executes the Miniconda installer script (`miniconda.sh`). The options used are:
>
>    - `-b`: Enables batch mode, which skips the user prompts and assumes default answers.
>    - `-u`: Updates the existing installation if a previous version of Miniconda is detected.
>    - `-p ~/miniconda3`: Specifies the target installation directory.
> 4. `rm -rf ~/miniconda3/miniconda.sh`: This command removes the downloaded Miniconda installer script (`miniconda.sh`) after the installation is complete. The `-rf` options force the removal without prompting and recursively remove directories.
>
> After running these commands, you should have Miniconda installed in the `~/miniconda3` directory on your Linux system. Make sure to activate the Miniconda environment by running `source ~/miniconda3/bin/activate` or by adding the Miniconda `bin` directory to your system's `PATH` variable.

下载完成之后，初始化bash和zsh

```
~/miniconda3/bin/conda init bash
~/miniconda3/bin/conda init zsh
```

#### Opencv

```
pip install opencv-contrib-python
```

(装不了可以翻墙了再装）

## API

opencv官网  https://opencv.org/

#### img-图像

读入图像: args=路径，读取方式

```
cv2.imread('path',mode)

img=cv2.imread('path',mode)   #创建图像对象
```

返回的img是一个图像，是一个三维的 NumPy 数组，通常具有形状 (height, width, channels)，后面的frame同理

显示图像：args=图像名，imread返回值

```
cv2.imshow('name',img)
```

加载窗口（即先创建一个空窗口，可以指定大小，后面的imshow再将图像加载到此窗口上)：

args=名字，图片大小

```
cv2.namedWindow('img', cv2.WINDOW_AUTOSIZE)
```

保存图像：args=图片保存的文件名，保存对象     （一般不需要用到)

```
cv2.imwrite('name',img)
```

颜色空间转换函数，args=图片，转换的格式(如RGB，灰度图像等等)

```
cv2.cvtColor('picture',mode)
```

> cv2.COLOR_BGR2RGB 将BGR格式转换成RGB格式
>
> cv2.COLOR_BGR2GRAY 将BGR格式转换成灰度图片

#### vedio-视频

创建对象  参数为摄像头设备的索引号，电脑内置摄像头默认为第一个，即为0，其他依次顺延

```
cap = cv2.VideoCapture(1)
```

读取视频  视频是一帧一帧地传的，所以需要放在一个死循环中读取，每一次read返回一个元组，第一个数据为布尔值，判断是否读到视频内容，第二个为该帧的内容

```
ret,frame = cap.read()
```

读取/设置函数信息

```
cap.get(ID)
cap.set(ID,val)
```

释放对象

```
cap.release()
```

## 绘图

本人电脑屏幕上的坐标大约是(0,0)--->(470,260)

画线: args=图像，左上角点，右上角点,BGR元组，线条粗细

```
cv2.line()
```

画圆：args=图像，圆心，半径，BGR元组，线条粗细（为-1时填充整个圆形)

```
cv2.circle()
```

画矩形：args=图像，左上角点，右上角点，BGR元组，线条粗细（为-1时填充整个矩形)

```
cv2.rectangle()
```

写文字：args=图片，显示的文字，检测框左上角坐标，字体，字体大小，颜色，字体粗细

```
cv2.putText()
```

此外还有一些参数（类似宏定义的)比如 `cv2.LINE_AA`可以放在封闭曲线和cv2.line函数的最后一个参数（在前面已有参数之后再加上)表示让线条更平滑，也可以在cv2.putText()函数中用来表示字体粗细；`cv2.FONT_HERSHEY_SIMPLEX` 作为cv2.putText()的‘字体’的参数

## 鼠标事件

可以通过以下代码查看鼠标事件的种类

```
import cv2
events = [i for i in dir(cv2) if 'EVENT' in i]
print(events)
```

返回：

```
['EVENT_FLAG_ALTKEY', 'EVENT_FLAG_CTRLKEY', 'EVENT_FLAG_LBUTTON', 'EVENT_FLAG_MBUTTON', 'EVENT_FLAG_RBUTTON', 'EVENT_FLAG_SHIFTKEY', 'EVENT_LBUTTONDBLCLK', 'EVENT_LBUTTONDOWN', 'EVENT_LBUTTONUP', 'EVENT_MBUTTONDBLCLK', 'EVENT_MBUTTONDOWN', 'EVENT_MBUTTONUP', 'EVENT_MOUSEHWHEEL', 'EVENT_MOUSEMOVE', 'EVENT_MOUSEWHEEL', 'EVENT_RBUTTONDBLCLK', 'EVENT_RBUTTONDOWN', 'EVENT_RBUTTONUP']
```

通过在事件发生后调用对应的回调函数，可以实现相应的操作

创建窗口并建立中断函数与之绑定：（两句的顺序不能颠倒，一定要先有窗口再绑上回调函数)

```
cv2.namedWindow('img')                      #创建窗口
cv2.setMouseCallback('img',function_name)   #创建回调函数
```

## 滑动条

滑动条创建函数：args=滑动条名字，滑动条被放置的窗口的名字，滑动条最小值，滑动条最大值，回调函数

(也要先创建窗口，再将滑动条以及回调函数与之绑定)

```
cv2.createTrackbar()
```

滑动条键值获取函数：args=滑动条名字，滑动条被放置窗口的名字

```
cv2.getTrackbarPos()
```

## 图像基本操作

### 图像信息的获取(重)

获取某个像素点的BGR数组：

```
px = img[x,y]
```

此处img为imread的返回值或cap.read的第二个返回值，即图像对象，img[x,y]表示对图像img进行相应坐标的索引（所以这并不是一个函数)

获取图像属性：`img.shape` (依次返回行数，列数，通道数(rgb图像为三通道，灰度图像为单通道))和 `img.dtype` 返回图像数据类型

截取“感兴趣的部分"：ROI.  即在整个图像上面通过索引的方式截取特定部分

```
roi = frame[100：200，150：250]		#表示100行到200行，150列到250列
cv2.imshow('img',roi)
```

（不过摄像头能看到的地方一般都是我们的ROI)

### 通道分离与合并

合并三个通道：

```
img2 = cv2.merge([b, g, r])
```

输入参数为bgr列表(注意此处的bgr是split的返回值)

对通道进行拆分：

```
b, g, r = cv2.split(img)
```

返回一整版的b,g,r二维数组（是的三组二维数组)

### 图像的加运算

加运算：

```
cv2.add()
```

混合运算：

```
cv2.addWeighted()
```

> merge和add的区别：merge是对一张图像的通道融合，add是对几张图像的像素进行叠加

## 图像处理

### 格式转换

颜色空间转化函数(args = 待转换图像，转换格式（比如BGR，GRAY，HSV))

```
cv2.cvtcolor()
```

第二个参数长这样：`cv2.COLOR_xxx2xxx` 表示格式从前一个xxx转成后一个xxx

确定阈值，建立掩膜 (args = 图像，下限，上限)

```
cv2.inRange()
```

高于上限和低于下限的都会变成0（黑色)，在上下限之间的变成255（白色)，得到一个黑白的图

除此之外还有二值化函数，即像素值高于阈值则设为白/黑色，低于阈值则设为黑/白色                (args = 原图像，阈值，二值化最大值，二值化方法)，返回一个元组，第二个元素为处理后的图像

```
cv2.threshold()
```

二值化的原图像应为灰度图

参数‘二值化方法’有 `cv2.THRESH_BINARY`，`cv2.THRESH_BINARY_INV` （简单阈值)

自适应阈值化：（过于灵敏一般不用于识别)

大津法阈值化：

### 滤波算法

图像之中经常有一些噪点噪声，需要通过滤波算法过滤噪点

比如自定义滤波 `CV2.filter2D()` 和高斯滤波，中值滤波等等

除此之外还有一系列形态学转换方法

**处理图像的一般流程：转为灰度图(cvtColor)---->滤波(filter)---->二值化(threshold)---->形态学转换**


### 轮廓检测

提取轮廓(args = 二值化图像，轮廓检索模式（如 `cv2.RETR_TREE`)，轮廓近似方法（一般用 `cv2.CHAIN_APPROX_SIMPLE`))

```
contours, hierarchy = cv2.findContours(image,mode,method)
```

绘制轮廓(args = 原图像，轮廓，轮廓的索引，颜色（BGR)，线条宽度）

```
cv2.drawContours(image, contours, contourIdx, color, thickness=None, lineType=None, hierarchy=None, maxLevel=None, offset=None)
```


图像的矩

```
"""
	图像矩求取函数
	参数：边界
	返回值：图像矩，使用['mij']索引进行特定图像矩的获取
"""
cv2.moments()
```

hu矩：

```
"""
	轮廓相似度比较函数
	第一个参数：原图像
	第二个参数：进行匹配的图像
	第三个参数：比较方式(int,填1即可)
	第四个参数：double，填0.0即可
	返回值：相似度，0为完全相似
"""
cv2.matchShapes()
```
