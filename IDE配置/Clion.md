# Clion+cube配置过程

## 在clion中创建项目

（以oled项目创建过程为例

在clion项目文件夹目录下新建文件（记住此处的命名）

![1707149671327](image/oled/1707149671327.png)

通过clion打开stm32cubemx，把芯片型号换好，正常配置cube，注意Project Manager中工程名字一定要和第一张图里的命名保持一致，点击GENERATE CODE，要出现"do you want to over write it "的提示字样才是正确复写

![1707149884847](image/oled/1707149884847.png)

然后就可以看到这样的界面噜

![1707150034386](image/oled/1707150034386.png)

*接下来配置调试和烧录工具(openOCD)

## openOCD

点击”编辑配置“

![1707150257170](image/oled/1707150257170.png)

然后点击左上角处的+号，选择openOCD

![1707150384591](image/oled/1707150384591.png)

![1707150462477](image/oled/1707150462477.png)

配置好可执行的二进制文件，然后点击面板配置文件的”协助“，选择蓝色药丸（别的也可以，点击复制到项目并使用，然后就可以叉掉这个界面了

![1707150532664](image/oled/1707150532664.png)

回到项目主页面，在文件管理处找到刚刚复制过来的蓝药丸.cfg文件，然后用以下代码替换掉该文件的内容

```
source [find interface/stlink.cfg]
transport select hla_swd

source [find target/stm32f1x.cfg]
adapter speed 10000
```

如图

![1707150773041](image/oled/1707150773041.png)

保存修改即可.

## 调试

在elf左边有个Debug，点它，弹出的选项卡长这样就好（不用特别去改

![1707150959685](image/oled/1707150959685.png)

保存修改后就可以开始锤(编译)和运行噜，插入st-link烧录(点三角形)后可以看到如下成功界面：(红字但是看字不看色)

![1707151152471](image/oled/1707151152471.png)

至此IDE的配置搞定，可以快乐地写代码噜（喜

# 添加文件

在clion中添加文件时最好另外新建一个跟core同级的文件夹User，在User下新建Inc和Src两个文件夹，将自己的.h/.c文件放在其中，并且在cmakelist.txt中引用：

在52行最后添加 `User/Inc`，如下所示

```
include_directories(Core/Inc Drivers/STM32F1xx_HAL_Driver/Inc Drivers/STM32F1xx_HAL_Driver/Inc/Legacy Drivers/CMSIS/Device/ST/STM32F1xx/Include Drivers/CMSIS/Include User/Inc)
```

在56行最后添加 `"User/Src/*.*"` ，如下所示

```
file(GLOB_RECURSE SOURCES "Core/*.*" "Drivers/*.*" "User/Src/*.*")
```

修改完cmakelist之后要重新加载才能真正联动文件（否则虽然不会报错但是洞察就不能用了） 点击这个自动弹出的小框框就好噜

![1707328094011](image/Clion/1707328094011.png)

如果新文件弹出了任何”添加到目标“的选项（亦如上图）  **不要点 千万不要点**  否则会找不到应有的路径。
