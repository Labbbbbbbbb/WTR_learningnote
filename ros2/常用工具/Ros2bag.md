# bag是什么

ros2 bag是ros2中一个常用的CLI工具，用于记录话题的数据；使用该指令将话题数据存储为文件，后续无需启动相应节点，就可以直接将bag里面的文件发布出来。

# rosbag2常用命令行

记录某个话题：(注意以下的topic_name前面都是带 / 的)

```
ros2 bag record <topic_name>
```

同时记录多个话题/所有话题

```
ros2 bag record <topic_name1> <topic_name2>
ros2 bag record -a
```

ctrl+c即可停止录制

自行修改记录文件的名字

```
ros2 bag record -o <file_name> <topic_name>
```

(file_name即为自定义的名字)

录制完成之后会在工作空间根目录底下生成相应文件夹，内含一个.yaml文件和一个.db3文件，其中文件主要存在于.db3中，可以使用命令进行查看

查看话题录制的相关信息，如时间，大小，类型等等

```
ros2 bag info xxx.db3
```

播放已经录制好的话题信息：

```
ros2 bag play xxx.db3
```

然后需要打开另一个终端，运行 `ros2 topic echo <topic_name>`才能看见播放出的内容

此外，bag play命令有其他参数，支持不同播放方式：（在命令末尾添加）

`-r num`  倍速播放，num表示倍数

`-l`   循环播放

`--topics <topic_name>`   有时.db3文件记录了不止一个话题，使用该参数只播放单个或者某几个话题的录制文件
