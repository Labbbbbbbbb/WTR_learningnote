下载指引：[VS Code搭建Typst写作环境教程(Windows) - 知乎 (zhihu.com)](https://zhuanlan.zhihu.com/p/644816041)

中文网站：[中文用户指南 – Typst 中文文档 (typst-doc-cn.github.io)](https://typst-doc-cn.github.io/docs/chinese/)

官网：[Typst Documentation](https://typst.app/docs)   指路 docs/tutorial/1-writing.md

中文版教程：[使用 Typst 写作 – Typst 中文文档 (typst-doc-cn.github.io)](https://typst-doc-cn.github.io/docs/tutorial/writing-in-typst/)

# Writting

## 标题

使用 `=` 号，（中间有空格）

`= Title`

与markdown一样，标题级别由=的个数决定,  ==即可生成副标题，可以多级嵌套使用

## 强调

使用下划线 `_`

```
_emphasize_
```

## 列表

有序列表：使用 `+` 号

```
+ the first
+ the second
+ the third
```

无序列表：使用 `-` 号

```
- point1
- point2
- point3
```

列表之间也可以嵌套

## 图表(figure)

使用 `image` 函数：`#image("path_of_the_image")`

如果要修改图像的大小：`image("path",width:n%) ` 指定大小的参数是一个命名参数 `name:value` ,其中的value可以是相对值如 `80%` ,也可以是绝对值如 `1cm`

加入说明(caption): `figure `函数（使用该函数把caption绑到image上）

caption的value值是用[中括号]括起来的内容块

```
#figure(
    image("path",width:"70%"),
    caption:[
      txt(加入想要的说明，然后这段文字会出现在图片下方)
    ],
)
```

> 注意Typst 区分两种模式：「标记模式」和「代码模式」 。  **默认是「标记模式」。 此模式下，你可以直接编排文本、使用不同的语法结构，如 *使用星号标记粗体文本* 。 而「代码模式」下，则更类似像Python一样的编程语言，提供了输入、执行代码的选项** 。
>
> Typst 里面，标记和代码相互交融在一起。 除了最常用的文档元素，其他所有均是由 *函数* 生成。 为了尽可能的便利，Typst 设计了精巧的语法，用来将代码嵌入在标记中：用 `#`(井号) 来**引入一个代码表达式([哈希标签语法](https://typst-doc-cn.github.io/docs/reference/scripting/#expressions))**， 表达式结束后，再恢复到正常的标记语法解析。 有些字符能够使其后字符继续解析为表达式，如果想将其解释为文本，可以用分号(`;`)来强制结束表达式解析。
>
> 而在figure函数的参数列表中，image已经是作为代码模式使用，故前面去掉#号


为图片贴上标签：在figure函数末尾加上 `<name>` (是的很像对结构体的typedef)

```
#figure(
  image("glacier.jpg", width: 70%),
  caption: [
    _Glaciers_ form an important part
    of the earth's climate system.
  ],
) <glaciers>
```

然后在其他文本处就可以通过 `@name` 直接引用图像


## 添加参考文献

使用 `#bibliography` 函数

```
bibliography(
str array,
title: noneautocontent,
full: bool,
style: str,
) -> content
```

其中第一个str为必填参数，是对应的 `.bib` 文件的路径


## 数学

 [参考的数学部分](https://typst-doc-cn.github.io/docs/reference/math/) 提供了数学模式提供的所有函数的完整列表，下为一些简要提取

数学公式使用两个 `$` 符号括起来，如果想要另起一行则$与公式之间应有空格

```
$Q = rho A v + C$    //行内公式
$ Q = rho A v + C $    //另起一行公式
```

上下标：上标使用 `^` ,下标使用 `_` 

若变量名是由多个字母组成的，可以用双引号括起来


# Format
