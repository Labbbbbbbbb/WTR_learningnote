### **Git structure:**

*copy from zhihu*

![](https://pic1.zhimg.com/80/v2-db336093c3e8c0257d0e599b797b8e74_1440w.webp)

git 结构图示

`Workspace`: 工作区

`Index/Stage`: 暂存区

`Repository`: 仓库区（本地仓库）

`Remote`: 远程仓库

***注：以下所有命令从 `$`之后开始。***

### **一、新建代码库**

| 1. 在当前目录新建一个Git代码库         | $ git init                |
| -------------------------------------- | ------------------------- |
| 2. 新建一个目录，将其初始化为Git代码库 | $ git init [project-name] |
| 3. 下载一个项目和它的整个代码历史      | $ git clone [url]         |

ps.clone所用的url必须代表着一个仓库，普通网页是会报错的

### **二、配置**

`Git`配置信息在 `.git/config`里面

* `$ git --version`: 查看Git版本号
* `$ which git`: 显示当前Git路径
* `$ git config --list`: 查看配置信息。可以看到 `user.name`和 `user.email`信息。在当前项目下查看的配置是全局配置+当前项目的配置，使用的时候会优先使用当前项目的配置。

配置全局用户名和邮箱：

* `$ git config --global user.name "yourName"`
* `$ git config --global user.email "yourEmail"`

单独针对某一个项目配置 `Git`用户名和邮箱：

* `$ git config user.name "yourName"`
* `$ git config user.email "yourEmail"`

### **三、增加、删除文件**

| 1. 添加指定文件到暂存区                     | $ git add [file1] [file2]           |
| ------------------------------------------- | ----------------------------------- |
| 2. 添加指定目录到暂存区，包括子目录         | $ git add [dir]                     |
| 3. 添加当前目录的所有文件到暂存区           | $ git add .                         |
| 4. 删除工作区文件，并且将这次删除放入暂存区 | $ git rm [file1] [file2]            |
| 5. 停止追踪指定文件，但该文件会保留在工作区 | $ git rm --cached [file]            |
| 6. 改名文件，并将这个改名放入暂存区         | $ git mv [file-name] [file-renamed] |

### **四、代码提交**

| 1. 提交暂存区到仓库区                                                                       | $ git commit -m [message]                     |
| ------------------------------------------------------------------------------------------- | --------------------------------------------- |
| 2. 提交指定区的指定文件到仓库区                                                             | $ git commit [file1] [file2] ... -m [message] |
| 3. 提交工作区自上次提交之后的变化，直接到仓库区                                             | $ git commit -a                               |
| 4. 提交时显示所有的diff信息                                                                 | $ git commit -v                               |
| 5. 使用一次新的提交，替代上一次提交（如果代码没有任何变化，则用来改写上一次提交的提交信息） | $ git commit --amend -m [message]             |
| 6. 重做上一次提交，并包括指定文件的新变化                                                   | $ git commit --amend [file1] [file2]          |

### **五、分支**

| 1. 列出所有本地分支                     | $ git branch                                  |
| --------------------------------------- | --------------------------------------------- |
| 2. 列出所有远程分支                     | $ git branch -r                               |
| 3. 列出所有本地分支和远程分支           | $ git branch -a                               |
| 4. 新建一个分支，但依然停留在当前分支   | $ git branch [branch-name]                    |
| 5. 新建一个分支，并切换到该分支         | $ git checkout -b [branch-name]               |
| 6. 切换到指定分支，并更新工作区         | $ git checkout [branch-name]                  |
| 7. 将现有分支与指定远程分支建立追踪关系 | $ git branch --track [branch] [remote-branch] |
| 8. 合并指定分支到当前分支               | $ git merge [branch]                          |
| 9. 删除分支                             | $ git branch -d [branch-name]                 |
| 10. 删除远程分支                        | $ git push origin --delete [branch-name]      |

### **六、标签**

| 1. 列出所有tag             | $ git tag                              |
| -------------------------- | -------------------------------------- |
| 2. 新建一个tag在当前commit | $ git tag [tag]                        |
| 3. 新建一个tag在指定commit | $ git tag [tag] [commit]               |
| 4. 删除本地tag             | $ git tag -d [tag]                     |
| 5. 删除远程tag             | $ git push origin :refs/tags/[tagName] |
| 6. 查看tag信息             | $ git show [tag]                       |
| 7. 提交指定tag             | $ git push [remote] [tag]              |
| 8. 提交所有tag             | $ git push [remote] --tags             |

### **七、查看信息**

| 1. 显示变更的文件             | $ git status                         |
| ----------------------------- | ------------------------------------ |
| 2. 显示暂存区和工作区的差异   | $ git diff                           |
| 3. 显示你今天写了多少行代码   | $ git diff --shortstat "{0 day ago}" |
| 4. 显示当前分支的最近几次提交 | $ git reflog                         |

### **八、远程同步**

| 1. 将现有仓库与远程仓库建立联系         | $ git remote add origin [remote address] |
| --------------------------------------- | ---------------------------------------- |
| 2. 下载远程仓库的所有变动               | $ git fetch [remote]                     |
| 3. 显示所有远程仓库                     | $ git remote -v                          |
| 4. 取回远程仓库的变化，并与本地分支合并 | $ git pull [remote] [branch]             |
| 5. 上传指定分支到远程仓库               | $ git push [remote] [branch]             |
| 6. 推送所有分支到远程仓库               | $ git push [remote] --all                |

创建新的远程仓库的指令：

```
$git remote add origin [remote address]  #常用于将本地仓库与远程仓库进行链接，以便可以同步远程仓库的内容。
$git branch -M main  
#git branch -M main 命令会将本地分支 main 重命名为 main,相当于在本地分支 main 上执行了一个 git branch 命令，这个命令会将当前分支名称设置为 main,并将本地分支 main 与远程分支 main 保持一致。#
$git push -u origin main
#用于将本地分支 main 的更新推送到远程分支 origin/main。在这个命令中，-u 参数指定了推送的分支，origin/main 是远程分支的名称。#
```

从远程仓库复制文件到此电脑

新建文件夹，开启git bash

```
git init
git clone [remote address]  #若使用ssh出现读取不了的情况可以试一下http
```

ps.ssh可能会出现 以下问题

```
fatal: Could not read from remote repository.

Please make sure you have the correct access rights
and the repository exists.
```

可能是因为公钥没配好（？）

 取回远程仓库的变化时

```
git pull [remote] [branch] #remote即远程仓库的url，branch即要载入的分支
```

此时可能会出现 `fatal: refusing to merge unrelated histories `（此处使用的是http）

> 原因是两个仓库不同而导致的，需要在后面加上--allow-unrelated-histories进行允许合并，即可解决问题

> 如果还不能解决问题，就把本地的remote删除，重新git remote add添加远程仓库，再按上面的方法来，问题解决。

可能出现conflict，此时需要commit的双方协商修改，解决conflict

*Q：pull之后难道没有什么回退机制的吗？*

### **九、回退**

| 1. 恢复暂存区的指定文件到工作区                                       | $ git checkout [file]                                      |
| --------------------------------------------------------------------- | ---------------------------------------------------------- |
| 2. 重置当前HEAD为指定commit，同时重置暂存区和工作区，与指定commit一致 | $ git reset --hard [commit]                                |
| 3. 恢复暂存区所有文件到工作区                                         | $ git checkout .                                           |
| 4. 远程版本回退                                                       | 先本地版本回退后再使用命令：$ git push origin HEAD --force |

### **十、暂存**

| 1. 暂存                   | $ git stash                     |
| ------------------------- | ------------------------------- |
| 2. 查看暂存记录           | $ git stash list                |
| 3. 恢复暂存并删除这个记录 | $ git stash pop stash@{index}   |
| 4. 恢复暂存并保留这个记录 | $ git stash apply stash@{index} |
| 5. 删除某个暂存记录       | $ git stash drop stash@{index}  |
| 6. 删除全部暂存记录       | $ git stash clear               |

### **十一、其他**

| 1. 生成一个可供发布的压缩包 | $ git archive |
| --------------------------- | ------------- |

### **十二、其它问题**

1. `git pull`时 `error: cannot lock ref 'xxx': ref xxx is at （一个commitID） but expected`

1）使用 `git`命令删除相应 `refs`文件，`git update-ref -d refs/remotes/origin/git/yousa/feature_01`

2）简单粗暴强行 `git pull`，执行 `git pull -p`

2.`git push`时 `! [rejected]        main -> main (non-fast-forward) error: failed to push some refs to 'https://github.com/Labbbbbbbbb/WTR_learningnote.git'`

此时应先执行`git pull --rebase origin main `

再重新`git push -u origin main`

解决！
