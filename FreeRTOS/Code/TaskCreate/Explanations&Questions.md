## E

创建多个pc13与oled等多个任务，优先级相同，观察其现象

## Q

* 将任务中的osDelay(ms)删掉后发现这个任务将一直占据cpu，这是否说明任务调度方式是等一个任务进入阻塞时再启动下一个就绪态任务？(怎么跟SSC说的不一样得？) 那么怎么才能更换调度方式？（比如时间片之类的
* 好像也没有看到 `vTaskDelete( NULL )`函数捏？这是可以灵活选择要不要添加的嘛？
* `vTaskDelayUntil `的绝对延时到底是几个意思？
* wxz靴长说 `osStatus_t osDelay(uint32_ticks)`的单位是1ms但是看了它的函数内部感觉跟 `vTaskDelay`应该没什么差别的啊？为什么单位会不一样呢？以及systick的频率在哪里改呢？

>
> 可以看到HAL_Delay函数的目的是提供毫秒级别的延时，意味着当你输入HAL_Delay(500)，硬件会尽量延时精确到500ms的时间。
>
> 与之不同的是，osDelay函数的输入是ticks。ticks是一个计时单位，表示任务将被挂起的时间长度。每个tick的时间取决于FreeRTOS配置的时钟节拍（tick）周期。例如，如果tick周期为1毫秒，那么传递参数ticks为10就会使任务挂起10毫秒。由此可见，osDelay函数延时的时间和一个ticks记时时间长度有很大关系。
>
> 那么如何确定ticks具体代表多长时间呢？首先我们应该找到用于配置的头文件，通常这个头文件名字叫做FreeRTOSConfig.h。其中，configTICK_RATE_HZ配置选项的值表示每秒钟系统时钟节拍（tick）的数量。configTICK_RATE_HZ的值一般默认被设置为1000，表示系统时钟每秒产生1000个tick，即每个tick的时间间隔为1毫秒，此时osDelay对单个任务延时的时间长度和HAL_Delay近似。
>
> ==========
> ————————————————
>
>     版权声明：本文为博主原创文章，遵循 CC 4.0 BY-SA 版权协议，转载请附上原文出处链接和本声明。
>
> 原文链接：https://blog.csdn.net/m0_59766260/article/details/134098825
