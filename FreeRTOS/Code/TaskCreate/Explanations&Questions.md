## E

创建多个pc13与oled等多个任务，优先级相同，观察其现象

## Q

* 将任务中的osDelay(ms)删掉后发现这个任务将一直占据cpu，这是否说明任务调度方式是等一个任务进入阻塞时再启动下一个就绪态任务？那么怎么才能更换调度方式？（比如时间片之类的