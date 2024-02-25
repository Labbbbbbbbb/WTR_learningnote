# Cube

创建任务可以在cube中直接点击Tasks处的Add，然后一般修改名字，Priority和Code Generation Option

* Priority：任务优先级，从上至下优先级越来越高。（数值越高优先级越高）
* Stack Size：任务的栈的大小，默认给的数值一般不会不够用，可以不用改
* Entry Function：入口函数名
* Code Generation Option：函数定义方式（一般，弱，外部）
* Parameter：参数
* Alloction：任务创建方式：动态/静态

> 创建任务的时候需要给任务指定堆栈，如果使用的函数 xTaskCreate()创建任务(动态方法)的话那么任务堆栈就会由函数 xTaskCreate()自动创建。如果使用函数 xTaskCreateStatic()创建任务(静态方法)的话就需要程序员自行定义任务堆栈，然后堆栈首地址作为函数的参数 puxStackBuffer 传递给函数。原文-->[CSDN](https://blog.csdn.net/qq_44318582/article/details/120153001)

![1708248333357](image/cube配置及原理/1708248333357.png)

注意task中有一个默认的default Task，可以将其修改为弱定义，以便在另外的文件夹复写，从而避免修改cube自动生成的文件。

# 代码

然后在freertos.c中就会自动生成：

![1708333562454](image/TaskCreat/1708333562454.png)

> 创建任务句柄；根据cube选项卡创建任务参数的结构体^

以及

![1708334061644](image/TaskCreat/1708334061644.png)

然后就可以直接在上图的任务函数中敲执行代码了！

这里注意一下如果是像cyt靴长一样使用弱定义，可以另建User文件夹来复写任务函数；是Default的话就直接在freertos.c中写。main中不需要自行添加与freertos有关的东西！（如果用了其他外设什么的一些初始化函数或者是定义什么变量可以写在main里，也可以视情况写在某个任务的死循环之前）

---

# 关于API

### 创建任务

（借鉴[知乎](https://zhuanlan.zhihu.com/p/514869870)）

在这个版本中创建函数使用的是**osThreadNew()**函数

```
/* creation of defaultTask */  defaultTaskHandle=osThreadNew(StartDefaultTask,NULL,&defaultTask_attributes);
/* creation of myTask02 */  
myTask02Handle=osThreadNew(StartTask02,NULL,&myTask02_attributes);

//这几句在freertos.c的void MX_FREERTOS_Init(void)中
```

下为该函数的定义：

```
osThreadId_t osThreadNew (osThreadFunc_t func, void *argument, const osThreadAttr_t *attr)
 {
  /*创建变量*/
  char empty;
  const char *name;
  uint32_t stack;
  TaskHandle_t hTask;
  UBaseType_t prio;
  int32_t mem;		//mem是与静态和动态的选择有关的标志变量
  
  /*初始化变量*/
  hTask = NULL;

  if (!IS_IRQ() && (func != NULL)) {
    stack = configMINIMAL_STACK_SIZE;
    prio  = (UBaseType_t)osPriorityNormal;

    empty = '\0';
    name  = &empty ;
    mem   = -1;

  /*将结构体attr中的值对应赋给变量，并传入xTaskCreate函数中*/
    if (attr != NULL) {
      if (attr->name != NULL) {
        name = attr->name;
      }
      if (attr->priority != osPriorityNone) {
        prio = (UBaseType_t)attr->priority;
      }

      if ((prio < osPriorityIdle) || (prio > osPriorityISR) || ((attr->attr_bits & osThreadJoinable) == osThreadJoinable)) {
        return (NULL);
      }

      if (attr->stack_size > 0U) {
        /* In FreeRTOS stack is not in bytes, but in sizeof(StackType_t) which is 4 on ARM ports.       */
        /* Stack size should be therefore 4 byte aligned in order to avoid division caused side effects */
        stack = attr->stack_size / sizeof(StackType_t);
      }

      if ((attr->cb_mem    != NULL) && (attr->cb_size    >= sizeof(StaticTask_t)) &&
          (attr->stack_mem != NULL) && (attr->stack_size >  0U)) {
        mem = 1;
      }
      else {
        if ((attr->cb_mem == NULL) && (attr->cb_size == 0U) && (attr->stack_mem == NULL)) {
          mem = 0;
        }
      }
    }
    else {
      mem = 0;
    }

    if (mem == 1) {
      hTask = xTaskCreateStatic ((TaskFunction_t)func, name, stack, argument, prio, (StackType_t  *)attr->stack_mem,
                                                                                    (StaticTask_t *)attr->cb_mem);
    }
    else {
      if (mem == 0) {
        if (xTaskCreate ((TaskFunction_t)func, name, (uint16_t)stack, argument, prio, &hTask) != pdPASS) {
          hTask = NULL;
        }
      }
    }
  }

  return ((osThreadId_t)hTask);
}
```

osThreadNew()的三个参数：

> · **func**为xTaskCreate()和xTaskCreateStatic()中的第一个入口参数，是一个函数指针，指向**执行任务的函数(是的就是我们自己编写执行内容的那个）**，对应cube中的Entry Function.
>
> · **argument**为xTaskCreate()和xTaskCreateStatic()中的第四个入口参数，是传递给任务的参数，不用时设为NULL。
>
> · **attr** 可以理解为包含了.name，.stack_size，.priority等内容的结构体，分别覆盖了xTaskCreate()和xTaskCreateStatic()中的第二、三、五个入口参数，这些信息皆在cube中配置生成。

**可以发现这个函数是对freertos原装函数 xTaskCreate()，xTaskCreateStatic() 的封装**

再来看一下这两个函数的声明：

```
#if( configSUPPORT_DYNAMIC_ALLOCATION == 1 )

	BaseType_t xTaskCreate(	TaskFunction_t pxTaskCode,
				const char * const pcName,
				const configSTACK_DEPTH_TYPE usStackDepth,
				void * const pvParameters,
				UBaseType_t uxPriority,
				TaskHandle_t * const pxCreatedTask )
```

· **pvTaskCode** 这是一个函数指针，指向执行任务的函数。

· **pcName** 任务的描述名称，方便调试，不用的话可以设为Null。

· **usStackDepth** 每个任务有自己的栈空间，这里根据任务占用需求设置栈空间的大小。

· **pvParameters** 用于传递给任务的参数，不用的话可以设为Null。

· **uxPriority** 设置任务的优先级，范围由0到(configMAX_PRIORITIES – 1)。数值越大，等级越高。

· **pxCreatedTask** 任务的句柄（handle），通过句柄可以对任务进行设置，比如改变任务优先级等，不用可以设为Null。

函数的返回值有两个pdPass和pdFail，pdPass表示任务创建成功，相反pdFail表示创建失败，创建失败的原因大多是因为系统没有足够的堆空间来保存任务的数据。

```
#if( configSUPPORT_STATIC_ALLOCATION == 1 )

	TaskHandle_t xTaskCreateStatic(	TaskFunction_t pxTaskCode,
					const char * const pcName,
					const uint32_t ulStackDepth,
					void * const pvParameters,
					UBaseType_t uxPriority,
					StackType_t * const puxStackBuffer,
					StaticTask_t * const pxTaskBuffer )
```

**xTaskCreate()与 xTaskCreateStatic()的区别：**

**xTaskCreate是操作系统自动分配内存，xTaskCreateStatic是需要程序员手动定义内存。**

· 使用动态方法创建任务（xTaskCreate）的时候需要将宏 configSUPPORT_DYNAMIC_ALLOCATION设置为 1

· 使用静态方法创建任务（xTaskCreateStatic）的时候需要将宏 configSUPPORT_STATIC_ALLOCATION 设为1

宏定义位置Core/Inc/ FreeRTOSConfig.h

### 延时函数

freertos原装函数：

```
/**
  * @brief 相对延时函数
  * @param xTicksToDelay  systick系统节拍数(一般为1ms)
  */
void vTaskDelay( const TickType_t xTicksToDelay );

/**
  * @brief 绝对延时函数
  * @param pxPreviousWakeTime 保存上次唤醒时间的指针,使用xTaskGetTickCount()获得
  * @param xTimeIncrement 周期循环时间。当时间等于(*pxPreviousWakeTime + xTimeIncrement)时，任务解除阻塞。如果不改变参数xTimeIncrement的值，调用该函数的任务会按照固定频率执行。
  */
void vTaskDelayUntil( TickType_t * const pxPreviousWakeTime, const TickType_t xTimeIncrement);
```

cmsis函数：

```
/**
 * @brief 相对延时函数
 * @param ticks  延迟时间(systicks数)
 * /
osStatus_t osDelay (uint32_t ticks);

/**
 * @brief 绝对延时函数
 * @param ticks  延迟时间(systicks数)
 * /
osStatus_t osDelayUntil (uint32_t ticks);
```

> 两者的关系--->[RTX5 | 时间延时](https://blog.csdn.net/wallace89/article/details/117933438)
>
> 总的来说，`osDelayUntil`从 `osKernelGetTickCount（）`中获取绝对时间，实现绝对延时；

freertos的程序**不能使用HAL_Delay**！原因是其与freertos的延迟作用是不同的

> HAL_Delay是由ST提供的STM32 Cube HAL库中的一个函数，通常用于在STM32微控制器上实现简单的延时。HAL_Delay函数使用系统时钟来进行延时，并且在延时期间会阻塞整个处理器，也就是说，它会使处理器暂时停止执行其他任务和代码。
>
> 在开始运行线程之前，线程A、B处于就绪态，由于线程A优先级比线程B高，FreeRTOS任务控制器优先选择线程A运行，此时线程A进入运行态。随后线程A打印A，然后被HAL_Delay函数"阻塞"，注意此时的"阻塞"并不意味着程序进入了阻塞态，由于HAL_Delay阻塞的是整个处理器，因此FreeRTOS无法进行其他线程的调度，也就是说，HAL_Delay同时阻塞了线程B。当HAL_Delay函数运行结束后，线程A重回就绪态，由于线程A优先级比线程B高，FreeRTOS任务控制器优先选择线程A运行，循环往复，线程B不被执行
>
> -->[FreeRTOS中osDelay和HAL_Delay的区别](https://blog.csdn.net/m0_59766260/article/details/134098825)
