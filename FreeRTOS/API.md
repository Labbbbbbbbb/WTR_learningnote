# FreeRTOS API介绍

FreeRTOS的API包括官网的原生API和CMSIS-RTOS提供的API

> CMSIS是 ARM公司制定的标准，芯片厂商按照此标准编写相应的程序，实现统一的接口，方便开发人员的使用。CMSIS-RTOS是其中一种抽象RTOS的接口。——来源于[知乎](https://zhuanlan.zhihu.com/p/551954775)
>
> CMSIS-RTOS在用户的应用代码和第三方的RTOS
> Kernel直接架起一道桥梁，一个设计在不同的RTOS、Cortex
> MCU移植的时候，如果两个RTOS都实现了CMSIS-RTOS，那么用户的应用程序代码可以不做修改。——来源于[电子发烧友](https://www.elecfans.com/d/2192556.html)

## FreeRTOS原生API

特点：接口以x、v开头，例如vTaskDelay(); 执行效率较高，内存用量较少；

你可以在

* 官网 [FreeRTOS API categories](https://www.freertos.org/zh-cn-cmn-s/a00106.html)
* cube生成的库文件Middlewares\Third_Party\FreeRTOS\Source\include

中找到这些api

## CMSIS-RTOS-API

The CMSIS-RTOS2 API is using the following **Namespace** prefixes:

* **os** for all definitions and function names.
* **os** with postfix **_t** for all typedefs.

你可以在

* [ARM](https://arm-software.github.io/CMSIS_5/latest/RTOS2/html/rtos_api2.html)[CMSIS_v2](https://arm-software.github.io/CMSIS_5/latest/RTOS2/html/rtos_api2.html)
* 库文件Middlewares\Third_Party\FreeRTOS\Source\CMSIS_RTOS_V2\cmsis_os2.h：283-end行

中找到这些api


除此之外，使用Cube生成FreeRTOS项目后，可以从生成好的freertos.c文件中找到在CubeMX中点任务、信号量等。
