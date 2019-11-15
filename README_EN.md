# Quadcopter

README [English](README_EN.md) | [中文](README.md)

## Introduction

[![MIT](https://img.shields.io/github/license/Crabor/Quadcopter.svg)](LICENSE)
![platform](https://img.shields.io/badge/platform-windows%20%7C%20linux-orange.svg)

This is an Integrated Course Design of School of Information and Software Engineering of UESTC: Quadcopter Based on STM32F401RE. It contains:

* Extending attitude sensor module on STM32 board to determine the flying attitude of quadcopter

* Complete the task of attitude calculation on STM32 board after the data from attitude sensor module are collected

* Through the PID control and PWM motor speed regulation on STM32, the quadcopter can maintain stable flight by adjusting the speed of the propeller, and constantly adjust to achieve stability, as shown in the figure.

![ ](https://i.loli.net/2019/07/23/5d36c3cba3aa786036.bmp)

## Hardware Requirement

| Components                  | Type and Parameters                                 |
| :----------------:          | :----------------------------:                      |
|       Quadcopter Rack       |           F450 (Propeller pitch is 450mm)           |
| Brushless Motor             |               Hobbywing Skywalker-20A               |
|            Motor            |                  A2212/13T (1000KV)                  |
|          Propeller          |  Gemfan 1045 (Diameter is 10 inches; Blade angle is 45°)   |
|           Battery           |       ACE 2200mAh 25C lithium-polymer battery       |
|     Development Board      |              STM32F401RE (Up to 84MHZ)              |
|  Accelerometer & Gyroscope  |            MPU6050 (Integrated in GY-86/MPU-9150)            |
|        Magnetometer         |           HMC5883L(Integrated in GY-86) or AK8975(Integrated in MPU-9150)           |
|      Remote Controller      | RadioLink T4EU-6 2.4G six-channels remote controller |
|       Remote Receiver       | RadioLink R7EH-S 2.4G six-channels remote receiver  |
|          Bluetooth          |            HM-10 * 2            |
|       Expansion Board       |         Design and draw using Altium Desogner          |

## Software Requirement

Operating System: `Windows` or `Linux`

IDE: `Keil v5`

Compiler" `ARM Compiler`(Integrated in `Keil uVision`) or `gcc-arm-none-eabi`(`Linux`)

## Project Structure

```txt
├── docs
│   ├── 电压分配.txt
│   ├── 引脚配置.txt
│   └── 九轴数据变化.txt
├── PCB
│   ├── IntLib
│   └── PCB
│       ├── pcb.PcbDoc  //PCB file
│       ├── PCB_Project.PrjPcb  //AD project file
│       └── pcb.SchDoc  //schematic diagram file
└── Project
    ├── APP  // user file
    │   ├── app_cfg.h  //selecting compiler options & APP_FUNC switch
    │   ├── includes.h  //head files set, see Development instructions
    │   ├── main.c  //main file
    │   ├── os_cfg.h  //ucosii module switch
    │   └── RTE_Components.h  //STM32 standard lib switch
    ├── APP_FUNC  //user lib
    │   ├── delay.c  //different from OSTimeDly()
    │   ├── delay.h
    │   ├── gpio.c
    │   ├── gpio.h
    │   ├── iic.c
    │   ├── iic.h
    │   ├── IMU.c  //attitude calculation
    │   ├── IMU.h
    │   ├── led.c
    │   ├── led.h
    │   ├── MPU9150.c  //GY-86、MPU9150
    │   ├── MPU9150.h
    │   ├── nvic.c
    │   ├── nvic.h
    │   ├── pid.c  //PID algorithm
    │   ├── pid.h
    │   ├── pwm.c
    │   ├── pwm.h
    │   ├── pwm_led.c
    │   ├── pwm_led.h
    │   ├── tim_led.c
    │   ├── tim_led.h
    │   ├── usart.c
    │   └── usart.h
    ├── BSP_SELF  //hardware related, need modify
    │   ├── BSP.c  //bsp init, including Systick interrupt config
    │   ├── BSP.h
    │   ├── startup_stm32f401xx.s  //startup file, had modified PendSV interrupt
    │   ├── stm32f4xx_it.c  //about interrupt
    │   ├── stm32f4xx_it.h
    │   ├── system_stm32f4xx.c  //Sysclock config
    │   └── system_stm32f4xx.h
    ├── BSP_SYS  //hardware related, don`t need modify
    │   ├── core_cm4.h
    │   ├── core_cm4_simd.h
    │   ├── core_cmFunc.h
    │   ├── core_cmInstr.h
    │   ├── stm32f4xx_conf.h
    │   ├── stm32f4xx.h
    │   ├── system_stm32f4xx.c
    │   └── system_stm32f4xx.h
    ├── CLIB  //C language lib
    ├── STLIB  //STM32 standard lib
    ├── UCOS_PORT  //ucosii hardware related
    │   ├── os_cpu_a.s
    │   ├── os_cpu_c.c
    │   ├── os_cpu.h
    │   └── os_dbg.c
    └── UCOS_SOURCE  //ucosii module
        ├── os_core.c
        ├── os_core.h
        ├── os_flag.c
        ├── os_mbox.c
        ├── os_mem.c
        ├── os_mutex.c
        ├── os_q.c
        ├── os_sem.c
        ├── os_task.c
        ├── os_time.c
        ├── os_tmr.c
        ├── ucos_ii.c
        └── ucos_ii.h
```

## Development Instructions

Head files relationship:

```txt
includes.h
├── BSP.h
├── core_cm4.h
├── stm32f4xx.h
│   └── stm32f4xx_conf.h
│       └── RTE_Components.h  //standard lib switch
│           ├── stm32f4xx_gpio.h
│           ├── ......
│           └── stm32f4xx_rcc.h
└── ucosii.h
    ├── app_cfg.h  //user file switch
    │   ├── gpio.h
    │   ├── ......
    │   └── pid.h
    └── os_cfg.h
```

Attention! There a piece of code in "stm32f4xx.h"

```c
#ifdef USE_STDPERIPH_DRIVER
    #includes "stm32f4xx_conf.h"
#endif
```

That means you should define **USE_STDPERIPH_DRIVER** to include **stm32f4xx_conf.h**.

If you want to add file in **APP_FUNC** folder, your code style should look like this and turn on the corresponding switch in **app_cfg.h**:

```c
//xx.h
#ifndef XX_H
#define XX_H
#includes "includes.h"
//#includes "math.h"  
//C lib file should be added separately, if you need

......

#endif
```

```c
//xx.c
#includes "xx.h"

......
```

## Get Started

* `Windows` (Keil uVision has been installed)

  1. Open `Project/exp4.uvprojx`

  1. Click button `Build`

  1. Click button `DownLoad` to burn

* `Linux` (`gcc-arm-none-eabi` and `st-flash` have been installed)(**Coming soon**)

  1. Execute `make` instruction

  1. Execute `make burn` instruction to burn

## Acknowledgement

LIAO Yong, [Alron](https://github.com/AlronSze)

## Contact

[![jianshu](https://img.shields.io/badge/-Black__Mirror-white.svg?style=social&logo=data:image/png;base64,iVBORw0KGgoAAAANSUhEUgAAAA4AAAAOCAYAAAAfSC3RAAABUUlEQVQ4T52SQUsCQRTH/4NokVQsGC1sYeChS2AXO9bRY+cufgGh1u4W2D23Q1+gT+A1j3tKO0kdDApW0DAka1lztWWdmAezSKVY/8sw895vZt57fwYAzYKug+MIjG2I/URxbnEwY/20eMEIAitOBX4EeY41CzkLQFzE5je3EE2m4FRMSvXfu1D2D9C7MTEaurT37TeAc0uAXCSFlhWsZLLwXp4xF0+gVzURXtXogrCqYWg9ol+rYth4orMAFMkhJYaR+4GIqsGt3xHQr91iIZmC127RpVIBKF9VD/PoVUyCxxWJJ9A6O/4dFF/tXF1iaS8NzqkCkmOWoeXPJ4MyKFa7XKLvxTJZAmYGRW2+3cXibvpvoH1dwmjg0jhmflHUKOW9duA7No3qe3MCA8jmjHdTzDe6vYPPdguDh3sZavzfcoHJwXRpvSm+bQDcWDsxjC9+b722Av5UFwAAAABJRU5ErkJggg==)](https://www.jianshu.com/u/68e65b750b62)
![QQ](https://img.shields.io/badge/-564102478-white.svg?style=social&logo=data:image/png;base64,iVBORw0KGgoAAAANSUhEUgAAAA4AAAAOCAYAAAAfSC3RAAABF0lEQVQ4T42SQU7CQBSG/99CMl3JDWwj7OEE1hu4dVGDJzCcgHoDbkBsQ1zqDRhPoPs2KUdgZ40ZnpnGYikCneWb+f7/vX8e0TjdRT7sbMwUggBETwQrALowzgT3/rp6zjpXQsYsLdAUhGAtXWdU3PpWCDugSrIVgYs96LcgwFMR9sc7oErSgODyEFTVP8N+abZ1VEkaEZyeAgVyXYQD/QfGqSZ51QJ8LMJBVIJHQ2koVXMS87znOib/N8kD9tJxfLpJNgPwcKrF+r11pYpTDXJI4LwtLCJvpaMBtAO8tAE3wskZxaulmo1JzI/BewugnnOP3+a9TUDW8evucrZ1dOPsVYigmtWqQ2A/+gYUT0APIh8gIrsAP1/XbEGbe63NAAAAAElFTkSuQmCC)
![wechat](https://img.shields.io/badge/-troy15270658794-white.svg?style=social&logo=data:image/png;base64,iVBORw0KGgoAAAANSUhEUgAAABEAAAAOCAYAAADJ7fe0AAABSklEQVQ4T4WSIVLDUBCGv80U3WoM3IBGEAs9AEN7AorDMQwCHEUygwCHbE9AUUiqU2YSblAcjlYh6MxjdvMCCSRlTbJv933Z/88KeSS0WHKMo4vQBuZACkxocEtoeWWInca0EZ6AVk3fHEeHyKB/QkjYZEliAMcLYl/c8Z2vfpp9m6xBSMjsN0WIGSIcWEHo2dNx7xtPEGbfuWNERL8KkngP6iQvEIbAB87kvuF4KErTSVKErUqCygt4xHFeUVfDe2p4WU65cwHcAWf+WPNm6d2REhHmxqrreUOOUsCR92hEwNi80YtrdFjybrWAvewXP9PFme4i6KowRZ1f+jOuM4jGlDGw67MmjguEy/rbviKcFiEDGtyUNnNqO7GxEiSs/0CqOrNNnlT4lXU7DokYroZoo270JwOEbgGmmzxjO5P/P6ROS0xfp9DyF8keXaKcVCZ8AAAAAElFTkSuQmCC)
[![twitter](https://img.shields.io/twitter/follow/Arya_PPGYR.svg?label=Follow&style=social)](https://twitter.com/Arya_PPGYR)

## About

My learning process is recorded on [Wiki](https://github.com/Crabor/Quadcopter/wiki).
