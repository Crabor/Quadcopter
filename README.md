# Quadcopter

README [English](README_EN.md) | [中文](README.md)

## 项目介绍

[![MIT](https://img.shields.io/github/license/Crabor/Quadcopter.svg)](LICENSE)
![platform](https://img.shields.io/badge/platform-windows%20%7C%20linux-orange.svg)

本项目是电子科技大学信息与软件工程学院综合课程设计课题：基于STM32F4的四轴飞行器。主要内容包括：

* STM32嵌入式开发平台上扩展姿态传感模块，用于确定四轴飞行器的飞行姿态

* 姿态传感器采集数据后，在STM32上完成姿态解算

* STM32上通过PID控制及PWM电机调速，使四轴飞行器能够通过调节四轴的扇叶转速保持平稳的飞行，并不断调整以达到稳定，如图所示

![ ](https://i.loli.net/2019/07/22/5d359b246f42576211.png)

## 硬件要求

| 部件           | 型号                       |
| :----------------: | :----------------------------: |
| 机架           | F450 (螺旋桨螺距为450mm) |
| 无刷电机     | 好盈Skywalker-20A          |
| 马达           | A2212/13T（1000KV）        |
| 螺旋桨        | 乾丰1045                   |
| 电池           | 格氏航模电池（2200mAh 25C） |
| 开发板        | STM32F401RE（最高频84MHz） |
| 加速度计、陀螺仪 | MPU6050（集成在MPU6050） |
| 电子罗盘     | HMC5883L（集成在MPU6050） |
| 高精度气压传感器 | MS5611                       |
| 遥控器        | RadioLink T4EU-6（4通道） |
| 遥控接收器  | RadioLink R7EH-S（7通道） |
| 蓝牙           | HM-10 * 2                    |
| 扩展板       | Altium Designer软件设计  |

## 软件要求

操作系统：`Windows`或者`Linux`

IDE：`Keil v5`

编译器：`ARM Compiler`（`Keil uVision`内置）或者 `gcc-arm-none-eabi`（`Linux`）

## 快速开始

* `Windows`（已安装`Keil uVision`）

  1. 打开`Project/exp4.uvprojx`

  1. 点击`Build`按钮

  1. 点击`Download`按钮烧录

* `Linux`（已安装`gcc-arm-none-eabi`、`st-flash`）（**暂未实现**）

  1. 输入命令`make`生成`hex`文件

  1. 输入命令`make burn`进行烧录

## 鸣谢

廖勇老师、[施荣圳师兄](https://github.com/AlronSze)

[正点原子](http://www.openedv.com/)、[野火电子](http://www.proewildfire.cn/)

## 小组成员

杨嵘、尧松、卓子豪、廖新语、赖鑫

## 联系

[![jianshu](https://img.shields.io/badge/-Black__Mirror-white.svg?style=social&logo=data:image/png;base64,iVBORw0KGgoAAAANSUhEUgAAAA4AAAAOCAYAAAAfSC3RAAABUUlEQVQ4T52SQUsCQRTH/4NokVQsGC1sYeChS2AXO9bRY+cufgGh1u4W2D23Q1+gT+A1j3tKO0kdDApW0DAka1lztWWdmAezSKVY/8sw895vZt57fwYAzYKug+MIjG2I/URxbnEwY/20eMEIAitOBX4EeY41CzkLQFzE5je3EE2m4FRMSvXfu1D2D9C7MTEaurT37TeAc0uAXCSFlhWsZLLwXp4xF0+gVzURXtXogrCqYWg9ol+rYth4orMAFMkhJYaR+4GIqsGt3xHQr91iIZmC127RpVIBKF9VD/PoVUyCxxWJJ9A6O/4dFF/tXF1iaS8NzqkCkmOWoeXPJ4MyKFa7XKLvxTJZAmYGRW2+3cXibvpvoH1dwmjg0jhmflHUKOW9duA7No3qe3MCA8jmjHdTzDe6vYPPdguDh3sZavzfcoHJwXRpvSm+bQDcWDsxjC9+b722Av5UFwAAAABJRU5ErkJggg==)](https://www.jianshu.com/u/68e65b750b62)
[![weibo](https://img.shields.io/badge/-Winterhell__-white.svg?style=social&logo=data:image/png;base64,iVBORw0KGgoAAAANSUhEUgAAABEAAAAOCAYAAADJ7fe0AAAB10lEQVQ4T5WTP3rTQBTEZ1aRqQCXfB/+lwbLVcQJYm4g38A5AbmBkxNgToByAsgJECdwqBRobKTQGzokpOHbteXESUPUSNrV/t68mSfiEVfaa60I9AHYezzMinN7nI9g4FvXi2pxTTCCwRTCIsjLNw8g6QsMjO/PRES2gGpNRjd/E/v8/aUf1oYzorpgbVa1h0TgfA+Sdv0piXcA241CAeejrDhrIJXHiNCphFMKIQyOd5ANgB/ut0cploVulVHVpKYXWoBUz2nMZwexvYrex//yR7i0hw3VFs1A4CmXbbT/PPOXTQuSLgAkkFYNdGck+NwqG+blid3b+OeFvO74cxi+BfRLtSJrYto5GBtjjgGthllpoXDFnvoJyCMbsak0efWzvHIRN9mbSq/t4i10q0P4FOTF5EHbwmWQFy5BXvdagvQ1yMvQymOrtUSt9yAHoPoAwyArnHcuYo8LF730ZZSX442Srp+Q7AdZcWg/qgzmdvNum0FWusjTXuuMwGwLORnlZewgGzoSm7tdTLt+7EabHO/5dGcErPmjvJzeGm8rdA7GMCamsAAqC6Edb09cy8ORhOkOCs6b4duDNC/u34CJCITbFCDgB4UrUMmT32V8uMb6/jz9Axzp8nOYDG1qAAAAAElFTkSuQmCC)](https://www.weibo.com/u/2694246064?topnav=1&wvr=6&topsug=1&is_all=1)
![QQ](https://img.shields.io/badge/-564102478-white.svg?style=social&logo=data:image/png;base64,iVBORw0KGgoAAAANSUhEUgAAAA4AAAAOCAYAAAAfSC3RAAABF0lEQVQ4T42SQU7CQBSG/99CMl3JDWwj7OEE1hu4dVGDJzCcgHoDbkBsQ1zqDRhPoPs2KUdgZ40ZnpnGYikCneWb+f7/vX8e0TjdRT7sbMwUggBETwQrALowzgT3/rp6zjpXQsYsLdAUhGAtXWdU3PpWCDugSrIVgYs96LcgwFMR9sc7oErSgODyEFTVP8N+abZ1VEkaEZyeAgVyXYQD/QfGqSZ51QJ8LMJBVIJHQ2koVXMS87znOib/N8kD9tJxfLpJNgPwcKrF+r11pYpTDXJI4LwtLCJvpaMBtAO8tAE3wskZxaulmo1JzI/BewugnnOP3+a9TUDW8evucrZ1dOPsVYigmtWqQ2A/+gYUT0APIh8gIrsAP1/XbEGbe63NAAAAAElFTkSuQmCC)
![wechat](https://img.shields.io/badge/-troy15270658794-white.svg?style=social&logo=data:image/png;base64,iVBORw0KGgoAAAANSUhEUgAAABEAAAAOCAYAAADJ7fe0AAABSklEQVQ4T4WSIVLDUBCGv80U3WoM3IBGEAs9AEN7AorDMQwCHEUygwCHbE9AUUiqU2YSblAcjlYh6MxjdvMCCSRlTbJv933Z/88KeSS0WHKMo4vQBuZACkxocEtoeWWInca0EZ6AVk3fHEeHyKB/QkjYZEliAMcLYl/c8Z2vfpp9m6xBSMjsN0WIGSIcWEHo2dNx7xtPEGbfuWNERL8KkngP6iQvEIbAB87kvuF4KErTSVKErUqCygt4xHFeUVfDe2p4WU65cwHcAWf+WPNm6d2REhHmxqrreUOOUsCR92hEwNi80YtrdFjybrWAvewXP9PFme4i6KowRZ1f+jOuM4jGlDGw67MmjguEy/rbviKcFiEDGtyUNnNqO7GxEiSs/0CqOrNNnlT4lXU7DokYroZoo270JwOEbgGmmzxjO5P/P6ROS0xfp9DyF8keXaKcVCZ8AAAAAElFTkSuQmCC)
[![twitter](https://img.shields.io/twitter/follow/Arya_PPGYR.svg?label=Follow&style=social)](https://twitter.com/Arya_PPGYR)

## 关于

我学习四轴的一些重要知识、疑难解答等记录过程都放在了[Wiki](https://github.com/Crabor/Quadcopter/wiki)上！欢迎查看！
