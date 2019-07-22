# Four-Axis-Vehicle

README [English](docs/README_EN.md) | [中文](README.md)


## 项目介绍

[![MIT](https://img.shields.io/github/license/Crabor/Four-Axis-Vehicle.svg)](https://github.com/Crabor/Four-Axis-Vehicle/tree/65eae864c7d39532fc4979a467a7aa47c9dc2918/LICENSE/README.md) ![repo size](https://img.shields.io/github/repo-size/Crabor/Four-Axis-Vehicle.svg)

本项目是电子科技大学信息与软件工程学院综合课程设计课题：基于STM32F4的四轴飞行器。主要内容包括：

* STM32嵌入式开发平台上扩展姿态传感模块，用于确定四轴飞行器的飞行姿态

* 姿态传感器采集数据后，在STM32上完成姿态解算

* STM32上通过PID控制及PMW电机调速，使四轴飞行器能够通过调节四轴的扇叶转速保持平稳的飞行，并不断调整以达到稳定，如图所示

![ ](https://i.loli.net/2019/07/22/5d359b246f42576211.png)

## 硬件要求

| 部件           | 型号                       |
| :----------------: | :----------------------------: |
| 机架           | F450 (螺旋桨螺距为450mm) |
| 无刷电机     | 好盈Skywalker-20A          |
| 马达           | A2212/13T（1000KV）        |
| 螺旋桨        | 乾丰1045                   |
| 电池           | 格式航模电池（2200mAh 25C） |
| 开发板        | STM32F401RE（最高频84MHz） |
| 加速度计、陀螺仪 | MPU6050（集成在MPU6050） |
| 电子罗盘     | HMC5883L（集成在MPU6050） |
| 高精度气压传感器 | MS5611                       |
| 遥控器        | RadioLink T4EU-6（4通道） |
| 遥控接收器  | RadioLink R7EH-S（7通道） |
| 电池           | 格氏航模电池（2200mAh、25C） |
| 蓝牙           | HM-10 * 2                    |
| 扩展板       | Altium Designer软件设计  |

## 软件要求

操作系统：`Windows`或者`Linux`

IDE：`Keil v5`

编译器：`ARM Compiler`（`Keil uVision`内置）或者 `gcc-arm-none-eabi`（`Linux`）

## 快速使用

* `Windows`

  1. 打开`Project/exp4.uvprojx`

  1. 点击编译按钮

  1. 烧录运行

* `Linux`（已安装`gcc-arm-none-eabi`、`st-flash`）

  1. 输入命令`make`生成`hex`文件

  1. 输入命令`make burn`进行烧录

## 鸣谢

廖勇老师、[施荣圳师兄](https://github.com/AlronSze)

正点原子、野火电子
