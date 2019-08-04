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
|  Accelerometer & Gyroscope  |            MPU6050 (Integrated in GY-86)            |
|        Magnetometer         |           HMC5883L (Integrated in GY-86)            |
|      Remote Controller      | RadioLink T4EU-6 2.4G four-channels remote controller |
|       Remote Receiver       | RadioLink R7EH-S 2.4G seven-channels remote receiver  |
|          Bluetooth          |            HM-10 * 2            |
|       Expansion Board       |         Design and draw using Altium Desogner          |

## Software Requirement

Operating System: `Windows` or `Linux`

IDE: `Keil v5`

Compiler" `ARM Compiler`(Integrated in `Keil uVision`) or `gcc-arm-none-eabi`(`Linux`)

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

[Openedv](http://www.openedv.com/), [WildFire](http://www.proewildfire.cn/)

## Contact

[![jianshu](https://img.shields.io/badge/-Black__Mirror-white.svg?style=social&logo=data:image/png;base64,iVBORw0KGgoAAAANSUhEUgAAAA4AAAAOCAYAAAAfSC3RAAABUUlEQVQ4T52SQUsCQRTH/4NokVQsGC1sYeChS2AXO9bRY+cufgGh1u4W2D23Q1+gT+A1j3tKO0kdDApW0DAka1lztWWdmAezSKVY/8sw895vZt57fwYAzYKug+MIjG2I/URxbnEwY/20eMEIAitOBX4EeY41CzkLQFzE5je3EE2m4FRMSvXfu1D2D9C7MTEaurT37TeAc0uAXCSFlhWsZLLwXp4xF0+gVzURXtXogrCqYWg9ol+rYth4orMAFMkhJYaR+4GIqsGt3xHQr91iIZmC127RpVIBKF9VD/PoVUyCxxWJJ9A6O/4dFF/tXF1iaS8NzqkCkmOWoeXPJ4MyKFa7XKLvxTJZAmYGRW2+3cXibvpvoH1dwmjg0jhmflHUKOW9duA7No3qe3MCA8jmjHdTzDe6vYPPdguDh3sZavzfcoHJwXRpvSm+bQDcWDsxjC9+b722Av5UFwAAAABJRU5ErkJggg==)](https://www.jianshu.com/u/68e65b750b62)
[![weibo](https://img.shields.io/badge/-Winterhell__-white.svg?style=social&logo=data:image/png;base64,iVBORw0KGgoAAAANSUhEUgAAABEAAAAOCAYAAADJ7fe0AAAB10lEQVQ4T5WTP3rTQBTEZ1aRqQCXfB/+lwbLVcQJYm4g38A5AbmBkxNgToByAsgJECdwqBRobKTQGzokpOHbteXESUPUSNrV/t68mSfiEVfaa60I9AHYezzMinN7nI9g4FvXi2pxTTCCwRTCIsjLNw8g6QsMjO/PRES2gGpNRjd/E/v8/aUf1oYzorpgbVa1h0TgfA+Sdv0piXcA241CAeejrDhrIJXHiNCphFMKIQyOd5ANgB/ut0cploVulVHVpKYXWoBUz2nMZwexvYrex//yR7i0hw3VFs1A4CmXbbT/PPOXTQuSLgAkkFYNdGck+NwqG+blid3b+OeFvO74cxi+BfRLtSJrYto5GBtjjgGthllpoXDFnvoJyCMbsak0efWzvHIRN9mbSq/t4i10q0P4FOTF5EHbwmWQFy5BXvdagvQ1yMvQymOrtUSt9yAHoPoAwyArnHcuYo8LF730ZZSX442Srp+Q7AdZcWg/qgzmdvNum0FWusjTXuuMwGwLORnlZewgGzoSm7tdTLt+7EabHO/5dGcErPmjvJzeGm8rdA7GMCamsAAqC6Edb09cy8ORhOkOCs6b4duDNC/u34CJCITbFCDgB4UrUMmT32V8uMb6/jz9Axzp8nOYDG1qAAAAAElFTkSuQmCC)](https://www.weibo.com/u/2694246064?topnav=1&wvr=6&topsug=1&is_all=1)
![QQ](https://img.shields.io/badge/-564102478-white.svg?style=social&logo=data:image/png;base64,iVBORw0KGgoAAAANSUhEUgAAAA4AAAAOCAYAAAAfSC3RAAABF0lEQVQ4T42SQU7CQBSG/99CMl3JDWwj7OEE1hu4dVGDJzCcgHoDbkBsQ1zqDRhPoPs2KUdgZ40ZnpnGYikCneWb+f7/vX8e0TjdRT7sbMwUggBETwQrALowzgT3/rp6zjpXQsYsLdAUhGAtXWdU3PpWCDugSrIVgYs96LcgwFMR9sc7oErSgODyEFTVP8N+abZ1VEkaEZyeAgVyXYQD/QfGqSZ51QJ8LMJBVIJHQ2koVXMS87znOib/N8kD9tJxfLpJNgPwcKrF+r11pYpTDXJI4LwtLCJvpaMBtAO8tAE3wskZxaulmo1JzI/BewugnnOP3+a9TUDW8evucrZ1dOPsVYigmtWqQ2A/+gYUT0APIh8gIrsAP1/XbEGbe63NAAAAAElFTkSuQmCC)
![wechat](https://img.shields.io/badge/-troy15270658794-white.svg?style=social&logo=data:image/png;base64,iVBORw0KGgoAAAANSUhEUgAAABEAAAAOCAYAAADJ7fe0AAABSklEQVQ4T4WSIVLDUBCGv80U3WoM3IBGEAs9AEN7AorDMQwCHEUygwCHbE9AUUiqU2YSblAcjlYh6MxjdvMCCSRlTbJv933Z/88KeSS0WHKMo4vQBuZACkxocEtoeWWInca0EZ6AVk3fHEeHyKB/QkjYZEliAMcLYl/c8Z2vfpp9m6xBSMjsN0WIGSIcWEHo2dNx7xtPEGbfuWNERL8KkngP6iQvEIbAB87kvuF4KErTSVKErUqCygt4xHFeUVfDe2p4WU65cwHcAWf+WPNm6d2REhHmxqrreUOOUsCR92hEwNi80YtrdFjybrWAvewXP9PFme4i6KowRZ1f+jOuM4jGlDGw67MmjguEy/rbviKcFiEDGtyUNnNqO7GxEiSs/0CqOrNNnlT4lXU7DokYroZoo270JwOEbgGmmzxjO5P/P6ROS0xfp9DyF8keXaKcVCZ8AAAAAElFTkSuQmCC)
[![twitter](https://img.shields.io/twitter/follow/Arya_PPGYR.svg?label=Follow&style=social)](https://twitter.com/Arya_PPGYR)

## About

My learning process is recorded on [Wiki](https://github.com/Crabor/Quadcopter/wiki).
