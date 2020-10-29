# Omega-MArgo
NEUQRMSudoI 2020 招新考核项目

在树莓派4B平台上，基于ROS+Ubuntu实现小车的可控运动，自动巡线等功能

实验环境：Ubuntu 20.04 MATE，ROS Noetic，OpenCV 3.4.12

为了实现控制过程的可视化我们还使用了Tkinter

感谢NEUQRMSudoI的学长为我们提供了核心代码。

注意：

我们在树莓派4B平台上需要通过在root账户下运行control.py，否者会出现“Not running on a RPi!”报错，为解决这个问题，源代码我们使用BCM定义引脚。


代码文件：

control.py 负责小车的控制
teleop.py 用于读取键盘的输入传达控制小车的指令
linefollow.py 基于openCV实现小车自动循迹功能

同时我们使用了一个自已定msg：dir.msg，关于如何在ROS中使用自定义msg请参见：https://blog.csdn.net/u013453604/article/details/72903398

更多详细内容以及我们在此过程中遇到的问题以及解决方案请参见目录下的report.pdf
