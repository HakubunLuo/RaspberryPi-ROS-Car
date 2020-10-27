# Omega-MArgo
NEUQRMSudoI 2020 招新考核项目

在树莓派4B平台上，基于ROS+Ubuntu实现小车的可控运动，自动巡线等功能

实验环境：Ubuntu 20.04 MATE，ROS Noetic，OpenCV 3.4.12

为了实现控制过程的可视化我们还使用了Tkinter


代码文件：

control.py 负责小车的控制

key.py 用于读取键盘的输入传达控制小车的指令

linefollow.py 基于openCV实现小车自动循迹功能

同时我们使用了一个自已定msg：dir.msg
