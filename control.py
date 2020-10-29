#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import RPi.GPIO as gpio
import time
import rospy
from geometry_msgs.msg import Twist
from rm_msg.msg import dir
from std_msgs.msg import Bool
import numpy as np
from std_msgs.msg import Float64
gpio.setmode(gpio.BCM)
#gpio.setmode(gpio.BOARD)

counter1=0
counter2=0
wheel1_actual_rpm=0
wheel2_actual_rpm=0
wheel2_target_rpm=0
wheel1_target_rpm=0
ak=0.9#转弯系数
visual_start=1
rpm=240#电机转速
c=26.69#pi=3.14 d=8.5


class gpio_control(object):
        #以下引脚是在BCM模式下定义的
        pin1 = 12   #PWM
        pin2 = 13   #PWM
        #编码器部分
        #incoder1 = 16
        #incoder2 = 18
        #逻辑控制
        IN1 = 17
        IN2 = 27
        IN3 = 22
        IN4 = 23

        def __init__(self):
                print("正在初始化……")
                rospy.Subscriber("cmd_vel",Twist,self.keyboard_callback)
                rospy.Subscriber("error",dir,self.error_callback)
                rospy.Subscriber("visual_stop",Bool,self.visual_select_callback)
                rospy.Timer(rospy.Duration(1),self.speed_cal)  #10ms
                gpio.setup(gpio_control.pin1,gpio.OUT)
                gpio.setup(gpio_control.pin2,gpio.OUT)
                gpio.setup(gpio_control.IN1,gpio.OUT)
                gpio.setup(gpio_control.IN2,gpio.OUT)
                gpio.setup(gpio_control.IN3,gpio.OUT)
                gpio.setup(gpio_control.IN4,gpio.OUT)
                #gpio.setup(gpio_control.incoder1,gpio.IN,pull_up_down=gpio.PUD_UP)
                #gpio.setup(gpio_control.incoder2,gpio.IN,pull_up_down=gpio.PUD_UP)
                #gpio.add_event_detect(gpio_control.incoder1,gpio.RISING,callback=self.speed_detect_callback1)
                #gpio.add_event_detect(gpio_control.incoder2,gpio.RISING,callback=self.speed_detect_callback2)

                #PWM initiation
                self.pwm1 = gpio.PWM(gpio_control.pin1,600)
                self.pwm2 = gpio.PWM(gpio_control.pin2,600)
                self.pwm1.start(0)  #占空比
                self.pwm2.start(0)
                print("初始化完成")

        def keyboard_callback(self,msg):#键盘控制
                print("获取键盘输入……")
                global wheel1_actual_rpm, wheel2_actual_rpm, wheel1_target_rpm, wheel2_target_rpm, ak
                linear_x = msg.linear.x
                print("线速度  %.2f",linear_x)
                angular_z = msg.angular.z#差速转弯
                wheel1_linear = linear_x + ak * angular_z
                wheel2_linear = linear_x - ak * angular_z
                wheel1_target_rpm = wheel1_linear
                wheel2_target_rpm = wheel2_linear

                #小于0代表反方向移动
                if wheel1_target_rpm>0:
                     gpio.output(gpio_control.IN1,1)
                     gpio.output(gpio_control.IN2,0)
                elif wheel1_target_rpm==0:
                     gpio.output(gpio_control.IN1,0)
                     gpio.output(gpio_control.IN2,0)
                elif wheel1_target_rpm<0:
                     gpio.output(gpio_control.IN1,0)
                     gpio.output(gpio_control.IN2,1)
                     wheel1_target_rpm = -wheel1_target_rpm
                    
                if wheel2_target_rpm>0:
                     gpio.output(gpio_control.IN3,1)
                     gpio.output(gpio_control.IN4,0)
                elif wheel2_target_rpm==0:
                     gpio.output(gpio_control.IN3,0)
                     gpio.output(gpio_control.IN4,0)
                elif wheel2_target_rpm<0:
                     gpio.output(gpio_control.IN3,0)
                     gpio.output(gpio_control.IN4,1)
                     wheel2_target_rpm = -wheel2_target_rpm

        def error_callback(self, msg):
                global wheel1_target_rpm, wheel2_target_rpm, ak
                if visual_start == 1:
                    print(msg.dir)
                    #x_speed=0.5*(100-0.4*abs(msg.dir))  #-180              
                    #z_speed = -0.85*msg.dir
                    linear_x =3.5 * (100 - 0.65 * abs(msg.dir))  #-180
                    angular_z = -0.4 * msg.dir
                    wheel1_linear = linear_x+angular_z  
                    #wheel2_linear = linear_x-1.0/2*angular_z*2
                    wheel2_linear = linear_x-angular_z
                    wheel1_target_rpm = wheel1_linear
                    wheel2_target_rpm = wheel2_linear
                    print("电机1目标转速：",'%.2f' % wheel1_target_rpm,"电机2目标转速：",'%.2f' % wheel2_target_rpm)

                    if wheel1_target_rpm>0:
                       gpio.output(gpio_control.IN1,1)
                       gpio.output(gpio_control.IN2,0)
                    elif wheel1_target_rpm==0:
                       gpio.output(gpio_control.IN1,0)
                       gpio.output(gpio_control.IN2,0)
                    elif wheel1_target_rpm<0:
                       gpio.output(gpio_control.IN1,0)
                       gpio.output(gpio_control.IN2,1)
                       wheel1_target_rpm = -wheel1_target_rpm

                    if wheel2_target_rpm>0:
                       gpio.output(gpio_control.IN3,1)
                       gpio.output(gpio_control.IN4,0)
                    elif wheel2_target_rpm==0:
                       gpio.output(gpio_control.IN3,0)
                       gpio.output(gpio_control.IN4,0)
                    elif wheel2_target_rpm<0:
                       gpio.output(gpio_control.IN3,0)
                       gpio.output(gpio_control.IN4,1)
                       wheel2_target_rpm = -wheel2_target_rpm

        def visual_select_callback(self,msg):
                global visual_start
                print("__close__camera__")
                visual_start=0

        #编码器部分
        '''def speed_detect_callback1(self,channel1):
                global counter1
                if gpio.event_detected(gpio_control.incoder1):
                        counter1=counter1+1
        def speed_detect_callback2(self,channel1):
                global counter2
                if gpio.event_detected(gpio_control.incoder2):
                        counter2=counter2+1'''
                        
        def pid_realize(self,target_speed,actual_speed):
                #print("target",target_speed)      
                kp=0.08
                ki=0
                kd=0.01
                error[0]=error[1]
                error[1]=error[2]
                error[2]=actual_speed-target_speed
                adjust[0]=adjust[1]
                adjust[1]=adjust[2]
                adjust[2]=adjust[1]+kp*(error[2]-error[1])+ki*error[2]+kd*(error[2]-2*error[1]+error[0])
                #print("adjust",adjust[2])
                return adjust[2]

        def speed_cal(self,er):
                global counter1,counter2,wheel1_actual_rpm,wheel2_actual_rpm
                wheel1_actual_rpm=counter1/11/0.1
                wheel2_actual_rpm=counter2/11/0.1
                #print("wheel1_actual_rpm: ",wheel1_actual_rpm,"wheel2_actual_rpm: ",wheel2_actual_rpm)
                #实现速度读取需要编码器
                counter1=0
                counter2=0

        def loop(self):
             global wheel1_target_rpm, wheel2_target_rpm
             while not rospy.is_shutdown():
                 #dc1=abs(self.pid_realize(wheel1_target_rpm,wheel1_actual_rpm))   
                 dc1=wheel1_target_rpm/1000*100
                 if dc1 > 100:
                     dc1=100
                 #dc2=abs(self.pid_realize(wheel2_target_rpm,wheel2_actual_rpm))
                 dc2=wheel2_target_rpm/1000*100
                 if dc2 > 100:
                     dc2=100
                 rpm1 = rpm * dc1 / 100
                 rpm2 = rpm * dc2 / 100
                 v1 = rpm1 * c
                 v2 = rpm2 * c
                 print(("PWM1调速比： %.2f  PWM2 is %.2f") % (dc1,dc2))
                 print(("电机1转速： %.2f  电机2转速： %.2f") % (rpm1, rpm2))
                 print(("车轮1速度： %.2f  cm/s  车轮2速度： %.2f  cm/s") % (v1, v2))
                 self.pwm1.ChangeDutyCycle(dc1)
                 self.pwm2.ChangeDutyCycle(dc2)
                 rospy.sleep(0.5)   #设定消息返回频率


if __name__=="__main__":
        rospy.init_node("control",anonymous=True)
        try:
            init = gpio_control()
            init.loop()
            rospy.spin()
        except Exception as e:
            print(e)
        finally:
            init.pwm1.stop()
            init.pwm2.stop()
            gpio.cleanup()
            print("end")





































