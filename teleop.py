#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
import sys, select, termios, tty
import tkinter as tk
import time

banner = """
Reading from the keyboard and Publishing to AckermannDriveStamped!
---------------------------
移动控制:
        w
   a    s    d
按下任意键停止
CTRL-C退出

速度控制：
        i
   j    k    l

"""

print(banner)

keyBindings = {
    'w':(250,0),
    'd':(250,-250),
    'a':(250,250),
    's':(-250,0),
}

speedBindings = {
    'i':(10,0),
    'j':(0,-10),
    'l':(0,10),
    'k':(-10,0),
    'I':(0,0)
}

visualBindings = {
    '1':'0'
}

def getKey():
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    if key == chr(32):
        key = "go"
    return key

speed = 1.5
turn = 0.5

if __name__=="__main__":
    rospy.init_node('keyboard')
    settings = termios.tcgetattr(sys.stdin)
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
    rate = rospy.Rate(1)
    visual_select = rospy.Publisher('visual_stop', Bool, queue_size=1)
    x,th,tem_x,tem_z,print_flag1,print_flag2,print_flag3,z=0,0,0,0,0,0,0,0
    try:
        while(1):
            key = getKey()
            if key in keyBindings.keys():
                if print_flag1==0:
                    print("定速巡航")
                    x = keyBindings[key][0]
                    th = keyBindings[key][1]
                    twist = Twist()
                    twist.linear.x = x * speed;
                    twist.linear.y = 0;
                    twist.linear.z = 0
                    twist.angular.x = 0;
                    twist.angular.y = 0;
                    twist.angular.z = th * turn
                    print("线速度:{},角速度:{}".format(twist.linear.x,twist.angular.z))
                    pub.publish(twist)
                    print_flag1,print_flag2,print_flag3,z=1,0,0,0

            elif key in speedBindings.keys():
                if print_flag2==0:
                    print("加速")
                    tem_x += speedBindings[key][0]
                    tem_z += speedBindings[key][1]
                    print("当前速度： %.2f %.2f"%(tem_x/10,tem_z/10))
                    if z==1:
                        twist = Twist()
                        twist.linear.x = tem_x;
                        twist.linear.y = 0;
                        twist.linear.z = 0
                        twist.angular.x = 0;
                        twist.angular.y = 0;
                        twist.angular.z = tem_z
                        pub.publish(twist)
                        print_flag1,print_flag2,print_flag3=0,1,0

            elif key == "go":
                if print_flag3==0:
                    print("前进")
                    msg = Twist()
                    msg.linear.x = tem_x;
                    msg.linear.y = 0;
                    msg.linear.z = 0
                    msg.angular.x = 0;
                    msg.angular.y = 0;
                    msg.angular.z = tem_z
                    print("线速度：{},角速度：{}".format(msg.linear.x,msg.angular.z))
                    pub.publish(msg)
                    rate.sleep()
                    print_flag1,print_flag2,print_flag3,z=0,0,0,1

            elif key in visualBindings.keys():
                print("停止")
                visual_select.publish(0)
                print_flag1,print_flag2,print_flag3,z=0,0,0,1

            else:
                print("停止")
                x = 0
                th = 0
                twist = Twist()
                twist.linear.x = x * speed;
                twist.linear.y = 0;
                twist.linear.z = 0
                twist.angular.x = 0;
                twist.angular.y = 0;
                twist.angular.z = th * turn
                pub.publish(twist)
                rate.sleep()
                print_flag1,print_flag2,print_flag3,z=0,0,0,0
            if (key == '\x03'): break


    except:
        print("错误")

    finally:
        print("结束")
        twist = Twist()
        twist.linear.x = 0;
        twist.linear.y = 0;
        twist.linear.z = 0
        twist.angular.x = 0;
        twist.angular.y = 0;
        twist.angular.z = 0
        pub.publish(twist)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
