#!/usr/bin/env python
# -*- coding: utf-8 -*-
import time
import cv2
import numpy as np
from rm_msg.msg import dir
import rospy

x=[]
y=[]
i,direction1 = 0,0
if __name__ == "__main__":
                rospy.init_node('visual')   #初始化节点
                pub = rospy.Publisher('error', dir, queue_size=1)
                center = 60   #中心范围
                black_lower = np.array([0,0,0])
                black_upper = np.array ([180,255,46])   #二值化
                cap = cv2.VideoCapture(0)
                ret = cap.set(3,320)
                ret = cap.set(4,240)    #腐化
                print(cap.isOpened())
                while not rospy.is_shutdown():
                    ret, frame = cap.read()
                    #cv2.imshow('input_image',frame)
                    frame = cv2.GaussianBlur(frame, (5, 5), 0)   #高斯模糊
                    #cv2.imshow('Gauss',frame)
                    #cv2.imshow('GaussianBlur',frame)
                    black = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)  #BGR和灰度图的转换
                    retval, mask = cv2.threshold(black, 100, 255, cv2.THRESH_BINARY)   #二值化
                    mask = cv2.erode(mask,None,iterations=6) 
                    #cv2.imshow('result',mask)
                    color = mask[160]
                    white_count = np.sum(color == 0)
                    print(white_count)
                    white_index = np.where(color == 0)
                    #image process
                    if white_count == 0:
                        print('未识别')
                        white_count=1

                    else:
                        center = (white_index[0][white_count - 1] + white_index[0][0]) // 2
                        print(center)
                        cv2.circle(frame, (center,160), 5, (255, 0, 255), 2)
                        cv2.imshow('result1',frame)
                        direction = center - 160
                        if i<=2:
                            x.append(direction)
                            directiom1 = sum(x)/float(len(x))
                           #print("<3")
                            i+=1
                        else:
                            x.pop(0)
                            x.append(direction)
                            #print(x)
                            direction1 = sum(x)/3.0
                            #print(direction1)
                            #print(">3")

                    pub.publish(direction1)
                    rospy.sleep(0.5)
                    c = cv2.waitKey(1)
                    if c == ord('q'):
                          break
                    
cap.release()
cv2.destroyAllWindows()

