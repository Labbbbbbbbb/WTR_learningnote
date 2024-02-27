'''
该程序只起显示视频图像的功能，不负责保存
'''

import cv2
import numpy as np

cap = cv2.VideoCapture(0)       #创建对象，索引号为0

while cap.isOpened() == False:
    cap.open()

while(True):
    ret,frame = cap.read()
    gray = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)  #转换颜色,返回值为一种格式
    cv2.imshow('frame',gray)
    if cv2.waitKey(1) & 0xFF == ord('q'):          #等待1s（还是ms？）若按q则退出
        break
    
cap.release()                                      #释放对象
cv2.destroyAllWindows()                            #关闭窗口

