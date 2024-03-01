import cv2
import numpy as np

cap = cv2.VideoCapture(0)      

while cap.isOpened() == False:
    cap.open()

while(1):
    ret,frame = cap.read()
    img = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY) 
    retval, res = cv2.threshold(img, 127, 255, cv2.THRESH_BINARY)   #先转化为灰度图像以及二值化图像
    contours, hierarchy = cv2.findContours(res,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
    cv2.drawContours(frame, contours,-1,(255,255,0),3)              #将轮廓叠加在原图像上
    cv2.imshow('frame',frame)
    print(hierarchy)
    key = cv2.waitKey(1)

    if key == 27:
        break

cap.release()
cv2.destroyAllWindows()