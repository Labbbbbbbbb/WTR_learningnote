import cv2 
import numpy as np

cap = cv2.VideoCapture(0)
while cap.isOpened == 0:
    cap.Open()



while 1 :
    ret,frame = cap.read()
    if ret == True:
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        retval, res = cv2.threshold(gray, 127, 255, cv2.THRESH_BINARY)
        contours, hierarchy = cv2.findContours(res,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
        k = []                                          #k一定要在循环之内定义，不然轮廓会无限累加
        for i in contours:
            approx = cv2.approxPolyDP(i,10,True)        #将轮廓转换为多边形拟合之后的结果
            k.append(approx)                            #将新的轮廓添加进数组中
        cv2.drawContours(frame,k,-1,(255,255,0),3)
        cv2.imshow('frame',frame)

        if cv2.waitKey(1) & 0xFF == 27 :
            break
    
cap.release()
cv2.desroyAllWindows()

'''
#一个比较科学的确定拟合精度的方法：

epsilon = 0.001 * cv2.arcLength(contours[i])
approx = cv2.approxPolyDP(contours[i],epsilon,True)

'''