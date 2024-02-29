'''
from ssc202
'''
import cv2
import numpy as np

img = cv2.imread('/home/zyt/图片/OIP-C.jpeg',cv2.IMREAD_COLOR)

# 结构元素定义
kernel = np.ones((9,9),np.uint8)
#kernel = cv2.getStructuringElement(cv2.MORPH_CROSS,(5,5))   #交叉形

while True:
    #res = cv2.filter2D(img, -1, kernel)            #滤波
    #res = cv2.erode(img,kernel,iterations=10)      #腐蚀
    #res = cv2.dilate(img,kernel,iterations=5)      #膨胀
    #res = cv2.morphologyEx(img,cv2.MORPH_OPEN,kernel)  #开运算
    #res = cv2.morphologyEx(img,cv2.MORPH_CLOSE,kernel)  #闭运算
    res = cv2.morphologyEx(img,cv2.MORPH_GRADIENT,kernel) #形态学梯度

    cv2.imshow('res',res)
    cv2.imshow('img',img)
    key = cv2.waitKey(1)
    if key == 27:
        break
    
cv2.destroyAllWindows()
        