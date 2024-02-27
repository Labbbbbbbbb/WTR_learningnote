'''
from ssc202
'''

import cv2
import numpy as np

cap = cv2.VideoCapture(0)
cap.set(10,1)

while cap.isOpened() == True:
    ret,frame = cap.read()
    if ret == True:
        # 行数，列数，通道数（元组）
        attr = frame.shape
        print(attr)
        # 图像数据类型
        dtype = frame.dtype
        print(dtype)
        cv2.imshow('img',frame)
        key = cv2.waitKey(1)
        if key == 27:
            break
    else:
        break

cv2.destroyAllWindows()
cap.release()