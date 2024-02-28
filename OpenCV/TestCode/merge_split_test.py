'''
from ssc202
'''

import cv2
import numpy as np

cap = cv2.VideoCapture(0)
cap.set(10, 1)

while cap.isOpened() == True:
    ret, frame = cap.read()
    if ret == True:
        # 通道分离
        b, g, r = cv2.split(frame)     
        print(b)
        #通道合并
        img = cv2.merge([b, g, r])
        cv2.imshow('img',img)
        '''
        #通道合并和分离耗时比较大，能使用索引操作，就使用索引操作。
        frame[:,:,1] = 0        
        #`frame[:,:,1] = 0` 这个表达式是针对 NumPy 数组的操作，而在 OpenCV 中，图像通常以 NumPy 数组的形式表示。

        #在这个表达式中：
        #- `frame` 是一个图像，是一个三维的 NumPy 数组，通常具有形状 (height, width, channels)。
        #- `[:,:,1]` 表示对数组的第二个维度（即通道维度）的所有元素进行操作，而 `1` 则表示操作的是图像的第二个通道（通道索引从0开始）。
        #- `= 0` 表示将选定的通道的所有元素的值设置为0。

        #因此，`frame[:,:,1] = 0` 的作用是将图像中所有像素的蓝色通道的值设置为0。这通常用于在图像处理中修改图像的特定通道的值，可能是为了去除或减弱该通道的影响，或者进行一些特定的通道操作。
        
        '''
        cv2.imshow('img',frame)
        key = cv2.waitKey(1)
        if key == 27:
            break
    else:
        break

cv2.destroyAllWindows()    
cap.release()