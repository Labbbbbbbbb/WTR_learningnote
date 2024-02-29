'''
from ssc202
'''

import cv2
import numpy as np

img = cv2.imread('/media/zyt/Data/WTR_workplace/OpenCV/image/OpenCV听课笔记/1708953301349.png')
shape = cv2.imread('/media/zyt/Data/WTR_workplace/OpenCV/image/OpenCV听课笔记/1708953685082.png')

img1 = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
ret, img1 = cv2.threshold(img1, 127, 255, cv2.THRESH_BINARY)
shape1 = cv2.cvtColor(shape, cv2.COLOR_BGR2GRAY)
ret, shape1 = cv2.threshold(shape1, 127, 255, cv2.THRESH_BINARY)

counter_img, hierarchy_img = cv2.findContours(
    img1, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
counter_shape, hierarchy_shape = cv2.findContours(
    shape1, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)

n = len(counter_img)
k = []
# 逐个进行模板匹配  会将img中从0到len的轮廓全部与shape的第一个轮廓进行比较
for i in range(n):
    temp = cv2.matchShapes(counter_img[i], counter_shape[0], 1, 0.0)    #函数返回相似度，相似度==0为完全相似
    print(temp)
    if temp < 2:
        k.append(counter_img[i])
    
cv2.drawContours(img, k, -1, (0, 255, 0), 2)        #将所有匹配成功了的轮廓全部画到原图像上
#cv2.drawContours(shape,counter_shape,0, (0, 255, 0), 2)  #用来看匹配标准，，，

while True:
    cv2.imshow('shape', shape)
    cv2.imshow('res', img)
    key = cv2.waitKey(1)
    if key == 27:
        break

cv2.destroyAllWindows()