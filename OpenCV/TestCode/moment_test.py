'''
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
    cnt = contours[0]
    M = cv2.moments(cnt)
    px = int(M['m10']/M['m00'])         #为什么在视频里就会出现除0错误？
    py = int(M['m01']/M['m00'])

    cv2.circle(frame,(px,py),5,(0,255,0),-1)
    cv2.imshow('frame',frame)

    key = cv2.waitKey(1)
    if key == 27:
        break

cap.release()
cv2.destroyAllWindows()

'''

import cv2
import numpy as np

img = cv2.imread('/home/zyt/图片/OIP-C.jpeg')
gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
ret, binary = cv2.threshold(gray, 127, 255, cv2.THRESH_BINARY)

# 寻找轮廓
counter, hierarchy = cv2.findContours(
    binary, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
# 求取质心
cnt = counter[474]   #本图最大索引至474,可以用len(contours)查看
M = cv2.moments(cnt)
cx = int(M['m10']/M['m00'])
cy = int(M['m01']/M['m00'])

while True:
    cv2.circle(img, (cx, cy), 5, (0, 255, 0), -1)
    cv2.imshow('res', img)
    key = cv2.waitKey(1)
    if key == 27:
        break

cv2.destroyAllWindows()
