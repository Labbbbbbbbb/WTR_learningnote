'''
实时打印某个点的BGR值
'''

import cv2
import numpy as np

cap = cv2.VideoCapture(0)
#cap.set(10,1)

while cap.isOpened() == 1:
    ret,frame = cap.read()
    if ret:
        px = frame[100,100]
        cv2.imshow('img',frame)
        print(px)
        key = cv2.waitKey(1)
        if key == 27:
            break
    else:
        break


cap.release()                                      #释放对象
cv2.destroyAllWindows()                            #关闭窗口