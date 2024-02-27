import cv2
import numpy as np

cap = cv2.VideoCapture(0)
cap.set(10,1)

while cap.isOpened() == True:
    ret,frame = cap.read()
    if ret == True:
		# 索引方式获取ROI
        roi = frame[100:200,100:200]
        cv2.imshow('img',roi)
        key = cv2.waitKey(1)
        if key == 27:
            break
    else:
        break
    
cv2.destroyAllWindows()
cap.release()