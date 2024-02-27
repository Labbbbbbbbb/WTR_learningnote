import numpy as np
import cv2

img=cv2.imread('/home/zyt/图片/OIP-C.jpeg',1)
cv2.line(img,(1,1),(640,260),(255 ,0,0),3)      #绘制函数一定要在imshow之前 不然会被覆盖
cv2.circle(img,(470,260),10,(0,255,0),-1)
cv2.putText(img,"opencv",(10,130),cv2.FONT_HERSHEY_SIMPLEX,4,(255,255,0),1,cv2.LINE_AA)
cv2.imshow('capture',img)


key=cv2.waitKey(0)                                   #无限等待

if key==27:                             #按下esc键
    cv2.destroyAllWindows()
