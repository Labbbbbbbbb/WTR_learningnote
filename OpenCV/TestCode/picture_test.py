import numpy as np
import cv2

img = cv2.imread('/home/zyt/图片/OIP-C.jpeg', 1)  #读入图像，灰白
cv2.namedWindow('img', cv2.WINDOW_AUTOSIZE)      # 加载窗口，可指定窗口大小
cv2.imshow('img', img)                           #显示图像
key=cv2.waitKey(0)                                   #无限等待

if key==27:                             #按下esc键
    cv2.destroyAllWindows()
elif key==ord('s'):                       #按下s键
    cv2.imwrite('save.jpeg',img)        #保存图片（？
    cv2.destroyAllWindows()