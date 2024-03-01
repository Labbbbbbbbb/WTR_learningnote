import cv2
import numpy as np

img = cv2.imread('/home/zyt/图片/OIP-C.jpeg')
gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
ret, binary = cv2.threshold(gray, 127, 255, cv2.THRESH_BINARY)

contours, hierarchy = cv2.findContours(
    binary, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)



for i in contours:
    (x, y), radius = cv2.minEnclosingCircle(i)
    # 必须化为整型(!)
    cv2.circle(img, (int(x), int(y)), int(radius), (0, 255, 0), 1)
cv2.imshow('res', img)
key = cv2.waitKey(0)
if key == 27:
    cv2.destroyAllWindows()