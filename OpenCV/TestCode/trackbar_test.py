import numpy as np
import cv2

img = np.zeros((512,512,3),np.uint8)        #创建黑框
cv2.namedWindow('image')  

draw = False
mode = True
ix,iy = -1,-1

def trackbar_callback():
    pass                    #此处的滑动条的回调函数可以不用做什么，pass掉就好

cv2.createTrackbar('R','image',0,255,trackbar_callback)     #创建滑动条
cv2.createTrackbar('B','image',0,255,trackbar_callback)
cv2.createTrackbar('G','image',0,255,trackbar_callback)

def mouse_event_callback(event,x,y,flags,param):
    #global ix,iy,draw,mode
    global draw,mode
    r = cv2.getTrackbarPos('R','image')                        #获取键值
    b = cv2.getTrackbarPos('B','image')
    g = cv2.getTrackbarPos('G','image')

    if event == cv2.EVENT_LBUTTONDOWN:
        draw = True
        #ix,iy = x,y
    elif event == cv2.EVENT_MOUSEMOVE and draw:
        if mode == True:
            cv2.circle(img,(x,y),10,(b,g,r),-1,cv2.LINE_AA)
        elif mode == False:
            cv2.rectangle(img,(x-5,y-5),(x+5,y+5),(b,g,r),-1,cv2.LINE_AA)    
    elif event == cv2.EVENT_LBUTTONUP:
        draw = False
    
cv2.setMouseCallback('image',mouse_event_callback)
while(1):
    cv2.imshow('image',img)
    key = cv2.waitKey(1)
    if key == 27:
        break
    elif key == ord('q'):
        mode =not mode
    
cv2.destroyAllWindows()    