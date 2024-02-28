'''
在视频中加入鼠标事件！

'''


import numpy as np 
import cv2

def draw_rectangle(event,x,y,flags,param): 
    if event == cv2.EVENT_LBUTTONDBLCLK:   
        cv2.rectangle(img,(x,y),(x+30,y+20),(255,255,0),5)


img = np.zeros((1080,1920,3),np.uint8)        #创建黑框
#cv2.namedWindow('image')
cap = cv2.VideoCapture(0)
cv2.namedWindow('addFI') 
cv2.setMouseCallback('addFI',draw_rectangle)    



while cap.isOpened == False:
    cap.open()

while(True):
    ret,frame = cap.read()
    if ret==True:
        addFI = cv2.add(frame,img)
        cv2.imshow('addFI',addFI)
        print(frame.shape)
        if cv2.waitKey(1) & 0xFF == 27:
            break   



cap.release()  
cv2.destroyAllWindows()   
