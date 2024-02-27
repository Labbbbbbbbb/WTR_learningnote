import numpy as np 
import cv2

def draw_rectangle(event,x,y,flags,param): #后面俩没什么用，但是要写
    if event == cv2.EVENT_LBUTTONDBLCLK:   #双击左键
        cv2.rectangle(img,(x,y),(x+30,y+20),(255,255,0),5)



###在空图片上
img = np.zeros((512,512,3),np.uint8)
cv2.namedWindow('img')                      #创建空窗口
cv2.setMouseCallback('img',draw_rectangle)
#注意上面这两句的顺序不能调换，一定要先创建窗口，再将回调函数绑定到这个窗口上

while(1):
    cv2.imshow('img',img)
    if cv2.waitKey(1)&0xFF==27:
        break

cv2.destroyAllWindows()


'''
###在视频中
###待修改，因为在视频中draw会因时间过短被覆盖，可以用add叠加
def draw_rectangle(event,x,y,flags,param): #后面俩没什么用，但是要写
    if event == cv2.EVENT_LBUTTONDBLCLK:   #双击左键
        cv2.rectangle(frame,(x,y),(x+30,y+20),(255,255,0),5)


cap = cv2.VideoCapture(0)
cv2.namedWindow('frame') 
cv2.setMouseCallback('frame',draw_rectangle)    


while cap.isOpened == False:
    cap.open()

while(True):
    ret,frame = cap.read()
    if ret==True:
        cv2.imshow('frame',frame)
        if cv2.waitKey(1) & 0xFF == 27:
            break   



cap.release()  
cv2.destroyAllWindows()   
'''