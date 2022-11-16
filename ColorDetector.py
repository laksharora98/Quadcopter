#Needs to be highly optimised


import numpy as np
import cv2

class ColorDetector:


    def method_detect(self):
        
        cap=cv2.VideoCapture(0)

        get,frame=cap.read()
        #frame = cv2.imread('C:\\Users\HP\Desktop\download.png',cv2.IMREAD_COLOR)
        hsv=cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)
        red_lowerl=np.array([0,100,100])
        red_upperl=np.array([10,255,255])
        red_loweru=np.array([160,100,100])
        red_upperu=np.array([170,255,255])
        blue_lower=np.array([90,100,100])
        blue_upper=np.array([135,255,255])
        green_lower=np.array([40,100,100])
        green_upper=np.array([75,255,255])
        black_lower=np.array([0,0,0])
        black_upper=np.array([180,255,15])
        #findingrange
        red=cv2.inRange(hsv,red_lowerl,red_upperl) + cv2.inRange(hsv, red_loweru, red_upperu)
        blue=cv2.inRange(hsv,blue_lower,blue_upper)
        green=cv2.inRange(hsv,green_lower,green_upper)
        black=cv2.inRange(hsv,black_lower,black_upper)
        #morph
        kernal=np.ones((5,5),'uint8')
    
        red=cv2.dilate(red,kernal)
        res=cv2.bitwise_and(frame,frame,mask=red)

        blue=cv2.dilate(blue,kernal)
        res=cv2.bitwise_and(frame,frame,mask=blue)

        green=cv2.dilate(green,kernal)
        res=cv2.bitwise_and(frame,frame,mask=green)

        black=cv2.dilate(black,kernal)
        res=cv2.bitwise_and(frame,frame,mask=black)

        #tracking
        (_,contours,hierarchy)=cv2.findContours(red,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
        for pic,contour in enumerate(contours):
            area=cv2.contourArea(contour)
            if(area>2000):
                x,y,w,h=cv2.boundingRect(contour)
                frame=cv2.rectangle(frame,(x,y),(x+w,y+h),(255,255,255),2)
                frame=cv2.rectangle(frame,(int(x+w/2),int(y+h/2)),(int(x+w/2),int(y+h/2)),(255,255,255),2)
                Red=[int(x+w/2),int(y+h/2)]
            else:
                Red=[0,0]
                

        (_,contours,hierarchy)=cv2.findContours(blue,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
        for pic,contour in enumerate(contours):
            area=cv2.contourArea(contour)
            if(area>2000):
                x,y,w,h=cv2.boundingRect(contour)
                frame=cv2.rectangle(frame,(x,y),(x+w,y+h),(255,255,255),2)
                frame=cv2.rectangle(frame,(int(x+w/2),int(y+h/2)),(int(x+w/2),int(y+h/2)),(255,255,255),2)
                Blue=[int(x+w/2),int(y+h/2)]
            else:
                Blue=[0,0]

    
        (_,contours,hierarchy)=cv2.findContours(green,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
        for pic,contour in enumerate(contours):
            area=cv2.contourArea(contour)
            if(area>2000):
                x,y,w,h=cv2.boundingRect(contour)
                frame=cv2.rectangle(frame,(x,y),(x+w,y+h),(255,255,255),2)
                frame=cv2.rectangle(frame,(int(x+w/2),int(y+h/2)),(int(x+w/2),int(y+h/2)),(255,255,255),2)
                Green=[int(x+w/2),int(y+h/2)]
            else:
                Green=[0,0]

        (_,contours,hierarchy)=cv2.findContours(black,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
        for pic,contour in enumerate(contours):
            area=cv2.contourArea(contour)
            if(area>2000):
                x,y,w,h=cv2.boundingRect(contour)
                frame=cv2.rectangle(frame,(x,y),(x+w,y+h),(255,255,255),2)
                frame=cv2.rectangle(frame,(int(x+w/2),int(y+h/2)),(int(x+w/2),int(y+h/2)),(255,255,255),2)
                Black=[int(x+w/2),int(y+h/2)]
            else:
                Black=[0,0]

                    
        cv2.imshow("Color Tracking",frame)
        if cv2.waitKey(10) & 0xFF==ord('q'):
            cap.release()
            cv2.destroyAllWindows()
        
        Req = [Red[0],Red[1],Blue[0],Blue[1],Green[0],Green[1],Black[0],Black[1]]
        return Req
                
        

