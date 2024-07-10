import hid
import time
import cv2
import numpy as np
import threading

dataList=np.empty((0,2))
centersList=np.zeros((4,2))

PIXEL_TO_CMETER=100

pointer=0

calculatable=0

dist=0

total_dist=0.0

cv2.namedWindow("img",cv2.WINDOW_NORMAL)
cv2.setWindowProperty("img",cv2.WND_PROP_FULLSCREEN,cv2.WINDOW_FULLSCREEN)

def collectXYData(device):
    global pointer
    global dataList
    while True:
        mouseData=device.read(64)
        
        pointer=mouseData[0]
        
        dx,dy=0,0

        
        if mouseData[-3]==255:
            dx=mouseData[-4]-256
        elif mouseData[-3]==0:
            dx=mouseData[-4]
        
        if mouseData[-1]==255:
            dy=mouseData[-2]-256
        elif mouseData[-1]==0:
            dy=mouseData[-2]
            
        dataList=np.append(dataList,[[dx,dy]],axis=0)
        
        time.sleep(0)
        
def calculateDisplacement():
    global centersList
    global center
    global dist
    global pointer
    global img
    global distx
    global disty
    global total_dist
        
    while True:        
        if calculatable:
            centersList[0]=center
            
            time.sleep(0.01)
            
            if not calculatable:
                while not calculatable:
                    centersList[1]=center
            else:
                centersList[1]=center
            
            distx=centersList[1,1]-centersList[0,1]
            disty=centersList[1,0]-centersList[0,0]
            
            dist=(distx**2 + disty**2)**0.5
            total_dist+=dist
            
            if total_dist/PIXEL_TO_CMETER >=5.0:
                cv2.line(img, (round(centersList[2][1]),round(centersList[2][0])),(round(centersList[3][1]),round(centersList[3][0])), (255,0,0), 1) #y değişim
                centersList[3]=center
                total_dist=0
            else:
                centersList[2]=center
            
            cv2.line(img, (round(centersList[1][1]),round(centersList[0][0])),(round(centersList[1][1]),round(centersList[1][0])), (0,255,0), 1) #y değişim
            cv2.line(img, (round(centersList[0][1]),round(centersList[0][0])),(round(centersList[1][1]),round(centersList[0][0])), (0,0,255), 1) #x değişim
            
            cv2.line(img, (round(centersList[0][1]),round(centersList[0][0])), (round(centersList[1][1]),round(centersList[1][0])), (255,255,255), 1)
        
        

    
    
    

imgSize=[1080,1920,3]

img=np.zeros(imgSize,dtype=np.uint8)

center=[imgSize[0]//2,imgSize[1]//2]

centersList[3]=center

devices=hid.enumerate()

isDeviceFounded=0

img[center[0],center[1]]=[255,255,255]


for device in devices:
    if device['product_string'] == 'Gaming Mouse':
        isDeviceFounded=1
        break
    else:
        isDeviceFounded=0
    
cv2.imshow("img",img)

if isDeviceFounded:
    vendor_id,product_id=device['vendor_id'],device['product_id']
    

    device=hid.Device(vendor_id,product_id)    
    
    dataThread = threading.Thread(target=collectXYData,args=(device,))
    dataThread.start()
    
    calculateThread=threading.Thread(target=calculateDisplacement)
    calculateThread.start()
    
    while True:
        
        if calculatable:
            img[1025:imgSize[1],:]=0
        
        if dataList.shape[0]!=0:
        
            tempCentersList=np.copy(dataList)

            for x,y in tempCentersList:
                
                calculatable=0
                
                center[1]+=x
                center[0]+=y

                if center[1]>=imgSize[1]:
                    center[1]=imgSize[1]-1
                elif center[1]<0.0:
                    center[1]=0

                if center[0]>=imgSize[0]:
                    center[0]=imgSize[0]-1
                elif center[0]<0.0:
                    center[0]=0
                
                calculatable=1

                #img[round(center[0]),round(center[1])]=255

            dataList=dataList[tempCentersList.shape[0]:]
            
        if pointer==1:
            img=np.zeros(imgSize,dtype=np.uint8)
            center=[imgSize[0]/2,imgSize[1]/2]
            centersList[3]=center
        if dist!=0:
            cv2.putText(img, f"{format(dist,'.2f')}pixels ,{format(dist/PIXEL_TO_CMETER,'.2f')}cm (total) , {format(distx/PIXEL_TO_CMETER,'.2f')}cm (x) {format(-disty/PIXEL_TO_CMETER,'.2f')}cm (y)", (0,imgSize[0]), cv2.FONT_HERSHEY_SIMPLEX ,  2, (255,255,255), 2, cv2.LINE_AA) 

        cv2.imshow("img",img)
        
        if cv2.waitKey(1)==ord('q'):
            break
else:
    print("Device not found")
            
