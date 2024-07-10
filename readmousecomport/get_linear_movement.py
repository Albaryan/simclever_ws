import hid
import time
import cv2
import numpy as np
import threading
import math

dataList=np.empty((0,2))
tempDataList=np.empty((0,2))
centersList=np.zeros((4,2))

PIXEL_TO_CMETER=1100

pointer=0

calculatable=0

dist=0

total_dist=0.0

slope=0

cv2.namedWindow("img",cv2.WINDOW_NORMAL)

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
        
    

imgSize=[1080,1920,3]

img=np.zeros(imgSize,dtype=np.uint8)

center=[imgSize[0]//2,imgSize[1]//2]

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
    
    cTime=time.time()
    
    disX,disY=0,0
    
    slopeOld=0.0
    
    totalDisX=0
    totalDisY=0
    
    totalCmeter=0
    
    totalDis=0
    
    while True:
        
        if dataList.shape[0]>10:
            tempDataList=np.copy(dataList)
            disX=np.sum(tempDataList[:,0],axis=0)
            disY=np.sum(tempDataList[:,1],axis=0)
            
            slopeNew=math.atan2(disY,disX)
            dataList=dataList[tempDataList.shape[0]:]
                
            if slopeNew==slopeOld:
                totalDisX+=disX
                totalDisY+=disY   
                cv2.circle(img,(round(center[1]),round(center[0])),2,(255,255,255),-1)
                cv2.line(img,(round(center[1]),round(center[0])),(round(center[1]+totalDisX),round(center[0]+totalDisY)),(255,255,255),1,cv2.LINE_AA)
            else:
                cv2.line(img,(round(center[1]),round(center[0])),(round(center[1]+totalDisX),round(center[0]+totalDisY)),(255,0,0),1,cv2.LINE_AA)
                cv2.circle(img,(round(center[1]+totalDisX),round(center[0]+totalDisY)),2,(255,255,255),-1)
                center[1]+=totalDisX
                center[0]+=totalDisY
                totalDisX,totalDisY=disX,disY
                slopeOld=slopeNew
            totalCmeter+=(((disX**2 + disY**2)**0.5))
                
            
            
            
        print(" ",totalCmeter,end="\r")
                
        if pointer==1:
            print("RESET!")
            totalCmeter=0
            img=np.zeros(imgSize,dtype=np.uint8)
            center=[imgSize[0]//2,imgSize[1]//2]
                

                
                


            
        cv2.imshow("img",img)
        
        if cv2.waitKey(1)==ord('q'):
            break
else:
    print("Device not found")
            
