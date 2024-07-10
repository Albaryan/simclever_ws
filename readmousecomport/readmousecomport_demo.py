import hid
import time
import cv2
import numpy as np
import threading

dataList=np.empty((0,2))

centersList=np.zeros((2,2))

pointer=0

calculatable=0

dist=0

def collectXYData(device):
    global pointer
    global dataList
    while True:
        mouseData=device.read(64)
        if mouseData[-1]==0 or mouseData[-1]==255 or mouseData[-3]==0 or mouseData[-3]==255:
            pointer,x,y,xMoved,yMoved = mouseData[0],mouseData[-3],mouseData[-1],mouseData[1],mouseData[2]

            if x==0 and xMoved:
                x=1
            elif x==255 and xMoved:
                x=-1

            if y==0 and yMoved:
                y=1
            elif y==255 and yMoved:
                y=-1

            dataList=np.append(dataList,[[x,y]],axis=0)
        time.sleep(0)
        
def calculateDisplacement():
    global centersList
    global center
    global dist
        
    while True:
        if calculatable:
            centersList[0]=center
            
            time.sleep(0.1)
            
            if not calculatable:
                while not calculatable:
                    centersList[1]=center
            else:
                centersList[1]=center
            
            distx=centersList[1,1]-centersList[0,1]
            disty=centersList[1,0]-centersList[0,0]
            
            dist=(distx**2 + disty**2)**0.5
        
        
    

imgSize=[1080,1920]

img=np.zeros(imgSize,dtype=np.uint8)

center=[imgSize[0]//2,imgSize[1]//2]

devices=hid.enumerate()

isDeviceFounded=0

img[center[0],center[1]]=255

print(devices)

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
    
    while True:
        mouseData=device.read(64)
        
        dx,dy=0,0

        #print(mouseData[-4],mouseData[-3],mouseData[-2],mouseData[-1])
        
        if mouseData[-3]==255:
            dx=256-mouseData[-4]
        elif mouseData[-3]==0:
            dx=mouseData[-4]
        
        if mouseData[-1]==255:
            dy=256-mouseData[-2]
        elif mouseData[-1]==0:
            dy=mouseData[-2]
            
        print(dx,dy)
            
        time.sleep(0)
        
        
else:
    print("Device not found")
            
