import hid
import time
import cv2
import numpy as np
import threading
import math

dataList=np.empty((0,2))

PIXEL_TO_CMETER=100

pointer=0


dist=0

total_dist=0.0

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
        


devices=hid.enumerate()


for device in devices:
    if device['product_string'] == 'Gaming Mouse':
        isDeviceFounded=1
        break
    else:
        isDeviceFounded=0
    

if isDeviceFounded:
    vendor_id,product_id=device['vendor_id'],device['product_id']
    

    device=hid.Device(vendor_id,product_id)    
    
    dataThread = threading.Thread(target=collectXYData,args=(device,))
    dataThread.start()
    
    totalx=0
    totaly=0
    
    while True:
        
        if dataList.shape[0]>0:
            
            tempDataList=dataList
            
            total_dist=np.sum(tempDataList,axis=0)
            
            totalx+=total_dist[0]
            totaly+=total_dist[1]
            
            print(((totalx**2 + totaly**2)**0.5)/275)
            
            if pointer==1:
                totalx,totaly=0,0
            
            dataList=dataList[tempDataList.shape[0]:]
            time.sleep(0)
else:
    print("Device not found")
            
