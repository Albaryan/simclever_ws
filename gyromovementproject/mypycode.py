import serial
import time
import threading
import numpy as np
import cv2
import math


counter=0
dataInd=0
medianReady=0
maximum=0
minimum=0

pTime=0

dataList=np.empty((0,2))

def readData():
    
    ser=serial.Serial('/dev/ttyUSB0',115200)
    
    global dataList
    
    if ser.readable():
        ser.read_until(b'#')
    
    while True:
    
        if ser.readable():
            
            data=ser.read_until(b'#').decode()[:-1] #Seri porttaki veriyi oku, string'e dönüştür ve \r karakterine kadar değerlendir

            dx,dy=data.split("|") #Veriyi '|' karakterine göre ayır
            
            dataList=np.append(dataList,[[float(dx),float(dy)]],axis=0)
        
        time.sleep(0)        
        
        
dataThread = threading.Thread(target=readData)
dataThread.start()

counter=0

v=[0]

centersList=np.empty((0,2))

center=[300,300]

img=np.zeros((600,600),dtype=np.uint8)

while True:
    
    
    if dataList.shape[0]!=0:
        
        tempDataList=dataList.copy()
        
        if not abs(tempDataList[0,1])<0.01 or not abs(tempDataList[0,0]<0.01):
            centersList=np.append(centersList,[tempDataList[0]],axis=0)
        else:
            
            x,y=np.sum(centersList,axis=0)
            
            centersList=np.empty((0,2))
            
            cv2.line(img,center,(int(center[0]+x),int(center[1]+y)),(255,255,255),1)
            
            center[0]+=int(x)
            center[1]+=int(y)
            
            print(center)
        
        dataList=dataList[tempDataList.shape[0]:]
        
    cv2.imshow("img",img)    
    
    cv2.waitKey(1)
        
    time.sleep(0)

        
        
        
    
    
    
    
    
        
        
        
            
