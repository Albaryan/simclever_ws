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

            roll,pitch=data.split("|") #Veriyi '|' karakterine göre ayır
            
            dataList=np.append(dataList,[[float(roll),float(pitch)]],axis=0)
        
        time.sleep(0)        
        
        
dataThread = threading.Thread(target=readData)
dataThread.start()

centersList=np.empty((0,2))

center=[300,300]

img=np.zeros((600,600,3),dtype=np.uint8)

while True:
    
    img= np.zeros_like(img)
    
    if dataList.shape[0]!=0:
        
        tempDataList=dataList.copy()
        
        print(100*math.cos((tempDataList[0,0])*math.pi/180), -100*math.sin((tempDataList[0,0])*math.pi/180))
        
        cv2.line(img,center,(center[0]+round(100*math.cos((tempDataList[0,1])*math.pi/180)), center[1]+round(100*math.sin((tempDataList[0,1])*math.pi/180))),(0,0,255),1,cv2.LINE_AA)
        
        cv2.line(img,center,(center[0]-round(100*math.cos((tempDataList[0,1])*math.pi/180 - math.pi/2)), center[1]+round(100*math.sin((tempDataList[0,0])*math.pi/180))), (255,0 ,0), 1, cv2.LINE_AA)
        
        
        dataList=dataList[tempDataList.shape[0]:]
        
    cv2.imshow("img",img)    
    
    cv2.waitKey(1)
        
    time.sleep(0.05)

        
        
        
    
    
    
    
    
        
        
        
            
