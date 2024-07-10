import serial,time

import numpy as np


ser=serial.Serial('/dev/ttyUSB0',115200)

dataList=np.zeros((0,6))

count_to=2000
i=0

while i<count_to:

    if ser.readable():
        i+=1
        data=ser.read_until(b'#').decode()[:-1] #Seri porttaki veriyi oku, string'e dönüştür ve \r karakterine kadar değerlendir
        
        ax,ay,az,gx,gy,gz=map(float,data.split("|")) #Veriyi '|' karakterine göre ayır
        dataList=np.append(dataList,[[ax,ay,az,gx,gy,gz]],axis=0)
        
        print(dataList.shape)
    
    time.sleep(0)        
    
print(*np.var(dataList,axis=0))