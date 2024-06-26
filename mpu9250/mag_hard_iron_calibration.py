import time, datetime,serial,re
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import matplotlib.dates as mdates
from collections import deque
import numpy as np

PORT ="/dev/ttyUSB0"

HISTORY_SIZE=5000

INTERVAL=1

serialport=serial.Serial(PORT,115200)

time.sleep(5)

def get_imu_data():
    while not serialport.readable():
        pass
    line=str(serialport.readline(),'utf-8')
    
    
    vals = line.strip().split(',')

    vals=[float(i) for i in vals]

    
    return vals

mag_x=[]
mag_y=[]
mag_z=[]
fig,ax=plt.subplots(1,1)
ax.set_aspect(1)

def onClick(event):
    anim.event_source.stop()
    
def animate(i):
    
    serialport.write(b'|')
    ret=get_imu_data()
    serialport.write(b'|')
    
    x,y,z=ret[6:9]
    
    ax.scatter(x,y, color='r')
    ax.scatter(y,z, color='g')
    ax.scatter(z,x, color='b')
    
    plt.pause(0.000001)
    
fig.canvas.mpl_connect('button_press_event',onClick)
anim=FuncAnimation(fig,animate)


while True:
    animate(0)
    time.sleep(0)

    


