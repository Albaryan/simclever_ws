import numpy as np
from ahrs.filters import EKF
from ahrs.common.orientation import acc2q
from ahrs.common.quaternion import Quaternion
from ahrs import RAD2DEG
import serial,time

ser=serial.Serial('/dev/ttyUSB0',115200)
ekf = EKF()

accel_array=np.empty((1,3))
gyro_array=np.empty((1,3))

time.sleep(5)

while not ser.writable():
    pass

ser.write(b'|')
while not ser.readable():
    pass

ax,ay,az,gx,gy,gz,mx,my,mz=ser.read_until(b'\n')[:-2].decode().split('#') 
ser.write(b'|')

Q = acc2q([float(ax),float(ay),float(az)])

while True:
    ser.write(b'|')
    while not ser.readable():
        pass
    ax,ay,az,gx,gy,gz,mx,my,mz=ser.read_until(b'\n')[:-2].decode().split('#') 
    ser.write(b'|')
    
    
    
    Q=ekf.update(Q,[float(gx),float(gy),float(gz)],[float(ax),float(ay),float(az)],[float(mx),float(my),float(mz)])
    print(Quaternion(Q).to_angles()*RAD2DEG)
    time.sleep(0)

# for t in range(1, num_samples):
#     Q = ekf.update(Q, gyr_data, acc_data)