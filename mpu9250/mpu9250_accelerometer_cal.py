######################################################
# Copyright (c) 2021 Maker Portal LLC
# Author: Joshua Hrisko
######################################################
#
# This code reads data from the MPU9250/MPU9265 board
# (MPU6050 - accel/gyro, AK8963 - mag)
# and solves for calibration coefficients for the
# accelerometer
#
#
######################################################
#
# wait 5-sec for IMU to connect

### calibration parameters ##################################
#[array([0.99909876, 0.01132312]), array([-0.99893015, -0.0091835 ]), array([-0.98948227,  0.02932166])]
#############################################################

import time,sys
import serial
sys.path.append('../')
t0 = time.time()
start_bool = False # if IMU start fails - stop calibration
while time.time()-t0<5:
    try: 
        ser=serial.Serial('/dev/ttyUSB0',115200)
        start_bool = True
        break
    except:
        continue
import numpy as np
import csv,datetime
import matplotlib.pyplot as plt
from scipy.optimize import curve_fit 
from scipy.integrate import cumtrapz
from scipy import signal

time.sleep(2) # wait for MPU to load and settle
# 
#####################################
# Accel Calibration (gravity)
#####################################
#
def accel_fit(x_input,m_x,b):
    return (m_x*x_input)+b # fit equation for accel calibration
#
def get_accel():
    while not ser.readable():
        pass
    ax,ay,az = ser.read_until(b'\n')[:-2].decode().split('#') # read and convert accel data
    print(ax,ay,az,end='\r')
    return float(ax),float(ay),float(az)
    
def accel_cal():
    print("-"*50)
    print("Accelerometer Calibration")
    mpu_offsets = [[],[],[]] # offset array to be printed
    axis_vec = ['z','y','x'] # axis labels
    cal_directions = ["upward","downward","perpendicular to gravity"] # direction for IMU cal
    cal_indices = [2,1,0] # axis indices
    for qq,ax_qq in enumerate(axis_vec):
        ax_offsets = [[],[],[]]
        print("-"*50)
        for direc_ii,direc in enumerate(cal_directions):
            input("-"*8+" Press Enter and Keep IMU Steady to Calibrate the Accelerometer with the -"+\
              ax_qq+"-axis pointed "+direc)
            ser.read_all()
            ser.write(b'|')
            mpu_array = []
            while len(mpu_array)<cal_size:
                try:
                    ax,ay,az = get_accel()
                    mpu_array.append([ax,ay,az]) # append to array
                except:
                    continue
            print("\n")
            ser.write(b'|')
            ax_offsets[direc_ii] = np.array(mpu_array)[:,cal_indices[qq]] # offsets for direction

        # Use three calibrations (+1g, -1g, 0g) for linear fit
        popts,_ = curve_fit(accel_fit,np.append(np.append(ax_offsets[0],
                                 ax_offsets[1]),ax_offsets[2]),
                   np.append(np.append(1.0*np.ones(np.shape(ax_offsets[0])),
                    -1.0*np.ones(np.shape(ax_offsets[1]))),
                        0.0*np.ones(np.shape(ax_offsets[2]))),
                            maxfev=10000)
        mpu_offsets[cal_indices[qq]] = popts # place slope and intercept in offset array
    print('Accelerometer Calibrations Complete')
    return mpu_offsets

def imu_integrator():
    #############################
    # Main Loop to Integrate IMU
    #############################
    #
    data_indx = 1 # index of variable to integrate
    dt_stop = 5 # seconds to record and integrate

    plt.style.use('ggplot')
    plt.ion()
    fig,axs = plt.subplots(3,1,figsize=(12,9))
    break_bool = False
    while True:
        #
        ##################################
        # Reading and Printing IMU values 
        ##################################
        #
        accel_array,t_array = [],[]
        print("Starting Data Acquisition")
        [axs[ii].clear() for ii in range(0,3)]
        t0 = time.time()
        loop_bool = False
        while True:
            try:
                ser.write(b'|')
                ax,ay,az=get_accel()
                ser.write(b'|')
                t_array.append(time.time()-t0)
                data_array = [ax,ay,az]
                accel_array.append(accel_fit(data_array[data_indx],
                                             *accel_coeffs[data_indx]))
                if not loop_bool:
                    loop_bool = True
                    print("Start Moving IMU...")
            except:
                continue
            if time.time()-t0>dt_stop:
                print("Data Acquisition Stopped")
                break
            
        if break_bool:
            break
        #
        ##################################
        # Signal Filtering
        ##################################
        #
        Fs_approx = len(accel_array)/dt_stop
        b_filt,a_filt = signal.butter(4,5,'low',fs=Fs_approx)
        accel_array = signal.filtfilt(b_filt,a_filt,accel_array)
        accel_array = np.multiply(accel_array,9.80665)
        #
        ##################################
        # Print Sample Rate and Accel
        # Integration Value
        ##################################
        #
        print("Sample Rate: {0:2.0f}Hz".format(len(accel_array)/dt_stop))
        veloc_array = np.append(0.0,cumtrapz(accel_array,x=t_array))
        dist_approx = np.trapz(veloc_array,x=t_array)
        dist_array = np.append(0.0,cumtrapz(veloc_array,x=t_array))
        print("Displace in y-dir: {0:2.2f}m".format(dist_approx))
        axs[0].plot(t_array,accel_array,label="$"+mpu_labels[data_indx]+"$",
                    color=plt.cm.Set1(0),linewidth=2.5)
        axs[1].plot(t_array,veloc_array,
                    label="$v_"+mpu_labels[data_indx].split("_")[1]+"$",
                    color=plt.cm.Set1(1),linewidth=2.5)
        axs[2].plot(t_array,dist_array,
                    label="$d_"+mpu_labels[data_indx].split("_")[1]+"$",
                    color=plt.cm.Set1(2),linewidth=2.5)
        [axs[ii].legend() for ii in range(0,len(axs))]
        axs[0].set_ylabel('Acceleration [m$\cdot$s$^{-2}$]',fontsize=16)
        axs[1].set_ylabel('Velocity [m$\cdot$s$^{-1}$]',fontsize=16)
        axs[2].set_ylabel('Displacement [m]',fontsize=16)
        axs[2].set_xlabel('Time [s]',fontsize=18)
        axs[0].set_title("MPU9250 Accelerometer Integration",fontsize=18)
        plt.pause(0.01)
        plt.savefig("accel_veloc_displace_integration.png",dpi=300,
                    bbox_inches='tight',facecolor="#FCFCFC")

if __name__ == '__main__':
    if not start_bool:
        print("IMU not Started - Check Wiring and I2C")
    else:
        #
        ###################################
        # Accelerometer Gravity Calibration
        ###################################
        #
        mpu_labels = ['a_x','a_y','a_z'] # gyro labels for plots
        cal_size = 1000 # number of points to use for calibration 
        accel_coeffs = accel_cal() # grab accel coefficients
        print(accel_coeffs)
#
        ###################################
        # Record new data 
        ###################################
        #
#
        ###################################
        # Record new data 
        ###################################
        #
        ser.write(b'|')
        data = np.array([get_accel() for ii in range(0,cal_size)]) # new values
        ser.write(b'|')
        
        accel_labels = ['a_x','a_y','a_z']

        plt.style.use('ggplot')
        fig,axs = plt.subplots(2,1,figsize=(12,9))
        for ii in range(0,3):
            axs[0].plot(data[:,ii],
                        label='${}$, Uncalibrated'.format(accel_labels[ii]))
            axs[1].plot(accel_fit(data[:,ii],*accel_coeffs[ii]),
                        label='${}$, Calibrated'.format(accel_labels[ii]))
        axs[0].legend(fontsize=14);axs[1].legend(fontsize=14)
        axs[0].set_ylabel('$a_{x,y,z}$ [g]',fontsize=18)
        axs[1].set_ylabel('$a_{x,y,z}$ [g]',fontsize=18)
        axs[1].set_xlabel('Sample',fontsize=18)
        axs[0].set_ylim([-2,2]);axs[1].set_ylim([-2,2])
        axs[0].set_title('Accelerometer Calibration Calibration Correction',fontsize=18)
        fig.savefig('accel_calibration_output.png',dpi=300,
                    bbox_inches='tight',facecolor='#FCFCFC')
        fig.show()