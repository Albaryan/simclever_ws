import pygame
import math
import numpy as np
from OpenGL.GL import *
from OpenGL.GLU import *
from pygame.locals import *
from ahrs.filters import EKF
from ahrs.common.orientation import acc2q
from ahrs.common.quaternion import Quaternion
from ahrs import RAD2DEG
from ahrs import DCM
import serial,time
import serial

useSerial = True # set true for using serial for data transmission, false for wifi
useQuat = False   # set true for using quaternions, false for using y,p,r angles





def main():
    ser = serial.Serial('/dev/ttyUSB0', 115200)
    ekf=EKF(frame='ENU',noises=[0.1**2, 0.3**2, 0.5**2],magnetic_ref=[510.0, 240.0, 570.0])
    
    time.sleep(5)
    
    while not ser.writable():
        pass
    
    ser.write(b'|')
    while not ser.readable():
        pass
    
    ax,ay,az,gx,gy,gz,mx,my,mz=ser.read_until(b'\n')[:-2].decode().split('#') 
    ser.write(b'|')
    
    
    Q = acc2q([float(ax),float(ay),float(az)])    

    video_flags = OPENGL | DOUBLEBUF
    pygame.init()
    screen = pygame.display.set_mode((960, 540), video_flags)
    pygame.display.set_caption("Aldebaran Core AHRS - MPU9255 QUATERNION")
    resizewin(960, 540)
    init()
    frames = 0
    ticks = pygame.time.get_ticks()     
    pTime=time.time()   
    while 1:
        event = pygame.event.poll()
        
        if event.type == QUIT or (event.type == KEYDOWN and event.key == K_ESCAPE):
            break

        ser.write(b'|')
        while not ser.readable():
            pass
        ax,ay,az,gx,gy,gz,mx,my,mz=ser.read_until(b'\n')[:-2].decode().split('#') 
        ser.write(b'|')
        
        print(mx,my,mz)

        cTime=time.time()
        ekf.Dt=(cTime-pTime)
        pTime=cTime

        Q=ekf.update(Q,
                     [float(gx),float(gy),float(gz)],
                     [float(ax),float(ay),float(az)],
                     #[float(mx),float(my),float(mz)]
                     )
        pitch,roll,yaw=DCM(Quaternion(Q).to_DCM()).to_angles()*RAD2DEG
    

        pitch = pitch - 0
        roll = -roll

        draw(1, yaw, pitch, roll)
        pygame.display.flip()
        frames += 1

    print("fps: %d" % ((frames*1000)/(pygame.time.get_ticks()-ticks)))
    if(useSerial):
        ser.close()


def resizewin(width, height):
    """
    For resizing window
    """
    if height == 0:
        height = 1
    glViewport(0, 0, width, height)
    glMatrixMode(GL_PROJECTION)
    glLoadIdentity()
    gluPerspective(45, 1.0*width/height, 0.1, 100.0)
    glMatrixMode(GL_MODELVIEW)
    glLoadIdentity()


def init():
    glShadeModel(GL_SMOOTH)
    glClearColor(0.0, 0.0, 0.0, 0.0)
    glClearDepth(1.0)
    glEnable(GL_DEPTH_TEST)
    glDepthFunc(GL_LEQUAL)
    glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST)

def draw(w, nx, ny, nz):
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
    glLoadIdentity()
    glTranslatef(0, 0.0, -7.0)

    drawText((-2.6, 1.8, 2), "Aldebaran Core AHRS", 18)
    drawText((-2.6, 1.6, 2), "MPU9250 AHRS Implementation - Using Quaternion based filter", 16)
    drawText((-2.6, -2, 2), "Press Escape to exit.", 16)

    yaw = nx
    pitch = ny
    roll = nz
    drawText((-2.6, -1.8, 2), "Yaw: %f, Pitch: %f, Roll: %f" %(yaw, pitch, roll), 16)
    glRotatef(-roll, 0.00, 0.00, 1.00)
    glRotatef(pitch, 1.00, 0.00, 0.00)
    glRotatef(yaw, 0.00, 1.00, 0.00)

    glBegin(GL_QUADS)
    glColor3f(0.0, 1.0, 0.0)
    glVertex3f(1.0, 0.2, -1.0)
    glVertex3f(-1.0, 0.2, -1.0)
    glVertex3f(-1.0, 0.2, 1.0)
    glVertex3f(1.0, 0.2, 1.0)

    glColor3f(1.0, 0.5, 0.0)
    glVertex3f(1.0, -0.2, 1.0)
    glVertex3f(-1.0, -0.2, 1.0)
    glVertex3f(-1.0, -0.2, -1.0)
    glVertex3f(1.0, -0.2, -1.0)

    glColor3f(1.0, 0.0, 0.0)
    glVertex3f(1.0, 0.2, 1.0)
    glVertex3f(-1.0, 0.2, 1.0)
    glVertex3f(-1.0, -0.2, 1.0)
    glVertex3f(1.0, -0.2, 1.0)

    glColor3f(1.0, 1.0, 0.0)
    glVertex3f(1.0, -0.2, -1.0)
    glVertex3f(-1.0, -0.2, -1.0)
    glVertex3f(-1.0, 0.2, -1.0)
    glVertex3f(1.0, 0.2, -1.0)

    glColor3f(0.0, 0.0, 1.0)
    glVertex3f(-1.0, 0.2, 1.0)
    glVertex3f(-1.0, 0.2, -1.0)
    glVertex3f(-1.0, -0.2, -1.0)
    glVertex3f(-1.0, -0.2, 1.0)

    glColor3f(1.0, 0.0, 1.0)
    glVertex3f(1.0, 0.2, -1.0)
    glVertex3f(1.0, 0.2, 1.0)
    glVertex3f(1.0, -0.2, 1.0)
    glVertex3f(1.0, -0.2, -1.0)
    glEnd()


def drawText(position, textString, size):
    font = pygame.font.SysFont("Courier", size, True)
    textSurface = font.render(textString, True, (255, 255, 255, 255), (0, 0, 0, 255))
    textData = pygame.image.tostring(textSurface, "RGBA", True)
    glRasterPos3d(*position)
    glDrawPixels(textSurface.get_width(), textSurface.get_height(), GL_RGBA, GL_UNSIGNED_BYTE, textData)

if __name__ == '__main__':
    main()