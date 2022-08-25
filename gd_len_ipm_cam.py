import cv2
import numpy as np
import math
from datetime import datetime
import pandas as pd
import time
from dynamixel_sdk import *                    # Uses Dynamixel SDK library

# Control table address
ADDR_PRO_TORQUE_ENABLE      = 64               # Control table address is different in Dynamixel model
ADDR_PRO_GOAL_POSITION      = 116
ADDR_PRO_PRESENT_POSITION   = 132

# Data Byte Length
LEN_PRO_GOAL_POSITION       = 112
LEN_PRO_PRESENT_POSITION    = 108

# Protocol version
PROTOCOL_VERSION            = 2.0               # See which protocol version is used in the Dynamixel

# Default setting
DXL1_ID                     = 1                 # Dynamixel#1 ID : 1
DXL2_ID                     = 3                 # Dynamixel#1 ID : 2
DXL3_ID                     = 5                 # Dynamixel#1 ID : 3
DXL4_ID                     = 7                 # Dynamixel#1 ID : 4
DXL5_ID                     = 9                 # Dynamixel#1 ID : 5
DXL6_ID                     = 11                # Dynamixel#1 ID : 6
BAUDRATE                    = 3000000           # Dynamixel default baudrate : 57600
DEVICENAME                  = 'COM4'            # Check which port is being used on your controller
                                                # ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"

TORQUE_ENABLE               = 1                 # Value for enabling the torque
TORQUE_DISABLE              = 0                 # Value for disabling the torque
DXL_MOVING_STATUS_THRESHOLD = 20                # Dynamixel moving status threshold



portHandler = PortHandler(DEVICENAME)
packetHandler = PacketHandler(PROTOCOL_VERSION)


# Open port
portHandler.openPort()
# Set port baudrate
portHandler.setBaudRate(BAUDRATE)   
#Q=int(input('馬達控制參數如下 : \n1.PWM、velocity、position\n2.velocity、position\n3.PID\n請輸入控制方案為 : '))
#print('控制方案:',Q)
groupSyncWrite = GroupSyncWrite(portHandler, packetHandler, 112, 8)

#讀取目標位置、速度、電流
groupSyncRead = GroupSyncRead(portHandler, packetHandler, 132, 4)

# Add parameter storage for Dynamixel#1 present position value
groupSyncRead.addParam(DXL1_ID)
# Add parameter storage for Dynamixel#2 present position value
groupSyncRead.addParam(DXL2_ID)
# Add parameter storage for Dynamixel#1 present position value
groupSyncRead.addParam(DXL3_ID)
# Add parameter storage for Dynamixel#2 present position value
groupSyncRead.addParam(DXL4_ID)
# Add parameter storage for Dynamixel#1 present position value
groupSyncRead.addParam(DXL5_ID)
# Add parameter storage for Dynamixel#2 present position value
groupSyncRead.addParam(DXL6_ID)

def readangle():
    global groupSyncRead, DXL1_ID, DXL2_ID, DXL3_ID, DXL4_ID, DXL5_ID, DXL6_ID
    groupSyncRead.txRxPacket()
    # Get Dynamixel#1 present position value
    
    # dxl1_present_velocity = groupSyncRead.getData(DXL1_ID, 128, 4)
    dxl1_present_position = groupSyncRead.getData(DXL1_ID, 132, 4)
    
    # dxl2_present_velocity = groupSyncRead.getData(DXL2_ID, 128, 4)
    dxl2_present_position = groupSyncRead.getData(DXL2_ID, 132, 4)
    
    # dxl3_present_velocity = groupSyncRead.getData(DXL3_ID, 128, 4)
    dxl3_present_position = groupSyncRead.getData(DXL3_ID, 132, 4)
    
    # dxl4_present_velocity = groupSyncRead.getData(DXL4_ID, 128, 4)
    dxl4_present_position = groupSyncRead.getData(DXL4_ID, 132, 4)
    
    # dxl5_present_velocity = groupSyncRead.getData(DXL5_ID, 128, 4)
    dxl5_present_position = groupSyncRead.getData(DXL5_ID, 132, 4)
    
    # dxl6_present_velocity = groupSyncRead.getData(DXL6_ID, 128, 4)
    dxl6_present_position = groupSyncRead.getData(DXL6_ID, 132, 4)
          
    angle_1 = +(dxl1_present_position-2048)/4096*360
    angle_2 = -(dxl2_present_position-2048)/4096*360
    angle_3 = +(dxl3_present_position-2048)/4096*360
    angle_4 = -(dxl4_present_position-2048)/4096*360
    angle_5 = +(dxl5_present_position-2048)/4096*360
    angle_6 = -(dxl6_present_position-2048)/4096*360
    return angle_1, angle_2, angle_3, angle_4, angle_5, angle_6


def display(im, bbox):
    bbox = np.array(bbox,dtype='int')
    n = len(np.array(bbox))
    for j in range((n-1)):
        cv2.line(im, (bbox[j][0],bbox[j][1]), (bbox[j+1][0],bbox[j+1][1]), (255,0,0), 2)
    cv2.line(im, (bbox[0]), (bbox[3]), (255,0,0), 3)
    # Display results
    return im

def Calcost(Q12,Q23,Q13,z,x1,y1,x2,y2,x3,y3,Z1,Z2,Z3):
    X1 =Z1*x1/z 
    X2 =Z2*x2/z 
    X3 =Z3*x3/z
    
    Y1 =Z1*y1/z 
    Y2 =Z2*y2/z 
    Y3 =Z3*y3/z
    
    Q1Q2 = pow(X2-X1,2)+pow(Y2-Y1,2)+pow(Z2-Z1,2)
    Q2Q3 = pow(X3-X2,2)+pow(Y3-Y2,2)+pow(Z3-Z2,2)
    Q1Q3 = pow(X3-X1,2)+pow(Y3-Y1,2)+pow(Z3-Z1,2)
    
    #cost =pow( (Q1Q2+Q2Q3+Q1Q3) - (Q12+Q23+Q13),2)
    cost = pow(Q1Q2-Q12,2)+pow(Q2Q3-Q23,2)+pow(Q1Q3-Q13,2)
    return cost

def CalcostZ1(Q12,Q23,Q13,z,x1,y1,x2,y2,x3,y3,Z1,Z2,Z3):
    #𝜕cost/𝜕𝑍1
    dcdz1 = 2.0 * ((pow(Z2*x2-Z1*x1,2)+pow(Z2*y2-Z1*y1,2))/(pow(z,2))+pow(Z2-Z1,2)-Q12) * ((-2*x1*(-x1*Z1+Z2*x2)-2*y1*(-y1*Z1+Z2*y2))/(pow(z,2))-2*(-Z1+Z2)) + 2.0 * ((pow(Z3*x3-Z1*x1,2)+pow(Z3*y3-Z1*y1,2))/(pow(z,2))+pow(Z3-Z1,2)-Q13) * ((-2*x1*(-x1*Z1+Z3*x3)-2*y1*(-y1*Z1+Z3*y3))/(pow(z,2))-2*(-Z1+Z3))
    return dcdz1

def CalcostZ2(Q12,Q23,Q13,z,x1,y1,x2,y2,x3,y3,Z1,Z2,Z3):
    #𝜕cost/𝜕𝑍2
    dcdz2 = 2.0 * ((pow(Z2*x2-Z1*x1,2)+pow(Z2*y2-Z1*y1,2))/(pow(z,2))+pow(Z2-Z1,2)-Q12) * ((2*x2*(x2*Z2-Z1*x1)+2*y2*(y2*Z2-Z1*y1))/(pow(z,2))+2*(Z2-Z1)) + 2.0 * ((pow(Z3*x3-Z2*x2,2)+pow(Z3*y3-Z2*y2,2))/(pow(z,2))+pow(Z3-Z2,2)-Q23) * ((-2*x2*(-x2*Z2+Z3*x3)-2*y2*(-y2*Z2+Z3*y3))/(pow(z,2))-2*(-Z2+Z3))
    return dcdz2

def CalcostZ3(Q12,Q23,Q13,z,x1,y1,x2,y2,x3,y3,Z1,Z2,Z3):
    #𝜕cost/𝜕𝑍3
    dcdz3 = 2.0 * ((pow(Z3*x3-Z2*x2,2)+pow(Z3*y3-Z2*y2,2))/(pow(z,2))+pow(Z3-Z2,2)-Q23) * ((2*x3*(x3*Z3-Z2*x2)+2*y3*(y3*Z3-Z2*y2))/(pow(z,2))+2*(Z3-Z2)) + 2.0 * ((pow(Z3*x3-Z1*x1,2)+pow(Z3*y3-Z1*y1,2))/(pow(z,2))+pow(Z3-Z1,2)-Q13) * ((2*x3*(x3*Z3-Z1*x1)+2*y3*(y3*Z3-Z1*y1))/(pow(z,2))+2*(Z3-Z1))
    return dcdz3

def solver(Q12,Q23,Q13,z,x1,y1,x2,y2,x3,y3,Z1,Z2,Z3):
    #rate=0.001
    rate=0.0009
    #d=0.02
   
    TIMEOUT=50000
   
    while True:
        
        # Z1 updata
        ##cost = Calcost(Q12,Q23,Q13,z,x1,y1,x2,y2,x3,y3,Z1,Z2,Z3)
        ##costz1 = Calcost(Q12,Q23,Q13,z,x1,y1,x2,y2,x3,y3,(Z1+d),Z2,Z3)
        ##dcdz1 =(costz1 - cost)/d
        
        dcdz1 = CalcostZ1(Q12,Q23,Q13,z,x1,y1,x2,y2,x3,y3,Z1,Z2,Z3)
        Z1-=rate*dcdz1
        
        #print("dcdz1 ",np.round(dcdz1,8))
        #print("cost0 ",np.round(cost0,8))
        #print("costZ1 ",np.round(costZ1,8))
        
        # Z2 updata
        # cost = Calcost(Q12,Q23,Q13,z,x1,y1,x2,y2,x3,y3,Z1,Z2,Z3)
        # costz2 = Calcost(Q12,Q23,Q13,z,x1,y1,x2,y2,x3,y3,Z1,(Z2+d),Z3)
        # dcdz2 =(costz2 - cost)/d
        
        dcdz2 = CalcostZ2(Q12,Q23,Q13,z,x1,y1,x2,y2,x3,y3,Z1,Z2,Z3)
        Z2-=rate*dcdz2
        #print("dcdz2 ",np.round(dcdz2,8))
        
        # Z3 updata
        # cost = Calcost(Q12,Q23,Q13,z,x1,y1,x2,y2,x3,y3,Z1,Z2,Z3)
        # costz3 = Calcost(Q12,Q23,Q13,z,x1,y1,x2,y2,x3,y3,Z1,Z2,(Z3+d))
        # dcdz3 =(costz3 - cost)/d
        
        dcdz3 = CalcostZ3(Q12,Q23,Q13,z,x1,y1,x2,y2,x3,y3,Z1,Z2,Z3)
        Z3-=rate*dcdz3
        #print("dcdz3 ",np.round(dcdz3,8))
        
        fin = abs(dcdz1)+abs(dcdz2)+abs(dcdz3)
        
        TIMEOUT=TIMEOUT-1
        if TIMEOUT<0:
            #print("timeout ",TIMEOUT)
            #print('timeoutfin ',np.round(fin,4))
            break
        if fin <0.001:
            print('fin ',np.round(fin,6))
            break
        
    return Z1,Z2,Z3,fin

#剃度下降法搜尋Q1Q2、Q1Q3相除為1
def cal_lencost(Q12,Q23,Q13,z,x1,y1,x2,y2,x3,y3,Z1,Z2,Z3):
    
    X1 =Z1*x1/z 
    X2 =Z2*x2/z 
    X3 =Z3*x3/z
    
    Y1 =Z1*y1/z 
    Y2 =Z2*y2/z 
    Y3 =Z3*y3/z
    
    Q1Q2 = pow(X2-X1,2)+pow(Y2-Y1,2)+pow(Z2-Z1,2)
    Q1Q3 = pow(X3-X1,2)+pow(Y3-Y1,2)+pow(Z3-Z1,2)
    costlen = pow((Q1Q2/Q1Q3)-1,2)
    return costlen

def CalcostlenZ1(Q12,Q23,Q13,z,x1,y1,x2,y2,x3,y3,Z1,Z2,Z3):
    #𝜕costlen/𝜕𝑍1
    lendcdz1 = (4.0*(((((x2*Z2-x1*Z1)**2)+((y2*Z2-y1*Z1)**2)+(((z**2)*(Z2-Z1))**2))/(((x3*Z3-x1*Z1)**2)+((y3*Z3-y1*Z1)**2)+(((z**2)*(Z3-Z1))**2)))-1)*((((x2*Z2-x1*Z1)**2)+((y2*Z2-y1*Z1)**2)+(z**2)*((Z2-Z1)**2))*(Z3*(x1*x3+y1*y3+(z**2))-(Z1*((x1**2)+(y1**2)+(z**2))))-(Z2*((x1*x2)+(y1*y2)+(z**2))-(Z1*((x1**2)+(y1**2)+(z**2))))*((((x3*Z3)-(x1*Z1))**2)+(((y3*Z3)-(y1*Z1))**2)+((z**2)*((Z3-Z1)**2)))))/(((((x3*Z3)-(x1*Z1))**2)+(((y3*Z3)-(y1*Z1))**2)+((z**2)*((Z3-Z1)**2)))**2)
    
    #cost_len/d
    return lendcdz1

def CalcostlenZ2(Q12,Q23,Q13,z,x1,y1,x2,y2,x3,y3,Z1,Z2,Z3):
    #𝜕costlen/𝜕𝑍2
    lendcdz2 = (4.0*(Z1*(x1*x2+y1*y2+(z**2))-(Z2*((x2**2)+(y2**2)+(z**2))))*(2*Z2*Z1*(x1*x2+y1*y2+(z**2))+Z3*(-2*x1*x3*Z1+(x3**2)*Z3-2*y1*y3*Z1+(y3**2)*Z3-2*(z**2)*Z1+(z**2)*Z3))+((Z2**2)*(-((x2**2)+(y2**2)+(z**2)))))/(((x1**2)*(Z1**2)-(2*x1*x3*Z1*Z3)+(x3**2)*(Z3**2)+(y1**2)*(Z1**2)-(2*y1*y3*Z1*Z3)+(y3**2)*(Z3**2)+(z**2)*(Z1**2)-(2*(z**2)*Z1*Z3)+(z**2)*(Z3**2)**2))
    return lendcdz2

def CalcostlenZ3(Q12,Q23,Q13,z,x1,y1,x2,y2,x3,y3,Z1,Z2,Z3):
    #𝜕costlen/𝜕𝑍3
    lendcdz3 = -((4.0*((x1**2)*(Z1**2)-(2*x1*x2*Z1*Z2)+(x2**2)*(Z2**2)+(y1**2)*(Z1**2)-(2*y1*y2*Z1*Z2)+(y2**2)*(Z2**2)+(z**2)*(Z1**2)-(2*(z**2)*Z1*Z2)+(z**2)*(Z2**2))*(Z1*(x1*x3+y1*y3+(z**2))-Z3*((x3**2)+(y3**2)+(z**2)))*(-Z2*(-2*x1*x2*Z1+(x2**2)*Z2)-(2*y1*y2*Z1)+(y2**2)*Z2-(2*(z**2)*Z1+(z**2)*Z2))-(2*Z3*Z1*(x1*x3+y1*y3+(z**2)))+(Z3**2)*((x3**2)+(y3**2)+(z**2)))/((((Z1**2)*((x1**2)+(y1**2)+(z**2)))-(2*Z3*Z1*(x1*x3+y1*y3+(z**2)))+(Z3**2)*((x3**2)+(y3**2)+(z**2)))**3))
    return lendcdz3

def lensolver(Q12,Q23,Q13,z,x1,y1,x2,y2,x3,y3,Z1,Z2,Z3):
    #rate_1=0.0001
    d=0.02
    TIMEOUT=50000
   
    while True:
        
        # lendcdz1 = CalcostlenZ1(Q12,Q23,Q13,z,x1,y1,x2,y2,x3,y3,Z1,Z2,Z3)               
        # Z1-=rate_1*lendcdz1
        
        costlen = cal_lencost(Q12,Q23,Q13,z,x1,y1,x2,y2,x3,y3,Z1,Z2,Z3)
        costlenz1 = cal_lencost(Q12,Q23,Q13,z,x1,y1,x2,y2,x3,y3,(Z1+d),Z2,Z3)
        lendcdz1 =(costlenz1 - costlen)/d
        
        # lendcdz2 = CalcostlenZ2(Q12,Q23,Q13,z,x1,y1,x2,y2,x3,y3,Z1,Z2,Z3)
        # Z2-=rate_1*lendcdz2
        
        costlen = cal_lencost(Q12,Q23,Q13,z,x1,y1,x2,y2,x3,y3,Z1,Z2,Z3)
        costlenz2 = cal_lencost(Q12,Q23,Q13,z,x1,y1,x2,y2,x3,y3,Z1,(Z2+d),Z3)
        lendcdz2 =(costlenz2 - costlen)/d
        
        # lendcdz3 = CalcostlenZ3(Q12,Q23,Q13,z,x1,y1,x2,y2,x3,y3,Z1,Z2,Z3)
        # Z3-=rate_1*lendcdz3
        
        costlen = cal_lencost(Q12,Q23,Q13,z,x1,y1,x2,y2,x3,y3,Z1,Z2,Z3)
        costlenz3 = cal_lencost(Q12,Q23,Q13,z,x1,y1,x2,y2,x3,y3,Z1,Z2,(Z3+d))
        lendcdz3 =(costlenz3 - costlen)/d
        
        fin_len = abs(lendcdz1)+abs(lendcdz2)+abs(lendcdz3)
        
        TIMEOUT=TIMEOUT-1
        if TIMEOUT<0:
            #print("timeout ",TIMEOUT)
            #print('timeoutfin ',np.round(fin,4))
            break
        if fin_len <0.001:
            #print('fin_len ',np.round(fin_len,4))
            break
        
    return Z1,Z2,Z3,fin_len
        
def ipm(p1,p2,p3,Width,Heigh,Q12,Q23,Q13,z,Z1,Z2,Z3):   
    x1 = p1[0]-Width/2
    y1 = p1[1]-Heigh/2
    x2 = p2[0]-Width/2
    y2 = p2[1]-Heigh/2
    x3 = p3[0]-Width/2
    y3 = p3[1]-Heigh/2
    
    Z1,Z2,Z3,fin = solver(Q12,Q23,Q13,z,x1,y1,x2,y2,x3,y3,Z1,Z2,Z3)
    Z1,Z2,Z3,fin_len = lensolver(Q12,Q23,Q13,z,x1,y1,x2,y2,x3,y3,Z1,Z2,Z3)
    
    X1 = Z1*x1/z
    Y1 = Z1*y1/z
    X2 = Z2*x2/z
    Y2 = Z2*y2/z
    X3 = Z3*x3/z
    Y3 = Z3*y3/z
    
    Xx = (X2-X1)
    Xy = (Y2-Y1)
    Xz = (Z2-Z1)
    
    S = pow((pow(Xx,2)+pow(Xy,2)+pow(Xz,2)),0.5)
    
    Xx = Xx / S
    Xy = Xy / S
    Xz = Xz / S
    
    Yx = (X3-X1)
    Yy = (Y3-Y1)
    Yz = (Z3-Z1)
    
    S = pow((pow(Yx,2)+pow(Yy,2)+pow(Yz,2)),0.5)
    
    Yx = Yx / S
    Yy = Yy / S
    Yz = Yz / S
    
    
    a = np.array([Xx,Xy,Xz])
    b = np.array([Yx,Yy,Yz])
    c = np.cross(a, b)
    
    p0 = np.array([X1,Y1,Z1])
    p1 = np.array([X2,Y2,Z2])
    p2 = np.array([X3,Y3,Z3])
    
    # cTw = np.array([[a[0],a[1],a[2],0],
    #                 [b[0],b[1],b[2],0],
    #                 [c[0],c[1],c[2],0],
    #                 [p[0],p[1],p[2],1]])
    
    cTw0 = np.array([[a[0],b[0],c[0],p0[0]],
                    [a[1],b[1],c[1],p0[1]],
                    [a[2],b[2],c[2],p0[2]],
                    [   0,   0,   0,  1]])
    
    cTw1 = np.array([[a[0],b[0],c[0],p1[0]],
                    [a[1],b[1],c[1],p1[1]],
                    [a[2],b[2],c[2],p1[2]],
                    [   0,   0,   0,  1]])
    
    cTw2 = np.array([[a[0],b[0],c[0],p2[0]],
                    [a[1],b[1],c[1],p2[1]],
                    [a[2],b[2],c[2],p2[2]],
                    [   0,   0,   0,  1]])
    
    det = np.linalg.det(cTw0)
    if det != 0:
        wTc0 = np.linalg.inv(cTw0)
    
    #print('cTw ',np.round(cTw,3))
    #print('wTc \n',np.round(wTc0,3)) #ans
    
    alpha,beta,gamma = GetEulaDeg(wTc0)
    
    #print('X1 alpha ',np.round(alpha,4))
    #print('Y2 beta  ',np.round(beta,4))
    #print('Z3 gamma ',np.round(gamma,4))
    
    #print('Z1 ',np.round(Z1,4))
    #print('Z2 ',np.round(Z2,4))
    #print('Z3 ',np.round(Z3,4))
    return cTw0,cTw1,cTw2,fin,fin_len

def GetEulaDeg(wTc):
    alpha = math.atan2(-wTc[1,2],wTc[2,2])*180.0/math.pi
    beta = math.atan2(wTc[0,2], pow( 1.0-pow(wTc[0,2],2),0.5))*180.0/math.pi
    gamma = math.atan2(-wTc[0,1],wTc[0,0])*180.0/math.pi
    return alpha,beta,gamma

# def Pixelangle(wTc,p1,p2):
#     #p1、p2轉換成wTp1、wTp2
#     wTp1 = wTc * cTp1
#     wTp2 = wTc * cTp2
    
#     #1個pixel角度
#     view = ((wTp2-wTc)*(wTp1-wTc))/((pow(abs(wTp2-wTc),0.5))*(pow(abs(wTp1-wTc),0.5)))
#     angle = math.acos(view)
#     #一個pixel占多少角度
#     pixelangle = angle/(pow(pow(p2,2)+pow(p1,2),0.5))
#     #print('pixelangle', np.round(pixelangle,3)
#     #print('pixelangle', pixelangle)
#     return pixelangle
def Getpixel(wTc,cTq1,cTq2,p1,p2):
    wTp1 =wTc @ cTq1
    wTp2 =wTc @ cTq2
    
    view = sum(((wTp2[:3,3]-wTc[:3,3])*(wTp1[:3,3]-wTc[:3,3])))/((pow(sum((wTp2[:3,3]-wTc[:3,3])**2),0.5))*(pow(sum((wTp1[:3,3]-wTc[:3,3])**2),0.5)))
    angle = np.arccos(view)*180.0/np.pi
    #一個pixel占多少角度
    pixelangle = (angle)/ ((sum((p2-p1)**2))**0.5)
    #print('angle', np.round(angle,3),' pixelangle', np.round(pixelangle,3))
    return pixelangle,angle

#def create_format(wTc,wTq1,wTq2,finc,finq1,finq2,alpha,beta,gamma):
# def create_format(wTc,finc,alpha,beta,gamma,pix,cTw):
def create_format(cTw,wTc,cTbase,wTbase,w_Eu0,w_Eu1,w_Eu2,w_pix,w_angle0,w_angle1,w_angle2,w_angle3,w_angle4,w_angle5,finc,cTq,qTc,c2Tbase,qTbase,q_Eu0,q_Eu1,q_Eu2,q_pix,q_angle0,q_angle1,q_angle2,q_angle3,q_angle4,q_angle5,finq1,wTq):
    names_1 = ['cTw'+str(i) for i in range(16)]
    #names_2 = ['wTq1'+str(i) for i in range(16)]
    names_2 = ['wTc'+str(i) for i in range(16)]
    #names_3 = ['wTq2'+str(i) for i in range(16)]
    names_3 = ['cTbase'+str(i) for i in range(16)]
    names_4 = ['wTbase'+str(i) for i in range(16)]
    names_5 = ['cTq'+str(i) for i in range(16)]
    names_6 = ['qTc'+str(i) for i in range(16)]
    names_7 = ['c2Tbase'+str(i) for i in range(16)]
    names_8 = ['qTbase'+str(i) for i in range(16)]
    names_9 = ['wTq'+str(i) for i in range(16)]
    #names =names_1 + names_2 + names_3 + ['finc']+ ['finq1']+ ['finq2']+ ['alpha']+ ['beta']+ ['gamma']
    names =names_1 + names_2 + names_3 + names_4  +['w_Eu0'] +['w_Eu1'] +['w_Eu2'] +['w_pix'] +['finc'] +['w_angle0'] +['w_angle1'] +['w_angle2'] +['w_angle3'] +['w_angle4'] +['w_angle5']+ names_5 + names_6 + names_7 + names_8 +['q_Eu0'] +['q_Eu1'] +['q_Eu2'] +['q_pix'] +['finq1'] +['q_angle0'] +['q_angle1'] +['q_angle2'] +['q_angle3'] +['q_angle4'] +['q_angle5'] + names_9
    scores =[]
    for i in range(4):#col
        for j in range(4):#rows
            scores.append(cTw[j,i])
    for i in range(4):#col
        for j in range(4):#rows
            scores.append(wTc[j,i])
    for i in range(4):#col
        for j in range(4):#rows
            scores.append(cTbase[j,i])
    for i in range(4):#col
        for j in range(4):#rows
            scores.append(wTbase[j,i])
    
    scores.append(w_Eu0)
    scores.append(w_Eu1)
    scores.append(w_Eu2)
    scores.append(w_pix)    
    scores.append(finc)
    scores.append(w_angle0)
    scores.append(w_angle1)
    scores.append(w_angle2)
    scores.append(w_angle3)
    scores.append(w_angle4)
    scores.append(w_angle5)
    
    for i in range(4):#col
        for j in range(4):#rows
            scores.append(cTq[j,i])
    for i in range(4):#col
        for j in range(4):#rows
            scores.append(qTc[j,i])
    for i in range(4):#col
        for j in range(4):#rows
            scores.append(c2Tbase[j,i])
    for i in range(4):#col
        for j in range(4):#rows
            scores.append(qTbase[j,i])
    
    scores.append(q_Eu0)
    scores.append(q_Eu1)
    scores.append(q_Eu2)
    scores.append(q_pix)
    scores.append(finq1)
    scores.append(q_angle0)
    scores.append(q_angle1)
    scores.append(q_angle2)
    scores.append(q_angle3)
    scores.append(q_angle4)
    scores.append(q_angle5)
    
    for i in range(4):#col
        for j in range(4):#rows
            scores.append(wTq[j,i])
    # for i in range(4):#col
    #     for j in range(4):#rows
    #         scores.append(wTq1[j,i])
    # for i in range(4):#col
    #     for j in range(4):#rows
    #         scores.append(wTq2[j,i])
    # for i in range(4):#col
    #      for j in range(4):#rows
    #          scores.append(cTw[j,i])
            
    
    
    
    
    # scores.append(finq2)
    # scores.append(alpha)
    # scores.append(beta)
    # scores.append(gamma)
    # scores.append(pix)
    
    mydict = {}
    for i in range(len(names)):
        mydict[names[i]] =[np.round(scores[i],6)] 
    return mydict
    
#def save_csv(path,wTcs,wTq1s,wTq2s,fincs,finq1s,finq2s,alphas,betas,gammas):
#def save_csv(path,wTcs,fincs,alphas,betas,gammas,pixs):
#def save_csv(path,wTcs,fincs,alphas,betas,gammas,pixs,cTws):
def save_csv(path,cTws,wTcs,cTbases,wTbases,w_Eu0s,w_Eu1s,w_Eu2s,w_pixs,w_angle0s,w_angle1s,w_angle2s,w_angle3s,w_angle4s,w_angle5s,cTqs,qTcs,c2Tbases,qTbases,wTqs,q_Eu0s,q_Eu1s,q_Eu2s,q_pixs,q_angle0s,q_angle1s,q_angle2s,q_angle3s,q_angle4s,q_angle5s,fincs,finq1s):
    #mydicts = create_format(wTcs[0],wTq1s[0],wTq2s[0],fincs[0],finq1s[0],finq2s[0],alphas[0],betas[0],gammas[0])
    #mydicts = create_format(wTcs[0],fincs[0],alphas[0],betas[0],gammas[0],pixs[0],cTws[0])
    mydicts = create_format(cTws[0],wTcs[0],cTbases[0],wTbases[0],w_Eu0s[0],w_Eu1s[0],w_Eu2s[0],w_pixs[0],w_angle0s[0],w_angle1s[0],w_angle2s[0],w_angle3s[0],w_angle4s[0],w_angle5s[0],fincs[0],cTqs[0],qTcs[0],c2Tbases[0],qTbases[0],q_Eu0s[0],q_Eu1s[0],q_Eu2s[0],q_pixs[0],q_angle0s[0],q_angle1s[0],q_angle2s[0],q_angle3s[0],q_angle4s[0],q_angle5s[0],finq1s[0],wTqs[0])
    
    for i in range(1,len(cTws)):
        #mydict = create_format(wTcs[i],wTq1s[i],wTq2s[i],fincs[i],finq1s[i],finq2s[i],alphas[i],betas[i],gammas[i])
        #mydict = create_format(wTcs[i],fincs[i],alphas[i],betas[i],gammas[i],pixs[i],cTws[i])
        mydict = create_format(cTws[i],wTcs[i],cTbases[i],wTbases[i],w_Eu0s[i],w_Eu1s[i],w_Eu2s[i],w_pixs[i],w_angle0s[i],w_angle1s[i],w_angle2s[i],w_angle3s[i],w_angle4s[i],w_angle5s[i],fincs[i],cTqs[i],qTcs[i],c2Tbases[i],qTbases[i],q_Eu0s[i],q_Eu1s[i],q_Eu2s[i],q_pixs[i],q_angle0s[i],q_angle1s[i],q_angle2s[i],q_angle3s[i],q_angle4s[i],q_angle5s[i],finq1s[i],wTqs[i])
        
        for k in mydicts.keys():
            mydicts[k].append(mydict[k][0])
    
    
    df = pd.DataFrame(mydicts)
    df.to_csv(path, index=False)

#順向矩陣運算
def rotation_x(theta):
    c = np.cos(theta)
    s = np.sin(theta)
    rtx = np.array([[  1, 0, 0, 0],
                      [0, c,-s, 0],
                      [0, s, c, 0],
                      [0, 0, 0, 1]])
    return rtx
    
def rotation_y(theta):
    c = np.cos(theta)
    s = np.sin(theta)
    rty = np.array([[  c, 0, s, 0],
                      [0, 1, 0, 0],
                      [-s, 0, c, 0],
                      [ 0, 0, 0, 1]])
    return rty
    
def rotation_z(theta):
    c = np.cos(theta)
    s = np.sin(theta)
    rtz = np.array([[  c,-s, 0, 0],
                     [ s, c, 0, 0],
                     [ 0, 0, 1, 0],
                     [ 0, 0, 0, 1]])
    return rtz

def translate_x(dx):
    x = np.array([[  1, 0, 0, dx],
                    [0, 1, 0, 0],
                    [0, 0, 1, 0],
                    [0, 0, 0, 1]])
    return x

def translate_y(dy):
    y = np.array([[  1, 0, 0, 0],
                    [0, 1, 0, dy],
                    [0, 0, 1, 0],
                    [0, 0, 0, 1]])
    return y

def translate_z(dz):
    z = np.array(  [[1, 0, 0, 0],
                    [0, 1, 0, 0],
                    [0, 0, 1, dz],
                    [0, 0, 0, 1]])
    return z
#順向運動學define
def fkmatrix(angle_1, angle_2, angle_3, angle_4, angle_5, angle_6):
   #原始角度
   theta = [np.deg2rad(angle_1),np.deg2rad(angle_2),np.deg2rad(angle_3),np.deg2rad(angle_4),np.deg2rad(angle_5),np.deg2rad(angle_6),np.deg2rad(90),np.deg2rad(180)]
   #加入下垂角度    
   #theta = [np.deg2rad(angle_1),np.deg2rad(angle_2),np.deg2rad(angle_3),np.deg2rad(angle_4),np.deg2rad(angle_5),np.deg2rad(angle_6),np.deg2rad(90),np.deg2rad(180),np.deg2rad(-3)]

   #原始
   # dx =[6]
   # dy = [9, 6]
   # dz = [24 , 22.5 ,10, 5, 9, 4]

   dx =[8]
   dy = [8, 6]
   dz = [24, 22.5,10 ,5.5 ,10, 3.5]

   translate_x(dx[0])
   t0 = translate_x(dx[0])

   translate_z(dz[0])
   t1 = translate_z(dz[0])

   translate_y(dy[0])
   t2 = translate_y(dy[0])

   rotation_y(theta[0])
   t3 = rotation_y(theta[0])

   #Id1 y軸下垂 旋轉角度
   #rotation_y(theta[8])
   #t17 = rotation_y(theta[8])

   translate_y(dy[1])
   t4 = translate_y(dy[1])

   rotation_x(theta[1])
   t5 = rotation_x(theta[1])
   
   #共軸位移
   # translate_z(dz[6])
   # t18 = translate_z(dz[6])
      
   rotation_z(theta[2])
   t6 = rotation_z(theta[2])

   translate_z(dz[1])
   t7 = translate_z(dz[1])

   rotation_y(theta[3])
   t8 = rotation_y(theta[3])

   translate_z(dz[2])
   t9 = translate_z(dz[2])

   rotation_z(theta[4])
   t10 = rotation_z(theta[4])

   translate_z(dz[3])
   t11 = translate_z(dz[3])

   rotation_x(theta[5])
   t12 = rotation_x(theta[5])

   translate_z(dz[4])
   t13 = translate_z(dz[4])

   rotation_x(theta[6])
   t14 = rotation_x(theta[6])

   rotation_z(theta[7])
   t15 = rotation_z(theta[7])

   translate_z(dz[5])
   t16 = translate_z(dz[5])

   #原始
   baseTc = t0@t1@t2@t3@t4@t5@t6@t7@t8@t9@t10@t11@t12@t13@t14@t15@t16
   #加入下垂角度
   #baseTc = t0@t1@t2@t3@t17@t4@t5@t6@t7@t8@t9@t10@t11@t12@t13@t14@t15@t16

   np.set_printoptions(suppress=True)

   print('baseTc \n' , np.round(baseTc,3))

   cTbase = np.linalg.inv(baseTc)
   print('cTbase \n' , np.round(cTbase,3))

   return cTbase


   
def detect():
    global packetHandler,portHandler, DXL1_ID, DXL2_ID, DXL3_ID, DXL4_ID, DXL5_ID, DXL6_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE
    # 選擇攝影機
    FOV = 90
    width =  1920
    heigh = 1080
    r = pow(pow(width/2.0,2)+pow(heigh/2.0,2),0.5)
    z = r/np.tan(FOV*np.pi/180.0/2.0)
    cap = cv2.VideoCapture(0+cv2.CAP_DSHOW)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, heigh)
    cap.set(cv2.CAP_PROP_SETTINGS, 0)
    # Q12 = 6.0*6.0 
    # Q13 = 6.0*6.0
    # Q23 = 6.0*6.0 + 6.0*6.0 #cm
    qrlength = 5.0
    
    Q12 = qrlength*qrlength 
    Q13 = qrlength*qrlength
    Q23 = qrlength*qrlength + qrlength*qrlength #cm
    
    
    
    Zs={} 
    Z1 = 20
    Z2 = 20
    Z3 = 20
    
    names=["Point_01","Point_02"]
    
    for name in names:
        Zs[name]=[Z1,Z2,Z3]
    qrdetector = cv2.QRCodeDetector()
    qrdata={}
    
    wTc=np.array([])
    
    wTq1 = np.eye(4)
    wTq2 = np.eye(4)
    cTw = np.eye(4)
    # cTw = np.array([[0,1,,]],
    #                 [0.707,0,-c[1],p0[1]],
    #                 [-0.707,b[2],c[2],20],
    #                 [   0,   0,   0,  1]])
    
    finc = 0
    # finq1 = 0
    # finq2 = 0
    alpha = 0
    beta = 0
    gamma = 0
    pix = 0
    finq1 = 0
    # wTcs =[]
    # wTq1s=[]
    # wTq2s=[]
    # fincs =[]
    # finq1s =[]
    # finq2s =[]
    # alphas =[]
    # betas =[]
    # gammas=[]
    # pixs=[]
    cTws=[]
    wTcs=[]
    cTbases=[]
    wTbases=[]
    w_Eu0s=[] 
    w_Eu1s=[]
    w_Eu2s=[]
    w_pixs=[]
    w_angle0s=[]
    w_angle1s=[]
    w_angle2s=[]
    w_angle3s=[]
    w_angle4s=[]
    w_angle5s=[]
    cTqs=[]
    qTcs=[]
    c2Tbases=[]
    qTbases=[]
    wTqs=[]
    q_Eu0s=[] 
    q_Eu1s=[]
    q_Eu2s=[]
    q_pixs=[]
    q_angle0s=[]
    q_angle1s=[]
    q_angle2s=[]
    q_angle3s=[]
    q_angle4s=[]
    q_angle5s=[]
    fincs=[]
    finq1s=[]
    
    ans={}
    ans['cTw'] = np.eye(4)
    ans['wTc'] = np.eye(4)
    ans['cTbase'] = np.eye(4)
    ans['wTbase'] = np.eye(4)
    ans['cTq'] = np.eye(4)
    ans['qTc'] = np.eye(4)
    ans['c2Tbase'] = np.eye(4)
    ans['qTbase'] = np.eye(4)
    ans['wTq'] = np.eye(4)
    ans['w_Eu']=(0,0,0)
    ans['q_Eu']=(0,0,0)
    ans['w_pixels']=0
    ans['q_pixels']=0
    ans['w_angle']=(0,0,0,0,0,0)
    ans['q_angle']=(0,0,0,0,0,0)
    
    while(True):
        # 從攝影機擷取一張影像
        ret, frame = cap.read()
        #cv2.imshow('frame', frame)
        t = datetime.now()
        rv,datas,bboxs,rectifiedImage= qrdetector.detectAndDecodeMulti(frame)
        dt = datetime.now()-t
        #print("qrdetector ",dt.microseconds)
        
        # im = display(frame, pts[0])
        # im = display(im, pts[1])
        # data,bbox,rectifiedImage = qrdetector.detectAndDecode(frame)
        if rv == True:
            cTq_={}
            is_find={}
            for i in range(len(names)):
                is_find[names[i]]=False
            cTq_[names[0]] =None
            count =0
            t = datetime.now()
            for i in range(len(datas)):
                print("Decoded Data : {}".format(datas[i]))
                bbox = np.array( bboxs[i],dtype='int')
                im = display(frame, bbox)
                p0 = bbox[0]
                p1 = bbox[3]
                p2 = bbox[1]
                p3 = bbox[2]
                pc = (p0+p1+p2+p3)/4 #QRcode中心點座標
                cv2.circle(im,p0,10,(0,255,0),2);
                cv2.circle(im,p1,10,(0,0,255),2);
                cv2.circle(im,p2,10,(255,0,0),2);
                #cv2.circle(im,p3,10,(255,255,255),2);
                #cv2.imshow("Results", im)
                rectifiedImage = np.uint8(rectifiedImage);
                #cv2.imshow("Rectified QRCode", rectifiedImage);
                
                if datas[i] !='':
                    is_find[datas[i]]=True
                    count+=1
                    cTq0,cTq1,cTq2,fin,fin_len = ipm(p0,p1,p2,width,heigh,Q12,Q23,Q13,z,Zs[datas[i]][0],Zs[datas[i]][1],Zs[datas[i]][2])
                    
                    
                    # pixel = (p1-p0)
                    pixel = (p2-p1)
                    #d = (pixel[0]**2+pixel[1]**2)**0.5
                    d = pow(pow(pixel[0],2)+pow(pixel[1],2),0.5)
                    #print("Decoded ",datas[i]," pixel " , pixel," D ",d)
                    qrdata[datas[i]] =np.round(d,3)
                    cTq_[datas[i]] = [cTq0,cTq1,cTq2,p1,p2,fin,np.round(d,3),fin_len]
                    #p1、p2轉換成wTp1、wTp2
                    #wTp1 = wTc * p1
                    
                    #1個pixel角度
                    #view = ((p2-wTc)*(p1-wTc))/((pow(abs(p2-wTc),0.5))*(pow(abs(p1-wTc),0.5)))
                    #angle = acos(view)
                    #一個pixel占多少角度
                    #(pow(pow(p2,2)+pow(p1,2),0.5))/angle
                    
            dt = datetime.now()-t
            #print("IPMs ", dt.microseconds)
            #cv2.imshow('frame', im)
            #print(qrdata)
            #print("pixel: ", qrdata)
            #print("pixel: ", np.round(d,3))
            #print("pixel: ",np.round(qrdata,3))
            if len(wTc)!=0:
                print('cTw \n' , np.round(cTw,3))
                print('wTc \n',np.round(wTc,3)) #ans
                
                
                
            for i in range(len(is_find)):
                if is_find[names[i]]==True:
                    if names[i]==names[0]:
                        cTw = cTq_[names[0]][0]
                        fin = cTq_[names[0]][5]
                        pix = cTq_[names[0]][6]
                        fin_len = cTq_[names[0]][7]
                        wTc =np.linalg.inv(cTw)
                        pixelangle,angle = Getpixel(wTc,cTq_[names[0]][1],cTq_[names[0]][2],cTq_[names[0]][3],cTq_[names[0]][4])
                        #print("Decoded Data :" , datas[i])
                        #print('QR0: ',' pixel:', pix,' angle', np.round(angle,3),' pixelangle', np.round(pixelangle,3), ' fin:', np.round(fin,3))
                        alpha,beta,gamma = GetEulaDeg(wTc)
                        print('wTc',i,'x alpha ',np.round(alpha,3))
                        print('wTc',i,'y beta  ',np.round(beta,3))
                        print('wTc',i,'z gamma ',np.round(gamma,3))
                        print('QR0: ',' pixel:', pix,' fin:', np.round(fin,6),' fin_len:', np.round(fin_len,6))
                        finc = fin
                        angle_1, angle_2, angle_3, angle_4, angle_5, angle_6 = readangle()
                        #順向運動學得到cTbase
                        cTbase = fkmatrix(angle_1, angle_2, angle_3, angle_4, angle_5, angle_6)
                        #wTc乘上cTbase得到wTbase    
                        wTbase = wTc@cTbase
                        print('wTbase \n',np.round(wTbase,3))
                        ans['cTw']= cTw
                        ans['wTc']= wTc
                        ans['cTbase']= cTbase
                        ans['wTbase']= wTbase
                        ans['w_Eu']=(alpha,beta,gamma)
                        ans['w_pixels']=pix
                        ans['w_angle']=(angle_1, angle_2, angle_3, angle_4, angle_5, angle_6)
                        
                    elif names[i]==names[1] or names[i]==names[2]:
                        if len(wTc)!=0:
                            cTqn = cTq_[names[i]][0]
                            fin = cTq_[names[i]][5]
                            pix = cTq_[names[i]][6]
                            wTqn = wTc @ cTqn
                            pixelangle,angle = Getpixel(wTc,cTq_[names[i]][1],cTq_[names[i]][2],cTq_[names[i]][3],cTq_[names[i]][4])
                            print("Decoded Data :" , datas[0])
                            print("wTq",i, "\n",np.round(wTqn,3))
                            alpha,beta,gamma = GetEulaDeg(wTqn)
                            print('q',i,'x alpha ',np.round(alpha,3))
                            print('q',i,'y beta  ',np.round(beta,3))
                            print('q',i,'z gamma ',np.round(gamma,3))
                            #print('QR',i,':' , ' pixel:', pix,' angle', np.round(angle,3),' pixelangle', np.round(pixelangle,3), ' fin', np.round(fin,3))
                            print('QR',i,':' , ' pixel:', pix,' fin:', np.round(fin,6),' fin_len:', np.round(fin_len,6))
                            if i ==1:
                                wTq1 = wTqn
                                finq1 =fin
                            if i ==2:
                                wTq2 = wTqn
                                finq2 =fin
                            #angle_1, angle_2, angle_3, angle_4, angle_5, angle_6 = readangle()
                            #順向運動學得到c2Tbase
                            #c2Tbase = fkmatrix(angle_1, angle_2, angle_3, angle_4, angle_5, angle_6)
                            #wTc乘上cTbase得到wTbase   
                            qTc = np.linalg.inv(cTqn)
                            #qTbase = qTc@c2Tbase
                            #wTq = wTbase @ np.linalg.inv(qTbase)
                            
                            ans['cTq'] = cTqn
                            ans['qTc'] = qTc
                            #ans['c2Tbase'] = c2Tbase
                            #ans['qTbase'] = qTbase
                            ans['wTq']= wTqn
                            ans['q_Eu']=(alpha,beta,gamma)
                            ans['q_pixels']=pix
                            #ans['q_angle']=(angle_1, angle_2, angle_3, angle_4, angle_5, angle_6)
            
            cTws.append(ans['cTw'].copy())
            wTcs.append(ans['wTc'].copy())
            cTbases.append(ans['cTbase'].copy())
            wTbases.append(ans['wTbase'].copy())
            w_Eu0s.append(ans['w_Eu'][0]) 
            w_Eu1s.append(ans['w_Eu'][1])
            w_Eu2s.append(ans['w_Eu'][2])
            w_pixs.append(ans['w_pixels'])
            w_angle0s.append(ans['w_angle'][0])
            w_angle1s.append(ans['w_angle'][1])
            w_angle2s.append(ans['w_angle'][2])
            w_angle3s.append(ans['w_angle'][3])
            w_angle4s.append(ans['w_angle'][4])
            w_angle5s.append(ans['w_angle'][5])
            cTqs.append(ans['cTq'].copy())
            qTcs.append(ans['qTc'].copy())
            c2Tbases.append(ans['c2Tbase'].copy())
            qTbases.append(ans['qTbase'].copy())
            wTqs.append(ans['wTq'].copy())
            q_Eu0s.append(ans['q_Eu'][0]) 
            q_Eu1s.append(ans['q_Eu'][1])
            q_Eu2s.append(ans['q_Eu'][2])
            q_pixs.append(ans['q_pixels'])
            q_angle0s.append(ans['q_angle'][0])
            q_angle1s.append(ans['q_angle'][1])
            q_angle2s.append(ans['q_angle'][2])
            q_angle3s.append(ans['q_angle'][3])
            q_angle4s.append(ans['q_angle'][4])
            q_angle5s.append(ans['q_angle'][5])
            fincs.append(finc)
            finq1s.append(finq1)
            
            # pixs.append(pix)
            # wTcs.append(wTc)
            # fincs.append(finc)
            # wTq1s.append(wTq1)
            # finq1s.append(finq1)
            # wTq2s.append(wTq2)
            # finq2s.append(finq2) 
            # alphas.append(alpha)
            # betas.append(beta)
            # gammas.append(gamma)  
            # pixs.append(pix)
            # cTws.append(cTw)            
            
        
        # 顯示圖片
        cv2.imshow('frame', frame)
        #cv2.imshow('frame', im)
        # 若按下q鍵則離開迴圈
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        if key == ord('s'):
            #save
            now = datetime.now()
            date_time = now.strftime("_%m_%d_%Y_%H_%M_%S")
            #path = 'C:/user/'
            #path = 'D:/researchdata/output_data'
            name = 'D:/researchdata5/output_data' + date_time + '.csv'
            #name = 'output_data' + date_time + '.csv'
            # save_csv(name,wTcs,wTq1s,wTq2s,fincs,finq1s,finq2s,alphas,betas,gammas)
            # save_csv(name,wTcs,fincs,alphas,betas,gammas,pixs,cTws)
            save_csv(name,cTws,wTcs,cTbases,wTbases,w_Eu0s,w_Eu1s,w_Eu2s,w_pixs,w_angle0s,w_angle1s,w_angle2s,w_angle3s,w_angle4s,w_angle5s,cTqs,qTcs,c2Tbases,qTbases,wTqs,q_Eu0s,q_Eu1s,q_Eu2s,q_pixs,q_angle0s,q_angle1s,q_angle2s,q_angle3s,q_angle4s,q_angle5s,fincs,finq1s)
            cTws=[]
            wTcs=[]
            cTbases=[]
            wTbases=[]
            w_Eu0s=[] 
            w_Eu1s=[]
            w_Eu2s=[]
            w_pixs=[]
            w_angle0s=[]
            w_angle1s=[]
            w_angle2s=[]
            w_angle3s=[]
            w_angle4s=[]
            w_angle5s=[]
            cTqs=[]
            qTcs=[]
            c2Tbases=[]
            qTbases=[]
            wTqs=[]
            q_Eu0s=[] 
            q_Eu1s=[]
            q_Eu2s=[]
            q_pixs=[]
            q_angle0s=[]
            q_angle1s=[]
            q_angle2s=[]
            q_angle3s=[]
            q_angle4s=[]
            q_angle5s=[]
            fincs=[]
            finq1s=[]
            # wTcs =[]
            # wTq1s=[]
            # wTq2s=[]
            # fincs =[]
            # finq1s =[]
            # finq2s =[]
            # alphas =[]
            # betas =[]
            # gammas =[]
            # pixs =[]
            # cTws =[]
        if key == ord('y'): #鎖住馬達
            # Enable Dynamixel#1 Torque
            packetHandler.write1ByteTxRx(portHandler, DXL1_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE)
            # Enable Dynamixel#2 Torque
            packetHandler.write1ByteTxRx(portHandler, DXL2_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE)
            # Enable Dynamixel#3 Torque
            packetHandler.write1ByteTxRx(portHandler, DXL3_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE)
            # Enable Dynamixel#4 Torque
            packetHandler.write1ByteTxRx(portHandler, DXL4_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE)
            # Enable Dynamixel#5 Torque
            packetHandler.write1ByteTxRx(portHandler, DXL5_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE)
            # Enable Dynamixel#6 Torque
            packetHandler.write1ByteTxRx(portHandler, DXL6_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE)
            
        if key == ord('n'): #解鎖馬達
            # Disable Dynamixel#1 Torque
            packetHandler.write1ByteTxRx(portHandler, DXL1_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE)
            # Disable Dynamixel#2 Torque
            packetHandler.write1ByteTxRx(portHandler, DXL2_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE)
            # Disable Dynamixel#3 Torque
            packetHandler.write1ByteTxRx(portHandler, DXL3_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE)
            # Disable Dynamixel#4 Torque
            packetHandler.write1ByteTxRx(portHandler, DXL4_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE)
            # Disable Dynamixel#5 Torque
            packetHandler.write1ByteTxRx(portHandler, DXL5_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE)
            # Disable Dynamixel#6 Torque
            packetHandler.write1ByteTxRx(portHandler, DXL6_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE)
            
    # 釋放攝影機
    cv2.waitKey(10)
    cap.release()
    # 關閉所有 OpenCV 視窗
    cv2.destroyAllWindows()
    portHandler.closePort()
if __name__ == '__main__':
    detect()