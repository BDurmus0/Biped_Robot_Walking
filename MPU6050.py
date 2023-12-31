import serial
import time

ACCData=[0.0]*8
GYROData=[0.0]*8
AngleData=[0.0]*8          
FrameState = 0           
Bytenum = 0              
CheckSum = 0                     
roll = pitch = yaw =   0.0
acc_x = 0.0
acc_y = 0
acc_z = 0
gyro_x = 0
gyro_y = 0
gyro_z = 0

a = [0.0]*3
w = [0.0]*3
Angle = [0.0]*3

# global exit_flag
# exit_flag = 0

## Define a function to convert raw data into suitable format

def DueData(inputdata):   
    global  FrameState    
    global  Bytenum
    global  CheckSum
    global  a
    global  w
    global  Angle , roll , pitch , yaw ,acc_x,acc_y,acc_z,gyro_x,gyro_y,gyro_z


    for data in inputdata:  

        
        if FrameState==0: 
            if data==0x55 and Bytenum==0: 
                CheckSum=data
                Bytenum=1
                continue
            elif data==0x51 and Bytenum==1:
                CheckSum+=data
                FrameState=1
                Bytenum=2
            elif data==0x52 and Bytenum==1: 
                CheckSum+=data
                FrameState=2
                Bytenum=2
            elif data==0x53 and Bytenum==1:
                CheckSum+=data
                FrameState=3
                Bytenum=2

        elif FrameState==1: # acc    
            
            if Bytenum<10:          
                ACCData[Bytenum-2]=data # 从0开始
                CheckSum+=data
                Bytenum+=1
            else:
                if data == (CheckSum&0xff):  
                    # a = get_acc(ACCData)
                    acc_x,acc_y,acc_z = get_acc(ACCData)
                CheckSum=0                  
                Bytenum=0
                FrameState=0

        elif FrameState==2: # gyro
            
            if Bytenum<10:
                GYROData[Bytenum-2]=data
                CheckSum+=data
                Bytenum+=1
            else:
                if data == (CheckSum&0xff):
                    # w = get_gyro(GYROData)
                    gyro_x,gyro_y,gyro_z = get_gyro(GYROData)
                CheckSum=0
                Bytenum=0
                FrameState=0

        elif FrameState==3: # angle
            
            if Bytenum<10:
                AngleData[Bytenum-2]=data
                CheckSum+=data
                Bytenum+=1
            else:
                if data == (CheckSum&0xff):
                    # Angle = get_angle(AngleData)
                    roll,pitch,yaw = get_angle(AngleData)
                    # d = a+w+Angle
                    # print("a(g):%10.3f %10.3f %10.3f w(deg/s):%10.3f %10.3f %10.3f Angle(deg):%10.3f %10.3f %10.3f"%d)
                CheckSum=0
                Bytenum=0
                FrameState=0
    #return roll,pitch,yaw
    return acc_x,acc_y,acc_z,gyro_x,gyro_y,gyro_z,roll,pitch,yaw
 
 
def get_acc(datahex):  
    axl = datahex[0]                                        
    axh = datahex[1]
    ayl = datahex[2]                                        
    ayh = datahex[3]
    azl = datahex[4]                                        
    azh = datahex[5]
    
    k_acc = 16.0
 
    acc_x = (axh << 8 | axl) / 32768.0 * k_acc
    acc_y = (ayh << 8 | ayl) / 32768.0 * k_acc
    acc_z = (azh << 8 | azl) / 32768.0 * k_acc

    if acc_x >= k_acc:
        acc_x -= 2 * k_acc
    if acc_y >= k_acc:
        acc_y -= 2 * k_acc
    if acc_z >= k_acc:
        acc_z-= 2 * k_acc
    
    return acc_x,acc_y,acc_z
 
 
def get_gyro(datahex):                                      
    wxl = datahex[0]                                        
    wxh = datahex[1]
    wyl = datahex[2]                                        
    wyh = datahex[3]
    wzl = datahex[4]                                        
    wzh = datahex[5]
    k_gyro = 2000.0
 
    gyro_x = (wxh << 8 | wxl) / 32768.0 * k_gyro
    gyro_y = (wyh << 8 | wyl) / 32768.0 * k_gyro
    gyro_z = (wzh << 8 | wzl) / 32768.0 * k_gyro

    if gyro_x >= k_gyro:
        gyro_x -= 2 * k_gyro
    if gyro_y >= k_gyro:
        gyro_y -= 2 * k_gyro
    if gyro_z >=k_gyro:
        gyro_z-= 2 * k_gyro

    return gyro_x,gyro_y,gyro_z
 
 
def get_angle(datahex):                                 
    rxl = datahex[0]                                        
    rxh = datahex[1]
    ryl = datahex[2]                                        
    ryh = datahex[3]
    rzl = datahex[4]                                        
    rzh = datahex[5]
    k_angle = 180.0
 
    angle_x = (rxh << 8 | rxl) / 32768.0 * k_angle
    angle_y = (ryh << 8 | ryl) / 32768.0 * k_angle
    angle_z = (rzh << 8 | rzl) / 32768.0 * k_angle

    if angle_x >= k_angle:
        angle_x -= 2 * k_angle
    if angle_y >= k_angle:
        angle_y -= 2 * k_angle
    if angle_z >=k_angle:
        angle_z-= 2 * k_angle
 
    return angle_x,angle_y,angle_z
  




if __name__ == 'main':
    DueData()