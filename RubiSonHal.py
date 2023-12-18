import serial,time,threading,time
import numpy as np
import pandas as pd 
from dynamixel_sdk import *
from MPU6050 import DueData
from tkinter import *
import matplotlib.pyplot as plt
import pyfirmata
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

ADDR_RX28_GOAL_POS = 30
ADDR_RX28_PRESENT_POS = 36
ADDR_RX28_MOVING_STATUS = 46
ADDR_RX28_ALARM_LED = 17
ADDR_RX28_CW_ANGLE_LIMIT = 6
ADDR_RX28_CCW_ANGLE_LIMIT = 8
ADDR_RX28_MOVING_SPEED = 32
ADDR_RX28_PRESENT_SPEED = 38
ADDR_RX28_PRESENT_LAOD = 40
ADDR_RX28_PRESENT_TEMPRATURE = 43
ADDR_RX28_MAX_TORQUE = 14
ADDR_RX28_CW_COMPLIANCE_SLOPE = 28
ADDR_RX28_CCW_COMPLIANCE_SLOPE = 29
ADDR_RX28_TORQUE_ENABLE = 24
LEN_MX_GOAL_POSITION = 4

TORQUE_ENABLE = 1
TORQUE_DISABLE = 0
ALL_MOTORS = 254
DYNAMIXEL_ID = [1,2,3,4,5,6,7,8,9,10,11,12]

STAND_UPRIGTH_ANGLE_org = [530, 496, 484 ,497, 530, 505, 500, 490, 540, 495, 504, 520]
STAND_UPRIGTH_ANGLE = [529, 500, 484 ,492, 530, 505, 500, 490, 540, 495, 504, 520]

BREAK_KNEE = [525 ,490 ,500 ,490 ,525, 485 ,490 ,475, 540 ,495 ,505 ,520]
WALKING_START_ANGLE = [525 ,490 ,500 ,490 ,525, 485 ,490 ,475, 540 ,495 ,505 ,520]
yb= [525 ,490, 500-30/0.293 ,490-20/0.293 ,530-40/0.293 ,505-43/0.293 ,485-11/0.293 ,485-23/0.293 ,540 ,495 ,505, 520]


MAPPING_VALUE_FOR_DYNAMIXEL = 0.293
BAUDRATE_DYNAMIXEL = 1_000_000
BAUDRATE_MPU6050 = 115200
PROTOCOL_VERSION = 1

COM_PORT_MPU6050_BLUETOOTH = 'COM6'
COM_PORT_DYNAMIEL = 'COM4'
COM_PORT_ARDUINO= 'COM15'

TIME_OUT_VALUE = 0.5

roll = 0.0
pitch = 0.0
yaw = 0.0
acc_x = 0.0
acc_y = 0
acc_z = 0
gyro_x = 0
gyro_y = 0
gyro_z = 0

sum_of_x = 0
sum_of_y = 0

CoP_x_store = []
CoP_y_store = []
CoP_x_value = 0.0
CoP_y_value = 0.0
Copx_list = []
Copy_list = []

trajectory_file = pd.read_excel(r'C:\Users\kullanici\Desktop\RUBI FILES\50mmfull.xls')
trajectory_array = (np.array(trajectory_file)/MAPPING_VALUE_FOR_DYNAMIXEL)

ser = serial.Serial(COM_PORT_MPU6050_BLUETOOTH,BAUDRATE_MPU6050)
portHandler = PortHandler(COM_PORT_DYNAMIEL)

packetHandler = PacketHandler(PROTOCOL_VERSION)
portHandler.openPort()
portHandler.setBaudRate(BAUDRATE_DYNAMIXEL)

f1 = 0
f2 = 0
f3 = 0
f4 = 0
f5 = 0
f6 = 0
f7 = 0
f8 = 0
roll_control = 0.0
pitch_control = 0.0
output_31 = 0
output_32 = 0
output_4 = 0

integral = 0
prev_error = 0


integral_p = 0
prev_error_p = 0

integral_copy = 0
prev_error_copy = 0

integral_copx = 0
prev_error_copx = 0

time_array=[]
roll_array = []
pitch_array = []
Copx_list = []
Copy_list = []
Kp = 0.2    #0.3
Ki = 0.1    #0.1
Kd = 0.5    #0.4

board = pyfirmata.ArduinoNano(COM_PORT_ARDUINO)
it = pyfirmata.util.Iterator(board)
it.start()

port_pin = board.get_pin('a:0:i')
port_pin_1 = board.get_pin('a:1:i')
port_pin_2 = board.get_pin('a:2:i')
port_pin_3 = board.get_pin('a:3:i')
port_pin_4 = board.get_pin('a:4:i')
port_pin_5 = board.get_pin('a:5:i')
port_pin_6 = board.get_pin('a:6:i')
port_pin_7 = board.get_pin('a:7:i')


master = Tk()
master.title('RUBY Graphic Unit Interface')
master.geometry('850x600')

control = IntVar()
control.set(0)

calibration = IntVar()
calibration.set(0)

def control_button() -> int:
    Flag = control.get()
    return Flag

def calib_button() -> int:
    Calibration_flag = calibration.get()
    return Calibration_flag


def PID(Kp,Ki,Kd,EulerAngle):
    global integral,prev_error
    error = 0 - int(EulerAngle)
    integral += error
    derivative = error - prev_error
    prev_error = error
    return Kp*error + Ki*integral + Kd*derivative

def PID_P(Kp_p,Ki_p,Kd_p,EulerAngle_p):
    global integral_p,prev_error_p
    error = 0 - int(EulerAngle_p)
    integral_p += error
    derivative = error - prev_error_p
    prev_error_p = error
    return Kp_p*error + Ki_p*integral_p + Kd_p*derivative


def Euler():
    global acc_x,acc_y,acc_z,gyro_x,gyro_y,gyro_z,roll,pitch,yaw,roll_array,pitch_array,time_array
    while True:
        
        datahex = ser.read(33)
        acc_x,acc_y,acc_z,gyro_x,gyro_y,gyro_z,roll,pitch,yaw = DueData(datahex)
        roll_array.append(roll)
        pitch_array.append(pitch)
        #print(f"Roll: {int(roll):<30} pitch: {int(pitch):<30} yaw: {yaw:<30} accx :{acc_y:<30} gyro_x : {gyro_x:<30}")
      
def justControlFunc():
    packetHandler.write2ByteTxRx(portHandler,ALL_MOTORS,48,2)
    global roll_control, pitch_control , output_31 , output_32 , output_4
    
    while True:
        if calib_button():
                m7 , _ , _ = packetHandler.read2ByteTxRx(portHandler,DYNAMIXEL_ID[6],ADDR_RX28_PRESENT_POS)
                time.sleep(0.05)
                m8 , _ , _ = packetHandler.read2ByteTxRx(portHandler,DYNAMIXEL_ID[7],ADDR_RX28_PRESENT_POS)
                time.sleep(0.05)
                m11 , _ , _ = packetHandler.read2ByteTxRx(portHandler,DYNAMIXEL_ID[10],ADDR_RX28_PRESENT_POS)
                time.sleep(0.05)
                m12 , _ , _ = packetHandler.read2ByteTxRx(portHandler,DYNAMIXEL_ID[11],ADDR_RX28_PRESENT_POS)
                time.sleep(0.05)
                m1 , _ , _ = packetHandler.read2ByteTxRx(portHandler,DYNAMIXEL_ID[0],ADDR_RX28_PRESENT_POS)
                time.sleep(0.05)
                m2 , _ , _ = packetHandler.read2ByteTxRx(portHandler,DYNAMIXEL_ID[1],ADDR_RX28_PRESENT_POS)
                time.sleep(0.05)
                m3 , _ , _ = packetHandler.read2ByteTxRx(portHandler,DYNAMIXEL_ID[2],ADDR_RX28_PRESENT_POS)
                time.sleep(0.05)
                m4 , _ , _ = packetHandler.read2ByteTxRx(portHandler,DYNAMIXEL_ID[3],ADDR_RX28_PRESENT_POS)
            
                packetHandler.write2ByteTxRx(portHandler, ALL_MOTORS, ADDR_RX28_MOVING_SPEED, 150)
                packetHandler.write2ByteTxRx(portHandler, 1, ADDR_RX28_MOVING_SPEED, 150)
                packetHandler.write2ByteTxRx(portHandler, 2, ADDR_RX28_MOVING_SPEED, 150)
                packetHandler.write2ByteTxRx(portHandler, 11, ADDR_RX28_MOVING_SPEED, 150)
                packetHandler.write2ByteTxRx(portHandler, 12, ADDR_RX28_MOVING_SPEED, 150)
                packetHandler.write2ByteTxRx(portHandler, 3, ADDR_RX28_MOVING_SPEED, 300)
                packetHandler.write2ByteTxRx(portHandler, 4, ADDR_RX28_MOVING_SPEED, 300)
                packetHandler.write2ByteTxRx(portHandler, 7, ADDR_RX28_MOVING_SPEED, 300)
                packetHandler.write2ByteTxRx(portHandler, 8, ADDR_RX28_MOVING_SPEED, 300)

                if m3 | m4 |  m7 | m8 == 0 :
                    print("M3 M4 M7 or M8 did not read !!!!")
                else: 
                    print(f"m1 : {m1} m2 : {m2} m3 : {m3} m4 :{m4} m7: {m7}  m8: {m8} m11 : {m11} m1 : {m12}")
                    print("Calibration Process is done !!!!!!")
                    print("Please remove the sign !!!")

                time.sleep(1)



        
        
        if control_button():

            roll_control = PID(0.2 ,0.1 ,0.5,roll) #0.2 0.1 0.5
            pitch_control = PID_P(1,0.1,0.6,pitch)
            output_3= 3 #PID_Copy(0.09,0.02,0.5,dimention2angle)
            
            output_31 = output_3
            output_32 = output_3
            
            output_4 = 3 #PID_Copx(0.09,0.02,0.5,dimention2angle2)

            if 20 <= CoP_x_value <= 42 or -42 <= CoP_x_value <= -20:
                output_4 = output_4
            else:
                output_4 = 0

            if 70<=CoP_y_value <= 77 or (43-7)<=CoP_y_value<=43 :  
                output_32 = 0
            elif -77<=CoP_y_value<=-70 or -43<=CoP_y_value<=(-43+7):
                output_31 = 0
            elif  5<=CoP_y_value<=15 or -15<=CoP_y_value<=-5:
                output_31 = 3
                output_32 = 3
            else:
                output_31 = 0
                output_32 = 0
            if (-3 < roll < 3):  roll_control = 0
            if (-3 < pitch < 3): pitch_control = 0
               
        
            #packetHandler.write2ByteTxRx(portHandler,DYNAMIXEL_ID[0],ADDR_RX28_GOAL_POS, m1 - int((output_31)/0.293))
            #packetHandler.write2ByteTxRx(portHandler,DYNAMIXEL_ID[1],ADDR_RX28_GOAL_POS, m2 + int((output_32)/0.293))
            #packetHandler.write2ByteTxRx(portHandler,DYNAMIXEL_ID[10],ADDR_RX28_GOAL_POS, m11 + int((pitch_control)/0.293))
            #packetHandler.write2ByteTxRx(portHandler,DYNAMIXEL_ID[11],ADDR_RX28_GOAL_POS, m12 + int((pitch_control )/0.293))
            #packetHandler.write2ByteTxRx(portHandler,DYNAMIXEL_ID[2],ADDR_RX28_GOAL_POS, m3 + int((roll_control + output_4)/0.293))
            #packetHandler.write2ByteTxRx(portHandler,DYNAMIXEL_ID[3],ADDR_RX28_GOAL_POS, m4 + int((roll_control + output_4)/0.293))
            #packetHandler.write2ByteTxRx(portHandler,DYNAMIXEL_ID[6],ADDR_RX28_GOAL_POS, m7 + int((roll_control )/0.293))
            #packetHandler.write2ByteTxRx(portHandler,DYNAMIXEL_ID[7],ADDR_RX28_GOAL_POS, m8 + int((roll_control )/0.293))
            time.sleep(0.05)
            


def CoP(f1,f2,f3,f4,f5,f6,f7,f8):
    
    if f1 == None: f1 = 0.0
    if f2 == None: f2 = 0.0
    if f3 == None: f3 = 0.0
    if f4 == None: f4 = 0.0
    if f5 == None: f5 = 0.0
    if f6 == None: f6 = 0.0
    if f7 == None: f7 = 0.0
    if f8 == None: f8 = 0.0
        
    lftop = f1 + f2 + f3 + f4
    if lftop == 0: 
        lftop = 0.01
    
    rftop = f5 + f6 + f7 + f8
    if rftop == 0: 
        rftop = 0.01
    
    lfycop = (((f1 * (17) + f2 * (17) + f3 * (-17) + f4 * (-17)))/ lftop)
    lfxcop = (f1 * (42) + f2 * (-42) + f3 * (-42) + f4 * (42)) / lftop 

    rfycop =  ((f5 * (17) + f6 * (17) + f7 * (-17) + f8 * (-17)) / rftop) 
    rfxcop =  ((f5 * (42) + f6 * (-42) + f7 * (-42) + f8 * (42)) / rftop) 

    CoP_x = (rfxcop + lfxcop) / 2
    CoP_y = (-(rfycop + lfycop) / 2)
    
    if rftop == 0.01 and lftop == 0.01: 
        CoP_y = 0.0 
    
    elif rftop == 0.01: 
        CoP_y= -(lfycop-60) 
        CoP_x= lfxcop   

    elif lftop == 0.01: 
        CoP_y= -(rfycop+60)
        CoP_x= rfxcop

    return CoP_x , CoP_y 

flag_co = 1
error_prev = 0
error_prev2 = 0
error_y_prev = 0
error_y_prev2 = 0
Ts = 1
counter = 0
moving_id1 = 0
moving_id2 = 0


def forceSensors():

    global CoP_x_store , CoP_x_value
    global CoP_y_store , CoP_y_value
    global f1,f2,f3,f4,f5,f6,f7,f8
    global sum_of_x , sum_of_y,Copx_list,Copy_list

    while True:
        readValue = port_pin.read()
        readValue_1 = port_pin_1.read()
        readValue_2 = port_pin_2.read()
        readValue_3 = port_pin_3.read()
        readValue_4 = port_pin_4.read()
        readValue_5 = port_pin_5.read()
        readValue_6 = port_pin_6.read()
        readValue_7 = port_pin_7.read()

        if readValue == None:
            readValue = 0.0
        elif readValue_1 == None:
            readValue_1 = 0.0
        elif readValue_2 == None:
            readValue_2 = 0.0
        elif readValue_3 == None:
            readValue_3 = 0.0
        elif readValue_4 == None:
            readValue_4 = 0.0
        elif readValue_5 == None:
            readValue_5 = 0.0
        elif readValue_6 == None:
            readValue_6 = 0.0
        elif readValue_7 == None:
            readValue_7 = 0.0
        else :
            f1 = int(readValue*1023)
            f2 = int(readValue_1*1023)
            f3 = int(readValue_2*1023)
            f4 = int(readValue_3*1023)
            f5 = int(readValue_4*1023)
            f6 = int(readValue_5*1023)
            f7 = int(readValue_6*1023)
            f8 = int(readValue_7*1023)
      
        CoP_x_value , CoP_y_value = CoP(f1,f2,f3,f4,f5,f6,f7,f8)
        if calib_button():
            sum_of_x , sum_of_y = calibrationProcessCoP()
                         


def Torque_Enable():
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler,ALL_MOTORS, ADDR_RX28_TORQUE_ENABLE,TORQUE_ENABLE)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))

def Torque_Disable():
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler,ALL_MOTORS, ADDR_RX28_TORQUE_ENABLE, TORQUE_DISABLE)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))

def yurumeye_basla():
    global flag
    for k in range(0,12):
        packetHandler.write2ByteTxRx(portHandler, DYNAMIXEL_ID[k],ADDR_RX28_GOAL_POS,int(yb[k]))
    flag = True

def Walking():
    time.sleep(3)
    packetHandler.write2ByteTxRx(portHandler, ALL_MOTORS, ADDR_RX28_MAX_TORQUE, 400)  #400
    packetHandler.write2ByteTxRx(portHandler,ALL_MOTORS,ADDR_RX28_MOVING_SPEED, 200) # 200

    packetHandler.write2ByteTxRx(portHandler,1,ADDR_RX28_MAX_TORQUE,900)
    packetHandler.write2ByteTxRx(portHandler,2,ADDR_RX28_MAX_TORQUE , 900)
    packetHandler.write2ByteTxRx(portHandler,ALL_MOTORS,48,1)
    
    i = 53  #53
    n = 0
    while True:

        roll_control = -0.2*roll
        pitch_control = -0.2*pitch
        output_3= 5
        
        output_31 = output_3
        output_32 = -output_3
        
        output_4 = 5
        if 20 <= CoP_x_value <= 42 or -42 <= CoP_x_value <= -20:
            output_4 = output_4
        else:
            output_4 = -output_4

        if 70<=CoP_y_value <= 77 or (43-7)<=CoP_y_value<=43 :  
            output_32 = 0
        elif -77<=CoP_y_value<=-70 or -43<=CoP_y_value<=(-43+7):
            output_31 = 0
        elif  5<=CoP_y_value<=15 or -15<=CoP_y_value<=-5:
            output_31 = output_3
            output_32 = -output_3
        else:
            output_31 = 0
            output_32 = 0
            

        if (-6 < roll < 6):  roll_control = 0
        if (-6 < pitch < 6): pitch_control = 0

        packetHandler.write2ByteTxRx(portHandler, DYNAMIXEL_ID[0],ADDR_RX28_GOAL_POS,(WALKING_START_ANGLE[0] - 10 + round(int(trajectory_array[i,0] + (pitch_control + output_31)/0.293))))
        packetHandler.write2ByteTxRx(portHandler, DYNAMIXEL_ID[1],ADDR_RX28_GOAL_POS,(WALKING_START_ANGLE[1] - 10 + round(int(trajectory_array[i,1] + (pitch_control + output_32)/0.293))))
        packetHandler.write2ByteTxRx(portHandler, DYNAMIXEL_ID[2],ADDR_RX28_GOAL_POS,(WALKING_START_ANGLE[2] + round(int(trajectory_array[i,2] + (roll_control + output_4)/0.293))))
        packetHandler.write2ByteTxRx(portHandler, DYNAMIXEL_ID[3],ADDR_RX28_GOAL_POS,(WALKING_START_ANGLE[3] + round(int(trajectory_array[i,3] + (roll_control + output_4)/0.293))))
        packetHandler.write2ByteTxRx(portHandler, DYNAMIXEL_ID[4],ADDR_RX28_GOAL_POS,(WALKING_START_ANGLE[4] + round(int(trajectory_array[i,4]))))
        packetHandler.write2ByteTxRx(portHandler, DYNAMIXEL_ID[5],ADDR_RX28_GOAL_POS,(WALKING_START_ANGLE[5] + round(int(trajectory_array[i,5]))))
        packetHandler.write2ByteTxRx(portHandler, DYNAMIXEL_ID[6],ADDR_RX28_GOAL_POS,(WALKING_START_ANGLE[6] + round(int(trajectory_array[i,6]) - 20 + (roll_control)/0.293)))
        packetHandler.write2ByteTxRx(portHandler, DYNAMIXEL_ID[7],ADDR_RX28_GOAL_POS,(WALKING_START_ANGLE[7] + round(int(trajectory_array[i,7]) - 20 + (roll_control)/0.293)))
        packetHandler.write2ByteTxRx(portHandler, DYNAMIXEL_ID[8],ADDR_RX28_GOAL_POS,(WALKING_START_ANGLE[8] + round(int(trajectory_array[i,8]))))
        packetHandler.write2ByteTxRx(portHandler, DYNAMIXEL_ID[9],ADDR_RX28_GOAL_POS,(WALKING_START_ANGLE[9] + round(int(trajectory_array[i,9]))))
        packetHandler.write2ByteTxRx(portHandler, DYNAMIXEL_ID[10],ADDR_RX28_GOAL_POS,(WALKING_START_ANGLE[10] + round(int(trajectory_array[i,10] + (pitch_control)/0.293))))
        packetHandler.write2ByteTxRx(portHandler, DYNAMIXEL_ID[11],ADDR_RX28_GOAL_POS,(WALKING_START_ANGLE[11] + round(int(trajectory_array[i,11] + (pitch_control)/0.293))))
        print(CoP_y_value , CoP_x_value)

        i = i + 5
        time.sleep(0.03)
            
        if i >= 103: # 50w
            for k in range(0,12):
                packetHandler.write2ByteTxRx(portHandler, DYNAMIXEL_ID[k],ADDR_RX28_GOAL_POS,round(yb[k]))
            time.sleep(3)
            i = 53
            n = n + 1
        
        if n == 2:
            break
            
    
def StandUpRight():

    packetHandler.write2ByteTxRx(portHandler, DYNAMIXEL_ID[2], ADDR_RX28_MOVING_SPEED, 100)
    packetHandler.write2ByteTxRx(portHandler, DYNAMIXEL_ID[3], ADDR_RX28_MOVING_SPEED, 100)
    packetHandler.write2ByteTxRx(portHandler, DYNAMIXEL_ID[4], ADDR_RX28_MOVING_SPEED, 300)
    packetHandler.write2ByteTxRx(portHandler, DYNAMIXEL_ID[5], ADDR_RX28_MOVING_SPEED, 300)

    packetHandler.write2ByteTxRx(portHandler,DYNAMIXEL_ID[4],ADDR_RX28_GOAL_POS, STAND_UPRIGTH_ANGLE_org[4])
    packetHandler.write2ByteTxRx(portHandler,DYNAMIXEL_ID[5],ADDR_RX28_GOAL_POS, STAND_UPRIGTH_ANGLE_org[5])
    packetHandler.write2ByteTxRx(portHandler,DYNAMIXEL_ID[2],ADDR_RX28_GOAL_POS, STAND_UPRIGTH_ANGLE_org[2])
    packetHandler.write2ByteTxRx(portHandler,DYNAMIXEL_ID[3],ADDR_RX28_GOAL_POS, STAND_UPRIGTH_ANGLE_org[3])
    packetHandler.write2ByteTxRx(portHandler,DYNAMIXEL_ID[6],ADDR_RX28_GOAL_POS, STAND_UPRIGTH_ANGLE_org[6])
    packetHandler.write2ByteTxRx(portHandler,DYNAMIXEL_ID[7],ADDR_RX28_GOAL_POS, STAND_UPRIGTH_ANGLE_org[7])
    packetHandler.write2ByteTxRx(portHandler,DYNAMIXEL_ID[0],ADDR_RX28_GOAL_POS, STAND_UPRIGTH_ANGLE_org[0])
    packetHandler.write2ByteTxRx(portHandler,DYNAMIXEL_ID[1],ADDR_RX28_GOAL_POS, STAND_UPRIGTH_ANGLE_org[1])
    packetHandler.write2ByteTxRx(portHandler,DYNAMIXEL_ID[8],ADDR_RX28_GOAL_POS, STAND_UPRIGTH_ANGLE_org[8])
    packetHandler.write2ByteTxRx(portHandler,DYNAMIXEL_ID[9],ADDR_RX28_GOAL_POS, STAND_UPRIGTH_ANGLE_org[9])
    packetHandler.write2ByteTxRx(portHandler,DYNAMIXEL_ID[10],ADDR_RX28_GOAL_POS, STAND_UPRIGTH_ANGLE_org[10])
    packetHandler.write2ByteTxRx(portHandler,DYNAMIXEL_ID[11],ADDR_RX28_GOAL_POS, STAND_UPRIGTH_ANGLE_org[11])
    
def BreakKnee():

    packetHandler.write2ByteTxRx(portHandler, ALL_MOTORS, ADDR_RX28_MOVING_SPEED, 300)
    packetHandler.write2ByteTxRx(portHandler,DYNAMIXEL_ID[4],ADDR_RX28_GOAL_POS, WALKING_START_ANGLE[4] + round(int(trajectory_array[26][4])))
    packetHandler.write2ByteTxRx(portHandler,DYNAMIXEL_ID[5],ADDR_RX28_GOAL_POS, WALKING_START_ANGLE[5] + round(int(trajectory_array[26][5])))
    packetHandler.write2ByteTxRx(portHandler,DYNAMIXEL_ID[2],ADDR_RX28_GOAL_POS, WALKING_START_ANGLE[2] + round(int(trajectory_array[26][2])))
    packetHandler.write2ByteTxRx(portHandler,DYNAMIXEL_ID[3],ADDR_RX28_GOAL_POS, WALKING_START_ANGLE[3] + round(int(trajectory_array[26][3])))
    packetHandler.write2ByteTxRx(portHandler,DYNAMIXEL_ID[6],ADDR_RX28_GOAL_POS, WALKING_START_ANGLE[6] + round(int(trajectory_array[26][6])))
    packetHandler.write2ByteTxRx(portHandler,DYNAMIXEL_ID[7],ADDR_RX28_GOAL_POS, WALKING_START_ANGLE[7] + round(int(trajectory_array[26][7])))

def calibrationProcessCoP() :
    CoP_x_axes =  []
    CoP_y_axes = []

    for _ in range(200):
        CoP_x_axes.append(CoP_x_value)
        CoP_y_axes.append(CoP_y_value)

    meanOfCoP_x_axes = np.sum(CoP_x_axes) / len(CoP_x_axes)
    meanOfCoP_y_axes = np.sum(CoP_y_axes) / len(CoP_y_axes)

    return meanOfCoP_x_axes , meanOfCoP_y_axes

 

EulerThread = threading.Thread(target= Euler,daemon=True)
EulerThread.start() 

forceThread = threading.Thread(target= forceSensors,daemon=True)
forceThread.start() 

controlThread = threading.Thread(target= justControlFunc,daemon=True)
controlThread.start() 

iD_1 = Label(master,text='A0 : ')
iD_1.place(x=10,y=90)
List = Label(master,text=str(f1))
List.place(x=40,y=90)

iD_2 = Label(master,text='A1 : ')
iD_2.place(x=10,y=130)
List1 =  Label(master,text=str(f2))
List1.place(x=40,y=130)

iD_3 = Label(master,text='A2 : ')
iD_3.place(x=10,y=170)
List2 = Label(master,text=str(f3))
List2.place(x=40,y=170)

iD_4 = Label(master,text='A3 : ')
iD_4.place(x=10,y=210)
List3 =  Label(master,text=str(f4))
List3.place(x=40,y=210)

iD_5 = Label(master,text='A4 : ')
iD_5.place(x=10,y=250)
List4 =  Label(master,text=str(f5))
List4.place(x=40,y=250)

iD_6 = Label(master,text='A5 : ')
iD_6.place(x=10,y=290)
List5 =  Label(master,text=str(f6))
List5.place(x=40,y=290)

iD_7 = Label(master,text='A6 : ')
iD_7.place(x=10,y=330)
List6 =  Label(master,text=str(f7))
List6.place(x=40,y=330)

iD_8 = Label(master,text='A7 : ')
iD_8.place(x=10,y=370)
List7 =  Label(master,text=str(f8))
List7.place(x=40,y=370)

copx = Label(master,text='CoPX : ')
copx.place(x=10,y=410)
copX =  Label(master,text=str(CoP_x_value))
copX.place(x=40,y=410)

copy = Label(master,text='CoPY : ')
copy.place(x=10,y=450)
copY = Label(master,text=str(CoP_y_value))
copY.place(x=40,y=450)


def update_matplotlib():

    scatter.set_offsets(np.column_stack((CoP_x_value, CoP_y_value)))
    new_scatter.set_offsets(np.column_stack((-roll, -pitch)))
    sum_scatter.set_offsets(np.array([[0, 0]]))
    ax.legend()
    canvas.draw() 
    
    List.config(text=str(f1))
    List1.config(text=str(f2))
    List2.config(text=str(f3))
    List3.config(text=str(f4))
    List4.config(text=str(f5))
    List5.config(text=str(f6))
    List6.config(text=str(f7))
    List7.config(text=str(f8))
    copX.config(text=str(CoP_x_value))
    copY.config(text=str(CoP_y_value))
    master.after(1, update_matplotlib)



calibration_button = Checkbutton(master,text='Calibration Button',variable = calibration,command=calib_button)
calibration_button.place(x=490,y = 30)

control__button = Checkbutton(master,text='Control Button',variable=control,command=control_button)
control__button.place(x=490,y = 10)


walk_Button = Button(master,  text='Walking' , command= Walking,fg='white',bg='black')
walk_Button.place(x=10,y=10)

standUpright_button = Button(master,text='Stand UpRight',command=StandUpRight,fg='white',bg='black')
standUpright_button.place(x=80,y=10)

prepareToWalk_button = Button(master,text='Prepare To Walk',command=yurumeye_basla,fg='white',bg='black')
prepareToWalk_button.place(x=180,y=10)

TorqueEnable_button = Button(master,text='Torque Enable',command=Torque_Enable,fg='white',bg='black')
TorqueEnable_button.place(x=290,y=10)

TorqueDisable_button = Button(master,text='Break Knee',command=BreakKnee,fg='white',bg='black')
TorqueDisable_button.place(x=390,y=10)
fig, ax = plt.subplots()
scatter = ax.scatter([], [], color='green',label='Force Sensors')
sum_scatter = ax.scatter([], [],color='blue',label='Zero Point')
new_scatter = ax.scatter([], [], color='red',label = 'IMU (Degree[°])')

ax.set_xlim(-55, 55)
ax.set_ylim(-80, 80)
plt.plot([-42, 42], [77, 77], color='green', linestyle='-')
plt.plot([-42, 42], [-77, -77], color='green', linestyle='-')
plt.plot([-42, 42], [43, 43], color='green', linestyle='-')
plt.plot([-42, 42], [-43, -43], color='green', linestyle='-')
plt.plot([42, 42],[-77, -43] , color='green', linestyle='-')
plt.plot([42, 42],[77, 43] , color='green', linestyle='-')
plt.plot([-42, -42],[-77, -43] , color='green', linestyle='-')
plt.plot([-42, -42],[77, 43] , color='green', linestyle='-')
ax.set_xlabel('x-mm')
ax.set_ylabel('[BACK] y-mm')

canvas = FigureCanvasTkAgg(fig, master=master)
canvas_widget = canvas.get_tk_widget()
canvas_widget.place(x=200,y=100)  
update_matplotlib()
master.mainloop()









