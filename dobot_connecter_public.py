#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import platform
from ctypes import *
import DobotDllType as dType
import time
import traceback
import os

os.chdir("/home/pi/dobot") #change dir to dobot folder.

__title__ = 'dobot_connecter.py'
__version__ = '0.0.01'
__build__ = 0x000001
__author__ = 'kyc1109'

CON_STR = {
    dType.DobotConnect.DobotConnect_NoError:  "DobotConnect_NoError",
    dType.DobotConnect.DobotConnect_NotFound: "DobotConnect_NotFound",
    dType.DobotConnect.DobotConnect_Occupied: "DobotConnect_Occupied"}

api = dType.load() #Load Dll

init_pos=[] #init position

state = dType.ConnectDobot(api, "", 115200)[0]
print("Connect status:",CON_STR[state])

def dobot_get_state():
    return CON_STR[state]

def dobot_get_pose(): #x,y,z,r,jointAngle[4] 底座、大臂、小臂、末端
    global state 
    if (state == dType.DobotConnect.DobotConnect_NoError):
        #Dobot interactive codes
        #Clean Command Queued
        dType.SetQueuedCmdClear(api)
        #Async Motion Params Setting
        tagPose = dType.GetPose(api) #x,y,z,r
        show_pose_data(tagPose)
        #Async Home
        #Start to Execute Command Queued
        dType.SetQueuedCmdStartExec(api)
        #Stop to Execute Command Queued
        dType.SetQueuedCmdStopExec(api)
    #dType.DisconnectDobot(api)
    return  tagPose
    
def show_pose_data(tagPose):
    print("   x:",format(tagPose[0],".3f"))
    print("   y:",format(tagPose[1],".3f"))
    print("   z:",format(tagPose[2],".3f"))
    #print("   r:",tagPose[3])
    return tagPose[0],tagPose[1],tagPose[2] #x,y,z

def dobot_move_xy(int_x, int_y, int_z=10,up_down=1,delay=0): #Up_down: 1 is press and release key. 0 is just moved
    global state 
    global init_pos
    if (state == dType.DobotConnect.DobotConnect_NoError):    
        try:
            while True:
                dType.SetQueuedCmdClear(api)
                
                dType.SetPTPJointParams(api, 100, 100, 100, 100, 100, 100, 100, 100, isQueued = 1) #四軸速度，加速度
                dType.SetPTPCommonParams(api, 100, 100, isQueued = 1)                
                
                #print("Before")
                tagPose = dType.GetPose(api) #x,y,z,r
                #show_pose_data(tagPose)                
                
                if not init_pos:
                    init_pos = dType.GetPose(api)
                    print("Inintial Position Saved.")
                    x = tagPose[0]+int_x
                    y = tagPose[1]+int_y
                    z = tagPose[2]
                    r = tagPose[3]
                else:                
                    x = tagPose[0]+int_x
                    y = tagPose[1]+int_y
                    z = tagPose[2]
                    r = tagPose[3]

                lastIndex = dType.SetPTPCmd(api, dType.PTPMode.PTPMOVLXYZMode, x, y, z, r, isQueued = 1)[0]#水平移動
                
                if up_down==1:
                    lastIndex = dType.SetPTPCmd(api, dType.PTPMode.PTPMOVLXYZMode, x, y, z-int_z, r, isQueued = 1)[0]#down                                  
                dType.SetQueuedCmdStartExec(api)
                #Wait for Executing Last Command
                while lastIndex > dType.GetQueuedCmdCurrentIndex(api)[0]:
                    dType.dSleep(delay*1000)
                dType.SetQueuedCmdStopExec(api)                
                if up_down==1:
                    lastIndex = dType.SetPTPCmd(api, dType.PTPMode.PTPMOVLXYZMode, x, y, z, r, isQueued = 1)[0]#up                                  
                dType.SetQueuedCmdStartExec(api)
                #Wait for Executing Last Command
                while lastIndex > dType.GetQueuedCmdCurrentIndex(api)[0]:
                    dType.dSleep(0)
                dType.SetQueuedCmdStopExec(api)   
                
                #print("After")
                tagPose = dType.GetPose(api) #x,y,z,r
                show_pose_data(tagPose)   
                
                return 0
        except Exception as e:
            traceback.print_exc()
            print("Disconnection")
            dType.DisconnectDobot(api)
            return 1
    elif (state == dType.DobotConnect.DobotConnect_Occupied):
        print("Device on use, please close previous app.")
        return 1
    elif (state == dType.DobotConnect.DobotConnect_NotFound):
        print("Device not found")
        return 1
    else:
        print("Unknow Error")
        return 1
        
#Most used
def dobot_move_xy_real(int_x, int_y, int_z=999, up_down=1, up_down_z=10, delay=0): #Up_down: 1 is press and release key. 0 is just moved
    global state 
    global init_pos
    if (state == dType.DobotConnect.DobotConnect_NoError):
        try:
            while True:
                dType.SetQueuedCmdClear(api) #if dobot_move_xy_real using too fast, it may cause jump position to move.
                
                dType.SetPTPJointParams(api, 200, 200, 200, 200, 200, 200, 200, 200, isQueued = 1) #四軸速度，加速度
                dType.SetPTPCommonParams(api, 100, 100, isQueued = 1)                
                
                #print("Before")
                tagPose = dType.GetPose(api) #x,y,z,r
                #show_pose_data(tagPose)                
                
                if not init_pos:
                    init_pos = dType.GetPose(api)
                    print("Inintial Position Saved.")
                    x = int_x
                    y = int_y
                    z = int_z
                    r = tagPose[3]
                else:                
                    x = int_x
                    y = int_y
                    z = int_z
                    r = tagPose[3]
                
                if int(z)==999: #check z, if z=999 skip
                    z = tagPose[2]
                    #print("z replace to tagPose ",z)
                if range_check(x,y,z):#x > 120 or x < 320:
                    #print("x,y,z in range")                  
                    lastIndex = dType.SetPTPCmd(api, dType.PTPMode.PTPMOVLXYZMode, x, y, z, r, isQueued = 1)[0]#水平移動
                    
                    #up_down_z=10
                    if up_down==1:
                        lastIndex = dType.SetPTPCmd(api, dType.PTPMode.PTPMOVLXYZMode, x, y, z-up_down_z, r, isQueued = 1)[0]#down                                  
                    dType.SetQueuedCmdStartExec(api)
                    #Wait for Executing Last Command
                    while lastIndex > dType.GetQueuedCmdCurrentIndex(api)[0]:
                        dType.dSleep(delay*1000)
                    dType.SetQueuedCmdStopExec(api)                
                    if up_down==1:
                        lastIndex = dType.SetPTPCmd(api, dType.PTPMode.PTPMOVLXYZMode, x, y, z, r, isQueued = 1)[0]#up                                  
                    dType.SetQueuedCmdStartExec(api)
                    #Wait for Executing Last Command
                    while lastIndex > dType.GetQueuedCmdCurrentIndex(api)[0]:
                        dType.dSleep(0)
                    dType.SetQueuedCmdStopExec(api) 
                
                #print("After")
                tagPose = dType.GetPose(api) #x,y,z,r
                x,y,z = show_pose_data(tagPose)   
                           
                return x,y,z
        except Exception as e:
            traceback.print_exc()
            print("Disconnection")
            dType.DisconnectDobot(api)
            return 1
    elif (state == dType.DobotConnect.DobotConnect_Occupied):
        print("Device on use, please close previous app.")
        return 1
    elif (state == dType.DobotConnect.DobotConnect_NotFound):
        print("Device not found")
        return 1
    else:
        print("Unknow Error")
        return 1

def dobot_get_xy_real(): #Up_down: 1 is press and release key. 0 is just moved
    global state 
    global init_pos
    up_down=1
    int_z=10
    delay=0
    if (state == dType.DobotConnect.DobotConnect_NoError):    
        try:
            while True:
                dType.SetQueuedCmdClear(api)
                
                dType.SetPTPJointParams(api, 200, 200, 200, 200, 200, 200, 200, 200, isQueued = 1) #四軸速度，加速度
                dType.SetPTPCommonParams(api, 100, 100, isQueued = 1)                
                
                #print("Before")
                tagPose = dType.GetPose(api) #x,y,z,r
                #show_pose_data(tagPose)                
                
                if not init_pos:
                    init_pos = dType.GetPose(api)
                    print("Inintial Position Saved.")
                    x = tagPose[0]
                    y = tagPose[1]
                    z = tagPose[2]
                    r = tagPose[3]
                else:                
                    x = tagPose[0]
                    y = tagPose[1]
                    z = tagPose[2]
                    r = tagPose[3]

                lastIndex = dType.SetPTPCmd(api, dType.PTPMode.PTPMOVLXYZMode, tagPose[0], tagPose[1], z, r, isQueued = 1)[0]#水平移動
                
                if up_down==1:
                    lastIndex = dType.SetPTPCmd(api, dType.PTPMode.PTPMOVLXYZMode, tagPose[0], tagPose[1], z-int_z, r, isQueued = 1)[0]#down                                  
                dType.SetQueuedCmdStartExec(api)
                #Wait for Executing Last Command
                while lastIndex > dType.GetQueuedCmdCurrentIndex(api)[0]:
                    dType.dSleep(delay*1000)
                dType.SetQueuedCmdStopExec(api)                
                if up_down==1:
                    lastIndex = dType.SetPTPCmd(api, dType.PTPMode.PTPMOVLXYZMode, tagPose[0], tagPose[1], z, r, isQueued = 1)[0]#up                                  
                dType.SetQueuedCmdStartExec(api)
                #Wait for Executing Last Command
                while lastIndex > dType.GetQueuedCmdCurrentIndex(api)[0]:
                    dType.dSleep(0)
                dType.SetQueuedCmdStopExec(api)    
                
                #print("After")
                tagPose = dType.GetPose(api) #x,y,z,r
                x,y,z = show_pose_data(tagPose) #x,y,z
                           
                return x,y
        except Exception as e:
            traceback.print_exc()
            print("關閉連線")
            dType.DisconnectDobot(api)
            return 1
    elif (state == dType.DobotConnect.DobotConnect_Occupied):
        print("Device on use, please close previous app.")
        return 1
    elif (state == dType.DobotConnect.DobotConnect_NotFound):
        print("Device not found")
        return 1
    else:
        print("Unknow Error")
        return 1        
    
def dobot_move_xy_reset(int_x,int_y,up_down=0): #up_down: 1 is press and release key. 0 is just moved
    global state 
    global init_pos
    if (state == dType.DobotConnect.DobotConnect_NoError):    
        try:
            while True:
                dType.SetQueuedCmdClear(api)
                
                dType.SetPTPJointParams(api, 200, 200, 200, 200, 200, 200, 200, 200, isQueued = 1) #四軸速度，加速度
                dType.SetPTPCommonParams(api, 100, 100, isQueued = 1)
                
                #print("Before")
                tagPose = dType.GetPose(api) #x,y,z,r
                #show_pose_data(tagPose)                
                
                x = int_x
                y = int_y
                z = tagPose[2]
                r = tagPose[3]

                lastIndex = dType.SetPTPCmd(api, dType.PTPMode.PTPMOVLXYZMode, x, y, z, r, isQueued = 1)[0]#水平移動
                
                if up_down==1:
                    lastIndex = dType.SetPTPCmd(api, dType.PTPMode.PTPMOVLXYZMode, x, y, z-5, r, isQueued = 1)[0]#down
                    lastIndex = dType.SetPTPCmd(api, dType.PTPMode.PTPMOVLXYZMode, x, y, z, r, isQueued = 1)[0]#up
                                  
                dType.SetQueuedCmdStartExec(api)            
                #Wait for Executing Last Command
                while lastIndex > dType.GetQueuedCmdCurrentIndex(api)[0]:
                    dType.dSleep(1000)            
                dType.SetQueuedCmdStopExec(api)
                
                #print("After")
                tagPose = dType.GetPose(api) #x,y,z,r
                #show_pose_data(tagPose)   
                                
                return 0
        except Exception as e:
            traceback.print_exc()
            print("關閉連線")
            dType.DisconnectDobot(api)
            return 1
    elif (state == dType.DobotConnect.DobotConnect_Occupied):
        print("Device on use, please close previous app.")
        return 1
    elif (state == dType.DobotConnect.DobotConnect_NotFound):
        print("Device not found")
        return 1
    else:
        print("Unknow Error")
        return 1 
        
def dobot_move_xy_hold(int_x,int_y,delay=1000,up_down=1): #to be removed.  up_down: 1 is press and release key. 0 is just moved
    global state 
    global init_pos
    if (state == dType.DobotConnect.DobotConnect_NoError):    
        try:
            while True:
                dType.SetQueuedCmdClear(api)
                
                dType.SetPTPJointParams(api, 200, 200, 200, 200, 200, 200, 200, 200, isQueued = 1) #四軸速度，加速度
                dType.SetPTPCommonParams(api, 100, 100, isQueued = 1)
                
                #print("Before")
                tagPose = dType.GetPose(api) #x,y,z,r
                #show_pose_data(tagPose)                
                
                if not init_pos:
                    init_pos = dType.GetPose(api)
                    print("Inintial Position Saved.")
                    x = tagPose[0]+int_x
                    y = tagPose[1]+int_y
                    z = tagPose[2]
                    r = tagPose[3]
                else:                
                    x = tagPose[0]+int_x
                    y = tagPose[1]+int_y
                    z = tagPose[2]
                    r = tagPose[3]

                lastIndex = dType.SetPTPCmd(api, dType.PTPMode.PTPMOVLXYZMode, x, y, z, r, isQueued = 1)[0]#水平移動
                
                if up_down==1:
                    lastIndex = dType.SetPTPCmd(api, dType.PTPMode.PTPMOVLXYZMode, x, y, z-5, r, isQueued = 1)[0]#down
                                  
                dType.SetQueuedCmdStartExec(api)
                #Wait for Executing Last Command
                while lastIndex > dType.GetQueuedCmdCurrentIndex(api)[0]:
                    dType.dSleep(delay)
                dType.SetQueuedCmdStopExec(api)
                
                if up_down==1:
                    lastIndex = dType.SetPTPCmd(api, dType.PTPMode.PTPMOVLXYZMode, x, y, z, r, isQueued = 1)[0]#up
                                  
                dType.SetQueuedCmdStartExec(api)
                #Wait for Executing Last Command
                while lastIndex > dType.GetQueuedCmdCurrentIndex(api)[0]:
                    dType.dSleep(0)
                dType.SetQueuedCmdStopExec(api)                
                                
                #print("After")
                tagPose = dType.GetPose(api) #x,y,z,r
                show_pose_data(tagPose)   
           
                return 0
        except Exception as e:
            traceback.print_exc()
            print("關閉連線")
            dType.DisconnectDobot(api)
            return 1
    elif (state == dType.DobotConnect.DobotConnect_Occupied):
        print("Device on use, please close previous app.")
        return 1
    elif (state == dType.DobotConnect.DobotConnect_NotFound):
        print("Device not found")
        return 1
    else:
        print("Unknow Error")
        return 1
        
def exit_dobot():    
    if not init_pos:
        pass
    else:
        dobot_move_xy_reset(float(init_pos[0]),float(init_pos[1]))
    dType.ClearAllAlarmsState(api)
    dType.DisconnectDobot(api)
    
def clear_dobot(): 
    global init_pos   
    if not init_pos:
        pass
    else:
        dobot_move_xy_reset(float(init_pos[0]),float(init_pos[1]))
    init_pos = dType.GetPose(api)
    
def get_init_position():
    global init_pos
    if not init_pos:
        init_pos = dType.GetPose(api)
    return init_pos
    
def dobot_init_home(): #如果機械臂運行速度過快或者負載過大可能會導致精度降低，此時可執行回零操作，提 高精度。
    global state 
    global init_pos
    if (state == dType.DobotConnect.DobotConnect_NoError):
        dType.SetQueuedCmdClear(api)
        dType.SetHOMEParams(api, 125, 0, 50, 0, isQueued = 1) #x,y,z,r. 125, 0, 50, 0,
        dType.SetHOMECmd(api, temp = 0, isQueued = 1) #move to home
        dType.SetQueuedCmdStartExec(api)
        dType.SetQueuedCmdStopExec(api)      

def dobot_relink(): #OK
    global state 
    global init_pos
    init_pos=[] #init position
    state = dType.ConnectDobot(api, "", 115200)[0]
    print("Connect status:",CON_STR[state])

def dobot_reset_pose(): #TBC 在以下情況，需重新設置即時位姿的基準值：角度感測器損壞。角度感測器精度太差。
    global state 
    if (state == dType.DobotConnect.DobotConnect_NoError):
        dType.SetQueuedCmdClear(api)
        dType.ResetPose(0) #x,y,z,r
        dType.SetQueuedCmdStartExec(api)
        dType.SetQueuedCmdStopExec(api)
    return  
def range_check(x,y,z):
    #x,y,z range check
    x_min=120
    x_max=321
    y_min=-250
    y_max=251
    z_min=-60
    z_max=151            
    if int(x) in range(x_min,x_max) and int(y) in range(y_min,y_max) and int(z) in range(z_min,z_max):#x > 120 or x < 320:
        #print("x,y,z in range")
        return True
    else:
        if not x in range(x_min,x_max):
            print(x," x not in range, x:120~320")
        elif not y in range(y_min,y_max):
            print(y," y not in range, y:+-250")            
        elif not z in range(z_min,z_max):
            print(z," z not in range, z:150~-60")           
        else:
            print("x, y or z not in range. x:120~320, y:+-250, z:150~-60")
        return False
def run_test():
    try:
        while True:
            print("1. X,Y Move.\n2. Go to Home.\n3. X,Y Move with real position.\n4. Get X,Y real position.\n5. Relink.\n6. DisconnectDobot")
            sel = input("\nSelect: (default is 3): \n") or "3"
            if sel == "1":
                while True:
                    x = input("\nx: (default is 0): \n") or "0"
                    y = input("\ny: (default is 0): \n") or "0"
                    if x=="q" or y=="q":
                        break
                    print(dobot_move_xy(float(x),float(y)))                
            elif sel == "2":
                dobot_init_home()
            elif sel == "3":    
                while True:
                    x = input("\nx: (default is 200): \n") or "200"
                    y = input("\ny: (default is 0): \n") or "0"
                    z = input("\nz: (default is 0): \n") or "0"
                    if x=="q" or y=="q":
                        break            
                    x,y,z = dobot_move_xy_real(float(x),float(y),float(z))
            elif sel == "4":    
                while True:
                    x = input("\nx: (default is 0): \n") or "0"
                    if x=="q":
                        break            
                    x,y = dobot_get_xy_real()
                    print("\nX:",x,"Y:",y)
            elif sel == "5":    
                dobot_relink()
            elif sel == "6":    
                exit_dobot()
            elif sel =="q":
                print("Goodbye")  
                exit_dobot()
                break
            else:
                print("No this option")
                #pass
            
    except KeyboardInterrupt:
        print("Esc by user")
    except Exception as e:
        traceback.print_exc()
    finally:
        if not init_pos:
            pass
        else:
            dobot_move_xy_reset(float(init_pos[0]),float(init_pos[1]))
if __name__ == "__main__":  # Start from here
    run_test()



