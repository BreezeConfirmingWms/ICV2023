"""
@Time : 2023/10/7 17:06  
@Author : BreezeConfirming
@Software : Pycharm
@File : AvoidSlowChangeLane17.py
@Mail : wmsthinksv@gmail.com
@CopyRight 2023-Yan-Ming
"""





from SimOneServiceAPI import *
from SimOneSensorAPI import *
from SimOneV2XAPI import *
from SimOnePNCAPI import *
from SimOneStreamingAPI import *

import HDMapAPI as HDMapAPI
from SimOneEvaluation import *
from PythonHDMapAPISample import *

import time
import random
import math

import numpy as np
import pandas as pd
from scipy import optimize, stats, signal, linalg
from scipy import io as sio
import sympy


import logging
import os
import sys
import time





Flag = False
GetObstacle = None
cnt = 1


def start():
    print("start")


def stop():
    print("stop")

def SoObstacleCB(mainVehicleId, Data_obs):
    global GetObstacle
    if Data_obs is not None:
        GetObstacle = Data_obs

def SoGpsCB(mainVehicleId, Data_Gps):
    if Data_Gps:
        print("mainVehicleId:{0}, posX:{1}, posY: {2}, posZ: {3}".format(mainVehicleId, Data_Gps[0].posX,
                                                                         Data_Gps[0].posY, Data_Gps[0].posZ))


def SoV2XCB(mainVehicleId, sensorId, Data_V2XNFS):
    if Data_V2XNFS:
        print("Data_V2XNFS:{0}, SensorID:{1}, Data_V2XNFS_Size:{2}, Data_V2XNFS_Frame: {3}".format(mainVehicleId,
                                                                                                   sensorId,
                                                                                                   Data_V2XNFS[
                                                                                                       0].V2XMsgFrameSize,
                                                                                                   Data_V2XNFS[
                                                                                                       0].MsgFrameData))


def SoMainVehicleStaus(mainVehicleId, data):
    if data:
        print("mainVehicleId:{0},data:{1}".format(mainVehicleId, data.mainVehicleStatus))


def SoSetSensorDetectionUpdateCBTest(mainVehicleId, sensorId, data):
    if data:
        # print("mainVehicleId:{0},SensorId:{1}, data:{2}".format(mainVehicleId,sensorId,data[0].objectSize))
        for i in range(data[0].objectSize):
            print("data[0][{0}].type:{1}".format(i, data[0].objects[i].type.value))
def get_current_pos(gpsData,pX,pY):

    cX = gpsData.posX
    cY = gpsData.posY


    if  pX is not None and pY is not None and cX!=pX:

        slope = (cY - pY) / (cX - pX)
        crad = np.arctan(slope)
    else:
        crad=0

    return (cX, cY, crad)


pose_upd = SimOne_Data_Pose_Control()
control_upd = SimOne_Data_Control()

pd_data = SimOne_Data_Point_Cloud()
gps_data = SimOne_Data_Gps()
case_data = SimOne_Data_CaseInfo()

waypointData = SimOne_Data_WayPoints()
Obstacle = SimOne_Data_Obstacle()
stream = SimOne_Data_Image()

judge = False
endJudge = False
Time_label = False

init_Time = None
initX = None
endX = None

cur_X = None
cur_Y = None

cur_Time = None
cross_Time = None
if __name__ == '__main__':
    mainVehicleID = "0"
    try:
        if SoInitSimOneAPI(mainVehicleID, 0, "127.0.0.1") == 1:
            print("################## API init success!!!")
            Flag = True
        else:
            print("################## API init fail!!!")
    except Exception as e:
        print(e)
        pass

    SoSetDriveMode(mainVehicleID, driverMode="ESimOne_Drive_Mode_API")
    InitEvaluationServiceWithLocalData(mainVehicleID)

    while 1:
        logger_control = ESimOne_LogLevel_Type(0)
        if HDMapAPI.loadHDMap(20):
            SoSetLogOut(logger_control, "HDMAP LOADED")

            # pass
            break

    SoApiSetGpsUpdateCB(SoGpsCB)

    # stream=SoGetStreamingImage()
    Sen_cfg_data = SimOne_Data_SensorConfiguration()
    SoGetSensorConfigurations(mainVehicleID, Sen_cfg_data)

    print(Sen_cfg_data.sensorId)

    while Flag:

        SoAPIGetCaseInfo(case_data)

        case_zhName = case_data.caseName.decode('utf-8')

        if case_zhName == " ":

            S = SoGetCaseRunStatus()
            if S == 1:
                # stop
                SaveEvaluationRecord()
                break


            control_upd.throttle = 6.94

            control_upd.steering = 0

            control_upd.gear = ESimOne_Gear_Mode(1)
            control_upd.EThrottleMode = ESimOne_Throttle_Mode(2)

            SoApiSetGroundTruthUpdateCB(SoObstacleCB)
            if GetObstacle is not None:
                if cnt % 5 == 0:
                    print(GetObstacle.contents.obstacleSize)


            SoSetDrive(mainVehicleID,control_upd)

        time.sleep(0.1)
        pass
