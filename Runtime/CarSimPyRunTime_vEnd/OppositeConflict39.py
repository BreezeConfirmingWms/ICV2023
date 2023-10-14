"""
@Time : 2023/10/8 11:48  
@Author : BreezeConfirming
@Software : Pycharm
@File : OppositeConflict39.py
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


def get_current_pos(gpsData, pX, pY):
    cX = gpsData.posX
    cY = gpsData.posY

    if pX is not None and pY is not None and cX != pX:

        slope = (cY - pY) / (cX - pX)
        crad = np.arctan(slope)
    else:
        crad = 0

    return (cX, cY, crad)


pose_upd = SimOne_Data_Pose_Control()
control_upd = SimOne_Data_Control()

pd_data = SimOne_Data_Point_Cloud()
gps_data = SimOne_Data_Gps()
case_data = SimOne_Data_CaseInfo()
signal_data = SimOne_Data_Signal_Lights()

waypointData = SimOne_Data_WayPoints()
Obstacle = SimOne_Data_Obstacle()
stream = SimOne_Data_Image()


judge = False
endJudge = False
Time_label = False
Turn_label = False

init_Time = None
initX = None
endX = None

cur_X = None
cur_Y = None
prev_X = None
prev_Y = None

cur_Time = None
cross_Time = None

obs_size = 0
Car_targetX = None
Car_targetY = None
Car_targetV = None
Car_targetA = None

Car_forward_X = None
Car_forward_Y = None
Car_forward_V = None

prev_Fv = None
init_F = True

TurnStraight = False
TurnAdjust = False

Car_slant_X = None
Car_slant_Y = None
Car_slant_V = None

Bike_X = None
Bike_Y =None
Bike_V =None

car_degree = None

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

    # stream=SoGetStreamingImage()
    Sen_cfg_data = SimOne_Data_SensorConfiguration()
    SoGetSensorConfigurations(mainVehicleID, Sen_cfg_data)

    print(Sen_cfg_data.sensorId)

    while Flag:

        SoAPIGetCaseInfo(case_data)

        case_zhName = case_data.caseName.decode('utf-8')

        if case_zhName == "39.对向冲突":

            S = SoGetCaseRunStatus()
            if S == 1:
                # stop
                SaveEvaluationRecord()
                break

            control_upd.gear = ESimOne_Gear_Mode(1)
            control_upd.EThrottleMode = ESimOne_Throttle_Mode(2)
            control_upd.throttle = 6.94
            control_upd.steering = 0

            SoApiSetGroundTruthUpdateCB(SoObstacleCB)
            if GetObstacle is not None:

                if cnt % 5 == 0:
                    print(GetObstacle.contents.obstacleSize)
                    obs_size = 0
                Car_targetX = GetObstacle.contents.obstacle[0].posX
                Car_targetY = GetObstacle.contents.obstacle[0].posY

                print(f"Target Car locate on ({Car_targetX},{Car_targetY})")
                if Car_forward_V is not None and init_F:
                    prev_Fv = Car_forward_V
                    init_F =False
                Car_forward_X = GetObstacle.contents.obstacle[2].posX
                Car_forward_Y = GetObstacle.contents.obstacle[2].posY
                Car_forward_V = GetObstacle.contents.obstacle[2].velX

                Bike_X = GetObstacle.contents.obstacle[1].posX
                Bike_Y = GetObstacle.contents.obstacle[1].posY
                Bike_V = GetObstacle.contents.obstacle[1].velX

                print(f"Forward Car locate on ({Car_forward_X},{Car_forward_Y})")
                print(f"Forward Bike locate on ({Bike_X},{Bike_Y})")

                Car_targetV = GetObstacle.contents.obstacle[0].velX
                Car_targetA = GetObstacle.contents.obstacle[0].accelX

            if SoGetGps(mainVehicleID, gps_data):

                if cur_X is not None and cur_Y is not None:
                    prev_Y = cur_Y
                    prev_X = cur_X

                cur_X = gps_data.posX
                cur_Y = gps_data.posY

                print(f"Sim car position is ({cur_X},{cur_Y})")

                if not Time_label:
                    init_Time = gps_data.timestamp / 1000
                    init_X = cur_X
                    Time_label = True
                if init_Time is not None:
                    cur_Time = gps_data.timestamp / 1000 - init_Time
                    print("Current Time is " + str(cur_Time))

                _, _, crad = get_current_pos(gps_data, prev_X, prev_Y)

                car_degree = np.rad2deg(crad)

            if cur_X is not None and Car_targetX is not None and prev_Fv is not None:
                if 0 < cur_X - Car_targetX < 17.5 or abs(cur_Y - Car_targetY) < 3 :
                    print("避让")
                    control_upd.throttle = 3/ 3.6
                    control_upd.steering = 18/ 30

                    Turn_label = True
                if abs(cur_X - Car_targetX)<3.5 or abs(cur_Y - Car_targetY)<1.5:
                    control_upd.throttle=12/3.6
                    control_upd.steering=1
            if cur_X is not None and Car_forward_X is not None:
                if car_degree is not None and Turn_label:
                    if car_degree < -20:
                        control_upd.throttle=3/3.6
                        control_upd.steering = -25/ 30
                        TurnAdjust=True
                    if TurnAdjust:
                        control_upd.throttle = 3 / 3.6
                        control_upd.steering = -25 / 30
                    if  -5<car_degree<0 and abs(prev_Y-cur_Y)<0.75 and TurnAdjust:

                        print("待回正")
                        control_upd.steering=-0.3/30
                        control_upd.throttle=15/3.6
                        TurnStraight = True
                        TurnAdjust = False


                    # if abs(cur_X-Bike_X)<3.5:
                    #     control_upd.steering=0
                    #     control_upd.throttle=8/3.6
                if TurnStraight:
                    control_upd.throttle=15/3.6
                    control_upd.steering=-1/30
                if -1.5<car_degree<1.5 and TurnStraight:
                    control_upd.steering =0

                if abs(cur_X-Car_forward_X)<2.5 and TurnStraight:
                    control_upd.throttle=0


            SoSetDrive(mainVehicleID, control_upd)

        time.sleep(0.1)
        pass
