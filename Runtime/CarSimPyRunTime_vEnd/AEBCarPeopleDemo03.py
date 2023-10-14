"""
@Time : 2023/9/19 21:14  
@Author : BreezeConfirming
@Software : Pycharm
@File : AEBCarMoveDemo.py
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

import time

import random
import math

import numpy as np
import pandas as pd
from scipy import optimize, stats, signal, linalg
import sympy


# import torch




Flag=False
GetObstacle= None
M = 1696 * 1e3
r = 33.90
a_f = 0.6


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




    # SoApiSetV2XInfoUpdateCB(SoV2XCB)
    # SoAPISetMainVehicleStatusUpdateCB(SoMainVehicleStaus)
    # SoApiSetSensorDetectionsUpdateCB(SoSetSensorDetectionUpdateCBTest)
    # SoApiSetGpsUpdateCB(SoGpsCB)


    SoSetDriveMode(mainVehicleID, driverMode="ESimOne_Drive_Mode_API")
    InitEvaluationServiceWithLocalData(mainVehicleID)

    while 1:
        logger_control = ESimOne_LogLevel_Type(0)
        if HDMapAPI.loadHDMap(20):
            SoSetLogOut(logger_control, "HDMAP LOADED")

            # pass
            break




    pose_upd = SimOne_Data_Pose_Control()
    control_upd = SimOne_Data_Control()



    pd_data = SimOne_Data_Point_Cloud()
    gps_data = SimOne_Data_Gps()
    case_data = SimOne_Data_CaseInfo()

    waypointData = SimOne_Data_WayPoints()
    Obstacle = SimOne_Data_Obstacle()
    stream = SimOne_Data_Image()

    judge = False
    endJudge=False
    Time_label  = False

    init_Time = None
    initX=None
    endX=None

    cur_X=None
    cur_Y = None
    cur_Time = None
    cross_Time = None

    people_x=None
    people_y = None
    people_vel = None


    P_x = 36


    while Flag:


        SoAPIGetCaseInfo(case_data)

        case_zhName = case_data.caseName.decode('utf-8')



        if case_zhName=="03.前方行人横穿":
            
            S = SoGetCaseRunStatus()
            if S==1:
            # stop
             SaveEvaluationRecord()
             break


            control_upd.gear = ESimOne_Gear_Mode(1)
            control_upd.EThrottleMode = ESimOne_Throttle_Mode(2)
            control_upd.throttle=30/3.6
            control_upd.steering =0


            SoApiSetGroundTruthUpdateCB(SoObstacleCB)
            if GetObstacle is not None :
                if cnt % 5 ==0:
                    print(GetObstacle.contents.obstacleSize)
                    print("people location is ({},{})".format(GetObstacle.contents.obstacle[0].posX,\
                                                              GetObstacle.contents.obstacle[0].posY))

                people_x = GetObstacle.contents.obstacle[0].posX
                people_y = GetObstacle.contents.obstacle[0].posY

                people_vel = GetObstacle.contents.obstacle[0].velX


            if SoGetGps(mainVehicleID,gps_data):
                cur_X = gps_data.posX
                cur_Y = gps_data.posY

                if not Time_label:
                    init_Time = gps_data.timestamp/1000
                    init_X = cur_X
                    Time_label=True
                if init_Time is not None:
                    cur_Time = gps_data.timestamp/1000-init_Time
                    print("Time is"+str(cur_Time))

            if people_x is not None and people_y is not None:

                if abs(people_x-cur_X)<10 and abs(people_y-cur_Y)<3:
                    control_upd.throttle=0
                    if cross_Time is None:
                        cross_Time = cur_Time
                    print(f"(Time Cross At {cross_Time})")

                # if abs(P_x-cur_X)<25:
                #     control_upd.throttle=0
                #     control_upd.gear = ESimOne_Gear_Mode(3)
                #     control_upd.brake=1.0
                #     if cross_Time is None:
                #         cross_Time = cur_Time

                if  cross_Time is not None and (cur_Time-cross_Time)>(8*3.6)/5 :
                    control_upd.throttle=30/3.6

                    #control_upd.gear=ESimOne_Gear_Mode(2)






            SoSetDrive(mainVehicleID,control_upd)

            cnt += 1





        time.sleep(0.1)
        pass