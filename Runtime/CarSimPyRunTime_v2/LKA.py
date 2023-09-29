from SimOneServiceAPI import *
from SimOneSensorAPI import *
from SimOneV2XAPI import *
from SimOnePNCAPI import *
from SimOneStreamingAPI import *
import HDMapAPI as HDMapAPI
from transforms3d.quaternions import *
from SimOneEvaluation import *

import time

import random
import math

import numpy as np
import pandas as pd
from scipy import optimize, stats, signal, linalg
import sympy

import control as ct
import cvxopt


def start():
    print("start")


def stop():
    print("stop")


def SoGetPD(mainVechileId, Data_pd):
    if Data_pd:
        pass


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

def SampleGetRoadST(laneId, pos):
    print("SampleGetRoadST:")
    stzInfo = HDMapAPI.getRoadST(laneId, pos)
    if stzInfo.exists == False:
        print("Not exists!")
        return
    # print("[s,t] relative to this road:", stzInfo.s, ",", stzInfo.t)
    # print("height of input point:", stzInfo.z)
    return stzInfo.t

distanceToLaneBoundaryInfo = None
distToLeft = None
distToRight = None

def SampleGetDistanceToLaneBoundary(pos):
    print("SampleGetDistanceToLaneBoundary:")
    distanceToLaneBoundaryInfo = HDMapAPI.getDistanceToLaneBoundary(pos)
    if distanceToLaneBoundaryInfo.exists == False:
        print("Not exists!")
        return
    print("laneId:", distanceToLaneBoundaryInfo.laneId.GetString())
    print("distToLeft:", distanceToLaneBoundaryInfo.distToLeft)
    print("distToRight:", distanceToLaneBoundaryInfo.distToRight)
    print("distToLeft2D:", distanceToLaneBoundaryInfo.distToLeft2D)
    print("distToRight2D:", distanceToLaneBoundaryInfo.distToRight2D)

def SampleGetLaneType(laneId):
	print("SampleGetLaneType:")
	laneType = HDMapAPI.getLaneType(laneId)
	if laneType.exists == False:
		print("Not exists!")
		return
	print("lane type:", laneType.laneType)
 

curlaneid = None
Flag = False 
pos = None


judge = False
endJudge = False
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

    M = 1696 * 1e3

    r = 33.90

    a_f = 0.6

    c = 0
    cnt = 1
    """
    control data update
    """
    pose_upd = SimOne_Data_Pose_Control()
    control_upd = SimOne_Data_Control()
    control_esp_upd = SimOne_Data_ESP_Control()

    pd_data = SimOne_Data_Point_Cloud()
    gps_data = SimOne_Data_Gps()
    case_data = SimOne_Data_CaseInfo()

    waypointData = SimOne_Data_WayPoints()
    Obstacle = SimOne_Data_Obstacle()
    

    if SoGetGps(mainVehicleID, gps_data):
        print(f"init velX : {gps_data.velX}")
    if SoGetGps(mainVehicleID, gps_data):
        pt=HDMapAPI.pySimPoint3D(gps_data.posX,gps_data.posY,gps_data.posZ)
        lanesInfo=HDMapAPI.getNearLanes(pt, 3.0)
        if lanesInfo.exists:
            idListSize = lanesInfo.laneIdList.Size()
            curlaneid = lanesInfo.laneIdList.GetElement(0)
            judge = False
            endJudge = False
    
    initX = None
    initY=None
    
    steer_c1=0
    endX = None
    while Flag:

        SoAPIGetCaseInfo(case_data)
        case_zhName = case_data.caseName.decode('utf-8')

        # if SoSetDriveTrajectory(mainVehicleID,controlTrajectory):
        #     print("trajectory control ")
        if case_zhName == "05.弯道车道偏离抑制":

            S = SoGetCaseRunStatus()
            if S == 1:
                # stop
                SaveEvaluationRecord()
                break
            control_upd.gear = ESimOne_Gear_Mode(1)
            # sencfg_data = SimOne_Data_SensorConfiguration()
            # if SoGetSensorConfigurations(mainVehicleID, sencfg_data):
            #     print(sencfg_data.sensorId)

            """
            protected
            """

            if SoGetGps(mainVehicleID,gps_data):
                pos=HDMapAPI.pySimPoint3D(gps_data.posX,gps_data.posY,gps_data.posZ)
            if pos is not None:
                SampleGetDistanceToLaneBoundary(pos)
            if distToLeft is not None and distToRight is not None and (distToLeft+distToRight)<1*1e-1 :
                control_upd.EThrottleMode = ESimOne_Throttle_Mode(2)
                control_upd.throttle = 10 / 3.6
                control_upd.steering= -(2/45)
            elif distToLeft is not None and distToRight is not None and (distToLeft+distToRight)<2*1e-1 :
                control_upd.EThrottleMode = ESimOne_Throttle_Mode(2)
                control_upd.throttle =  10 / 3.6
                control_upd.steering= (2/45)
            
            if curlaneid is not None:
                SampleGetLaneType(curlaneid)
            # if curlaneid is not None:
            #     SampleGetRoadST(curlaneid, pos)
            if not judge and SoGetGps(mainVehicleID, gps_data) and gps_data.velX >= 0 :
                initX = gps_data.posX
                initY=gps_data.posY
                print("init position  X is :{}".format(initX))
                print("init position Y is :{}".format(initY))
                judge = True
                
            if SoSetDrive(mainVehicleID, control_upd) and SoGetGps(mainVehicleID, gps_data) and gps_data.velX >= 0:
                if cnt % 5 == 0 and not endJudge :
                    print("running online: {}".format(S))
                    print("accel X: {}".format(gps_data.accelX))
                    print("vel X: {}".format(gps_data.velX))
                    print("brake : {}".format(gps_data.brake))
                    print("throttle : {}".format(gps_data.throttle))
                    print(gps_data.posX,gps_data.posY)
                    # print(case_data.caseName.decode('utf-8'))
                    print("#########")
                if c < 99:
                    c += 1
                cnt += 1
                time.sleep(0.1)
                pass

