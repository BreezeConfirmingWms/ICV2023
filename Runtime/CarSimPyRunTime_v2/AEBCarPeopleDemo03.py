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


Flag = False
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

    velX = np.linspace(0, 10, 100)
    velY = np.linspace(0, 20, 100)
    velT = np.linspace(0, 0.5, 100)
    velS = np.linspace(0, 0.3, 100)

    M=1696*1e3

    r = 33.90

    a_f = 0.6


    c = 0
    cnt = 1
    pose_upd = SimOne_Data_Pose_Control()
    control_upd = SimOne_Data_Control()



    pd_data = SimOne_Data_Point_Cloud()
    gps_data = SimOne_Data_Gps()
    case_data = SimOne_Data_CaseInfo()

    waypointData = SimOne_Data_WayPoints()
    Obstacle = SimOne_Data_Obstacle()
    stream = SimOne_Data_Image()
    # if SoGetGps(mainVehicleID, gps_data):
    #     print(f"init velX : {gps_data.velX}")
    judge = False
    endJudge=False

    initX=None
    endX=None
    while Flag:


        SoAPIGetCaseInfo(case_data)

        case_zhName = case_data.caseName.decode('utf-8')



        if case_zhName=="03.前方行人横穿":
            
            S = SoGetCaseRunStatus()
            if S==1:
            # stop
             SaveEvaluationRecord()
             break
         
            sencfg_data = SimOne_Data_SensorConfiguration()
            if SoGetSensorConfigurations(mainVehicleID,sencfg_data) :

                print(sencfg_data.sensorId)




            control_upd.EBrakeMode=ESimOne_Brake_Mode(0)


            planeinfo = SimOne_Data_LaneInfo()
            if SoGetSensorLaneInfo(mainVehicleID,"sensorFusion1",planeinfo): # "objectBasedCamera1"
       
                id = planeinfo.id
                lanetype = planeinfo.lanetype
                laneleftid = planeinfo.laneleftid
                lanerightid = planeinfo.lanerightid
                lanepredecessorid = planeinfo.lanepredecessorid
                lanesuccessorid = planeinfo.lanesuccessorid
                print("id: {0}".format(id))
                print("lanetype: {0}".format(lanetype))
                print("laneleftid: {0}".format(laneleftid))
                print("lanerightid: {0}".format(lanerightid))
                i = 0
                while i < 3:
                    print("lanepredecessorid[{0}]: {1}".format(i, lanepredecessorid[i]))
                i += 1








        # if not SoGetCaseRunStatus():
        #     SaveEvaluationRecord()
        #     break
        # waypoint = SimOne_Data_WayPoints()
        # SoGetWayPoints(mainVehicleID,waypoint)
        # vehicleState = (ESimOne_Data_Vehicle_State * 3)(
        # 	ESimOne_Data_Vehicle_State.ESimOne_Data_Vehicle_State_SO_M_SW,
        # 	ESimOne_Data_Vehicle_State.ESimOne_Data_Vehicle_State_S0_Vz_SM,
        # 	ESimOne_Data_Vehicle_State.ESimOne_Data_Vehicle_State_SO_My_DR_L1);
        # vehicleStatelen = len(vehicleState)
        # if(SoRegisterVehicleState(mainVehicleID,vehicleState,vehicleStatelen)):
        # 	print("RegisterVehicleState Success")

        # vehicleExtraState = SimOne_Data_Vehicle_Extra();
        # if(SoGetVehicleState(mainVehicleID,vehicleExtraState)):
        # 	for i in range(0,vehicleStatelen):
        # 		print("SoGetVehicleState Success: state{0}:= {1}".format(i,vehicleExtraState.extra_states[i]))




        # control_upd.brake = 0.0267
        # control_upd.gear = 0
        # control_upd.throttle = 0



        # pose_upd.posX = velX[c]
        # pose_upd.posY = velY[c]

        # control_upd.throttle = velT[c]
        # control_upd.steering = velS[c]
        # control_upd.gear = 1


        # if SoSetPose(mainVehicleID,pose_upd):
        # 	if cnt%20==0:
        # 		print("the vechile is moving!")
        # 		print("the pose data posX:{} and posY:{}".format(pose_upd.posX,pose_upd.posY))

       
        #control_upd.brake = M*a_f*r


        """
        protected
        """
       #control_upd.brake = 0.0263

        control_upd.brake = 0.0263

        # if not judge and SoGetGps(mainVehicleID,gps_data) and gps_data.velX>0:
        #     initX=gps_data.posX
        #     print("init position is :{}".format(initX))
        #     judge=True

        # if SoSetDrive(mainVehicleID, control_upd) and SoGetGps(mainVehicleID,gps_data) and gps_data.velX>0:
        #     if cnt % 5 == 0 and not endJudge:
        #         print("running online: {}".format(S))
        #        # print("the brake data {}".format(control_upd.brake))
        #         print("accel X: {}".format(gps_data.accelX))
        #         print("vel X: {}".format(gps_data.velX))
        #         print("brake : {}".format(gps_data.brake))
        #         print("throttle : {}".format(gps_data.throttle))
        #         #print(case_data.caseName.decode('utf-8'))
        #         print("#########")
        #         if abs(gps_data.velX-0) < 1e-3 and not endJudge:
        #             endX=gps_data.posX
        #             print("total move distance is {}".format(endX-initX))

        #             endJudge=True





        # if SoGetGroundTruth(mainVehicleID,Obstacle):
        #
        #     if cnt%20==0:
        #         print(Obstacle.obstacleSize)
        #         print("Obstacle one's position is: \
        #         X:{0}, Y:{1}, Z:{2}".format(Obstacle.obstacle[0].posX,
        #         Obstacle.obstacle[0].posY,
        #         Obstacle.obstacle[0].posX))
        # if SoGetWayPoints(mainVehicleID,waypointData):
        # 	pass
        if c < 99:
            c += 1
        cnt += 1



        # if SoGetStreamingImage(ip = "127.0.0.1",port= 13956,imageData=stream):
        #     #print(stream.width, stream.height)
        #     pass



        # cnt=1;
        # if SoGetGps(mainVehicleID, gpsData):
        # 	cnt+=1
        # 	if cnt%100==0:
        # 		print("timestamp:{0},posX:{1},posY:{2},brake:{3},steering:{4},throttle:{5}".format(gpsData.timestamp,gpsData.posX,gpsData.posY,gpsData.brake,gpsData.steering,gpsData.throttle))
        # 		print("IMU: angVelX:{0}, angVelY: {1}, angVelZ: {2}".format(gpsData.imuData.angVelX, gpsData.imuData.angVelY, gpsData.imuData.angVelZ))

        # obstacleData = SimOne_Data_Obstacle()
        # if SoGetGroundTruth(mainVehicleID,obstacleData):
        # 	print("Size:{0}".format(obstacleData.obstacleSize))

        # v2xData = SimOne_Data_V2XNFS()
        # if SoGetV2XInfo(mainVehicleID,"obu1",1,v2xData):
        # 	print("Size:{0},MsgFrameData:{1}".format(v2xData.V2XMsgFrameSize,v2xData.MsgFrameData))

       
        # pHdmapData = SimOne_Data_Map()
        # if SoGetHDMapData(pHdmapData):
        # 	print("pHdmapData.openDrive:{0},pHdmapData.openDriveUrl:{1}".format(pHdmapData.openDrive,pHdmapData.openDriveUrl))

        # ImageData = SimOne_Data_Image()
        # Ip = "127.0.0.1"
        # Port = 13956
        # if SoGetStreamingImage(Ip, Port, ImageData):
        # 	print("ImageData.width:{0},ImageData.height:{1}, ImageData.format:{2}, ImageData.imagedataSize:{3}".format(ImageData.width, ImageData.height, ImageData.format, ImageData.imagedataSize))

        time.sleep(0.1)
        pass