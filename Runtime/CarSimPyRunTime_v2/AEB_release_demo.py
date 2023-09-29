"""
@Time : 2023/9/24 12:46  
@Author : BreezeConfirming
@Software : Pycharm
@File : AEB_release_demo.py
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


GetObstacle=None

def start():
    print("start")


def stop():
    print("stop")


def SoGetPD(mainVechileId, Data_pd):
    if Data_pd:
        pass

def SoObstacleCB(mainVehicleId,Data_obs):
    global GetObstacle
    if Data_obs is not None:
       GetObstacle=Data_obs


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

    # while 1:
    #     """
    #     设置地图的日志等级以及等待加载
    #     """
    #
    #     logger_control = ESimOne_LogLevel_Type(0)
    #     if HDMapAPI.loadHDMap(20):
    #         SoSetLogOut(logger_control, "HDMAP LOADED")
    #        # pass
    #         break

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
    ultra_data_list = SimOne_Data_UltrasonicRadars()
    ultra_data= SimOne_Data_UltrasonicRadar()

    sensor_detect_data =SimOne_Data_SensorDetections()

    WaypointData = SimOne_Data_WayPoints()

    Stream = SimOne_Data_Image()
    # if SoGetGps(mainVehicleID, gps_data):
    #     print(f"init velX : {gps_data.velX}")
    judge = False
    endJudge=False

    initX=None
    endX=None



    while Flag:

        S = SoGetCaseRunStatus()
        # if S==1:
        #     SaveEvaluationRecord()
        #     break
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
        #
        # sencfg_data = SimOne_Data_SensorConfigurations()
        # if SoGetSensorConfigurations(mainVehicleID,sencfg_data)  and cnt %10==0:
        #
        #     print(sencfg_data.data[0].sensorId.decode('ascii'))
        #     print(sencfg_data.data[0].mainVehicle.decode())


        #     print(sencfg_data.data[0].sensorType)
        #     print(sencfg_data.data[0].yaw)
        #
        #     print(sencfg_data.data[0].sensorId.decode('ascii'))
        #     print("********")

        # if  SoGetUltrasonicRadar(mainVehicleID,"ultrasonic1",ultra_data) and cnt%5==0:
        #     print(ultra_data.obstalcleDetections.x)


        # Obstacle = SimOne_Data_Obstacle()
        # if SoGetGroundTruth(mainVehicleID,Obstacle) :
        #     print(f"Size: {Obstacle.obstacleSize}")

        SoApiSetGroundTruthUpdateCB(SoObstacleCB)
        if GetObstacle is not None:
            print(GetObstacle.contents.obstacleSize)
            print(GetObstacle.contents.obstacle[0].posX)

            # if Obstacle.obstacleSize>0:
            #print(Obstacle.obstacle[0].type)
            #print(Obstacle.obstacle[0].id)
            # print(Obstacle.obstacle[0].posX)
            # print(Obstacle.obstacle[0].posY)




        control_upd.EBrakeMode=ESimOne_Brake_Mode(0)
        #control_upd.brake = M*a_f*r


        """
        protected
        """
       #control_upd.brake = 0.0263

        control_upd.brake = 0.0263

        if SoGetSensorDetections(mainVehicleID,'sensorFusion1',sensor_detect_data) and cnt % 5 ==0:
            print(sensor_detect_data.objectSize)
            #print(sensor_detect_data.objects[0].type)
            # for i in range(sensor_detect_data.objectSize):
            #     print("detection pos is {}".format(sensor_detect_data.objects[i].posX))
            #     print("target detect prob is {}".format(sensor_detect_data.objects[i].velX))
            print("detection pos is {}".format(sensor_detect_data.objects[0].posX))
            print("target detect vel is {}".format(sensor_detect_data.objects[0].velX))
            print("bbox is shown below")
            print(sensor_detect_data.objects[0].bbox2dMinX)
            print(sensor_detect_data.objects[0].bbox2dMinY)
            print(sensor_detect_data.objects[0].bbox2dMaxX)
            print(sensor_detect_data.objects[0].bbox2dMaxY)



        # if not judge and SoGetGps(mainVehicleID,gps_data) and gps_data.velX>0:
        #     initX=gps_data.posX
        #     print("init position is :{}".format(initX))
        #     judge=True
        #
        # if SoSetDrive(mainVehicleID, control_upd) and SoGetGps(mainVehicleID,gps_data) and gps_data.velX>0:
        #     if cnt % 5 == 0 and not endJudge:
        #         print("running online: {}".format(S))
        #        # print("the brake data {}".format(control_upd.brake))
        #         print("accel X: {}".format(gps_data.accelX))
        #         print("vel X: {}".format(gps_data.velX))
        #         print("brake : {}".format(gps_data.brake))
        #         print("throttle : {}".format(gps_data.throttle))
        #
        #         if abs(gps_data.velX-0) < 1e-3 and not endJudge:
        #             endX=gps_data.posX
        #             print("total move distance is {}".format(endX-initX))
        #
        #             endJudge=True


        #
        # pSensorConfigs = SimOne_Data_SensorConfigurations()
        # if SoGetSensorConfigurations(mainVehicleID, pSensorConfigs):
        # 	for index in range(pSensorConfigs.dataSize):
        # 		print("pSensorConfig.sensorId:{0}, pSensorConfig.SensorType:{1}".
        # 		format(pSensorConfigs.data[index].sensorId,pSensorConfigs.data[index].sensorType))


        # if SoGetWayPoints(mainVehicleID,waypointData):
        # 	pass

        ### count
        if c < 99:
            c += 1


        cnt += 1



        """
        
        limits number of loop
        """
        if cnt>=1000:
            break;

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

        # planeInfo = SimOne_Data_LaneInfo()
        # if SoGetSensorLaneInfo(mainVehicleID,"sensorFusion1",planeInfo): # "objectBasedCamera1"
        # 	id = planeInfo.id
        # 	laneType = planeInfo.laneType
        # 	laneLeftID = planeInfo.laneLeftID
        # 	laneRightID = planeInfo.laneRightID
        # 	lanePredecessorID = planeInfo.lanePredecessorID
        # 	laneSuccessorID = planeInfo.laneSuccessorID
        # 	print("id: {0}".format(id))
        # 	print("laneType: {0}".format(laneType))
        # 	print("laneLeftID: {0}".format(laneLeftID))
        # 	print("laneRightID: {0}".format(laneRightID))
        # 	i = 0
        # 	while i <3:
        # 		print("lanePredecessorID[{0}]: {1}".format(i, lanePredecessorID[i]))
        # 		i += 1
        # 	i = 0
        # 	while i <3:
        # 		print("laneSuccessorID[{0}]: {1}".format(i, laneSuccessorID[i]))
        # 		i += 1
        # 	l_Line = planeInfo.l_Line
        # 	l_Line_lineID = l_Line.lineID
        # 	l_Line_lineType = l_Line.lineType.value
        # 	l_Line_lineColor = l_Line.lineColor.value
        # 	l_Line_linewidth = l_Line.linewidth
        # 	# l_Line_linePoints = l_Line.linePoints
        # 	# l_Line_linecurveparameter = l_Line.linecurveParameter
        # 	print("l_Line_lineID: {0}".format(l_Line_lineID))
        # 	print("l_Line_lineType: {0}".format(l_Line_lineType))
        # 	print("l_Line_lineColor: {0}".format(l_Line_lineColor))
        # 	print("l_Line_linewidth: {0}".format(l_Line_linewidth))
        # 	c_Line = planeInfo.c_Line
        # 	c_Line_lineID = c_Line.lineID
        # 	c_Line_lineType = c_Line.lineType.value
        # 	c_Line_lineColor = c_Line.lineColor.value
        # 	c_Line_linewidth = c_Line.linewidth
        # 	# c_Line_linePoints = c_Line.linePoints
        # 	# c_Line_linecurveparameter = c_Line.linecurveParameter
        # 	print("c_Line_lineID: {0}".format(c_Line_lineID))
        # 	print("c_Line_lineType: {0}".format(c_Line_lineType))
        # 	print("c_Line_lineColor: {0}".format(c_Line_lineColor))
        # 	print("c_Line_linewidth: {0}".format(c_Line_linewidth))
        # 	r_Line = planeInfo.r_Line
        # 	r_Line_lineID = r_Line.lineID
        # 	r_Line_lineType = r_Line.lineType.value
        # 	r_Line_lineColor = r_Line.lineColor.value
        # 	r_Line_linewidth = r_Line.linewidth
        # 	# r_Line_linePoints = r_Line.linePoints
        # 	# r_Line_linecurveparameter = r_Line.linecurveParameter
        # 	print("r_Line_lineID: {0}".format(r_Line_lineID))
        # 	print("r_Line_lineType: {0}".format(r_Line_lineType))
        # 	print("r_Line_lineColor: {0}".format(r_Line_lineColor))
        # 	print("r_Line_linewidth: {0}".format(r_Line_linewidth))
        # 	ll_Line = planeInfo.ll_Line
        # 	ll_Line_lineID = ll_Line.lineID
        # 	ll_Line_lineType = ll_Line.lineType.value
        # 	ll_Line_lineColor = ll_Line.lineColor.value
        # 	ll_Line_linewidth = ll_Line.linewidth
        # 	# ll_Line_linePoints = ll_Line.linePoints
        # 	# ll_Line_linecurveparameter = ll_Line.linecurveParameter
        # 	print("ll_Line_lineID: {0}".format(ll_Line_lineID))
        # 	print("ll_Line_lineType: {0}".format(ll_Line_lineType))
        # 	print("ll_Line_lineColor: {0}".format(ll_Line_lineColor))
        # 	print("ll_Line_linewidth: {0}".format(ll_Line_linewidth))
        # 	rr_Line = planeInfo.rr_Line
        # 	rr_Line_lineID = rr_Line.lineID
        # 	rr_Line_lineType = rr_Line.lineType.value
        # 	rr_Line_lineColor = rr_Line.lineColor.value
        # 	rr_Line_linewidth = rr_Line.linewidth
        # 	# rr_Line_linePoints = rr_Line.linePoints
        # 	# rr_Line_linecurveparameter = rr_Line.linecurveParameter
        # 	print("rr_Line_lineID: {0}".format(rr_Line_lineID))
        # 	print("rr_Line_lineType: {0}".format(rr_Line_lineType))
        # 	print("rr_Line_lineColor: {0}".format(rr_Line_lineColor))
        # 	print("rr_Line_linewidth: {0}".format(rr_Line_linewidth))
        # 	print("------------ SoGetSensorLaneInfo ------------")

        # pMainVehicleInfo = SimOne_Data_MainVehicle_Info();
        # if SoGetMainVehicleList(pMainVehicleInfo):
        # 	print("	pMainVehicleInfo.size:{0}, pMainVehicleInfo.idList:{1}".format(pMainVehicleInfo.size,pMainVehicleInfo.id_list[0]))
        # 	for index in range(pMainVehicleInfo.size):
        # 		print("pMainVehicleInfo.id:		{0}".format(pMainVehicleInfo.id_list[index].value))
        # 	for indextype in range(pMainVehicleInfo.size):
        # 		print("pMainVehicleInfo.type:		{0}".format(pMainVehicleInfo.type_list[indextype].value))



        # pHdmapData = SimOne_Data_Map()
        # if SoGetHDMapData(pHdmapData):
        # 	print("pHdmapData.openDrive:{0},pHdmapData.openDriveUrl:{1}".format(pHdmapData.openDrive,pHdmapData.openDriveUrl))

        # ImageData = SimOne_Data_Image()
        # Ip = "127.0.0.1"
        # Port = 0
        # if SoGetStreamingImage(Ip, Port, ImageData):
        # 	print("ImageData.width:{0},ImageData.height:{1}, ImageData.format:{2}, ImageData.imagedataSize:{3}".format(ImageData.width, ImageData.height, ImageData.format, ImageData.imagedataSize))

        time.sleep(0.1)
        pass