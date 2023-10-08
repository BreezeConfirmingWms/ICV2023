"""
@Time : 2023/10/7 1:24  
@Author : BreezeConfirming
@Software : Pycharm
@File : NoTrafficLight30.py
@Mail : wmsthinksv@gmail.com
@CopyRight 2023-Yan-Ming
"""

from SimOneServiceAPI import *
from SimOneSensorAPI import *
from SimOneV2XAPI import *
from SimOnePNCAPI import *
from SimOneStreamingAPI import *
from SimOneEvaluation import *



import logging
import os
import  sys

import time


Flag = False
GetObstacle=None
cnt=1


def start():
    print("start")


def stop():
    print("stop")


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




pose_upd = SimOne_Data_Pose_Control()
control_upd = SimOne_Data_Control()

pd_data = SimOne_Data_Point_Cloud()
gps_data = SimOne_Data_Gps()
case_data = SimOne_Data_CaseInfo()

waypointData = SimOne_Data_WayPoints()
Obstacle = SimOne_Data_Obstacle()
stream = SimOne_Data_Image()
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

    controler = SimOne_Data_Control()
    SoSetDriverName(mainVehicleID, driverName="AEB")
    SoSetDriveMode(mainVehicleID, driverMode="")

    SoApiSetGpsUpdateCB(SoGpsCB)

    # stream=SoGetStreamingImage()
    Sen_cfg_data = SimOne_Data_SensorConfiguration()
    SoGetSensorConfigurations(mainVehicleID, Sen_cfg_data)

    print(Sen_cfg_data.sensorId)

    while Flag:


        controler.brake = 1
        gpsData = SimOne_Data_Gps()
        if SoGetGps(mainVehicleID, gpsData):
            print("timestamp:{0},posX:{1},posY:{2},brake:{3},steering:{4},throttle:{5}".format(gpsData.timestamp,
                                                                                               gpsData.posX,
                                                                                               gpsData.posY,
                                                                                               gpsData.brake,
                                                                                               gpsData.steering,
                                                                                               gpsData.throttle))
            print(
                "IMU: angVelX:{0}, angVelY: {1}, angVelZ: {2}".format(gpsData.imuData.angVelX, gpsData.imuData.angVelY,
                                                                      gpsData.imuData.angVelZ))



        time.sleep(0.1)
        pass
