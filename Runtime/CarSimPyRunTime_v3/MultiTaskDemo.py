"""
@Time : 2023/10/8 0:02  
@Author : BreezeConfirming
@Software : Pycharm
@File : MultiTaskDemo.py
@Mail : wmsthinksv@gmail.com
@CopyRight 2023-Yan-Ming
"""

from SimOneServiceAPI import *
from SimOneSensorAPI import *
from SimOneV2XAPI import *
from SimOnePNCAPI import *
from SimOneStreamingAPI import *
import HDMapAPI as HDMapAPI
from PythonHDMapAPISample import SampleGetParkingSpaceIds, SampleGetParkingSpaceKnots

from SimOneEvaluation import *

import time
import os
import sys
import subprocess

import random
import math

import numpy as np
import pandas as pd
from scipy import optimize, stats, signal, linalg
import sympy

import control as ct
from control.matlab import *

import threading
from multiprocessing import context,connection,managers,Pool
from itertools import chain,count,cycle

def init_simone():
    try:
        if SoInitSimOneAPI("0", 0, "127.0.0.1") == 1:
            print("################## API init success!!!")
            Flag = True
        else:
            print("################## API init fail!!!")
    except Exception as e:
        print(e)
        pass


cnt=1
Flag = False

if __name__=="__main__":
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

    case_data = SimOne_Data_CaseInfo()

    SoSetDriveMode(mainVehicleID, driverMode="ESimOne_Drive_Mode_API")
    InitEvaluationServiceWithLocalData(mainVehicleID)

    while 1:
        logger_control = ESimOne_LogLevel_Type(0)
        if HDMapAPI.loadHDMap(20):
            SoSetLogOut(logger_control, "HDMAP LOADED")
            break


    caseZhNames = ["07.垂直泊车驶出","02.前方车辆制动","05.弯道车道偏离抑制"]
    Event_flag = np.zeros((3,1))
    proc = None
    SoAPIGetCaseInfo(case_data)

    case_zhName = case_data.caseName.decode('utf-8')
    while Flag:

        #case_zhName = caseZhNames[Event_cnt]




        if case_zhName == "02.前方车辆制动":
            print("AEB02的例程")
            cmd = ['powershell.exe','py36','AEBCarBrakeDemo02.py']
            if not Event_flag[0]:
                proc=subprocess.run(cmd,shell=True,stdout=subprocess.PIPE)

            # t = threading.Thread(target=init_simone)
            # t.start()
            # t.join()
            Event_flag[0] = 1
            #print(proc.stdout)
            #if SoInitSimOneAPI(mainVehicleID, 0, "127.0.0.1") == 1:
            case_zhName = "05.弯道车道偏离抑制"
            time.sleep(5)

            #time.sleep(1.5)
        elif case_zhName == "05.弯道车道偏离抑制":
            print("LKA05的例程")
            cmd = ['powershell.exe','py36','LKACurveDemo05.py']
            if not Event_flag[1]:
                proc=subprocess.run(cmd, shell=True, stdout=subprocess.PIPE)
            # t = threading.Thread(target=init_simone)
            # t.start()
            # t.join()
            print(proc.stdout)

            Event_flag[1]=1
            #time.sleep(1.5)
        elif case_zhName == "07.垂直泊车驶出":
            print("APA07的例程")
            cmd = ['powershell.exe', 'py36', 'APACarHorizon07.py']
            if not Event_flag[2]:
                proc=subprocess.run(cmd, shell=True, stdout=subprocess.PIPE)
            # t = threading.Thread(target=init_simone)
            # t.start()
            # t.join()
            print(proc.stdout)
            Event_flag[2]=1
            # if SoInitSimOneAPI(mainVehicleID, 0, "127.0.0.1") == 1:
            case_zhName = "02.前方车辆制动"
            time.sleep(5)
            #time.sleep(1.5)
        else:
            print("check your choice pls!!!")

        if np.all(Event_flag==1):
            break