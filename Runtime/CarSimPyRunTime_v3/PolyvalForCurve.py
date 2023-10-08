"""
@Time : 2023/9/29 17:34  
@Author : BreezeConfirming
@Software : Pycharm
@File : PolyvalForCurve.py
@Mail : wmsthinksv@gmail.com
@CopyRight 2023-Yan-Ming
"""


import math
import random

import numpy as np

import os
from pathlib import Path
import csv

from collections import OrderedDict,deque
from itertools import count,cycle

from scipy.optimize import curve_fit

def circle_curve(x,h,k):
    return (x[0]-h)**2+(x[1]-k)**2-50**2


prt_check = True
if __name__ == "__main__":
    with open("LKA_waypoints.txt") as f:
        poly_dots = list(csv.reader(f,delimiter=',',quoting=csv.QUOTE_NONNUMERIC))

    poly_dots = np.asarray(poly_dots)
    #poly_center = np.mean(poly_dots,axis=0)
    poly_center = [poly_dots[1][0],poly_dots[1][1]-50]


    poly_fx = np.arange(poly_dots[0][0],poly_dots[1][0],0.5)
    poly_fx = poly_fx+np.random.uniform(-0.1,0.1,poly_fx.shape[0])
    poly_fx=poly_fx.reshape(-1,1)
    poly_fy = poly_dots[0][0]*np.ones((poly_fx.shape[0],1))+np.random.uniform(-0.1,0.1,(poly_fx.shape[0],1))
    poly_fv = 3/3.6*np.ones((poly_fx.shape[0],1))
    poly_circle = poly_dots[1:-1,...]
    r=50

    param,_ =  curve_fit(circle_curve,(poly_circle[:,0].T,poly_circle[:,1].T),np.zeros(3),p0=poly_center)

    angle_s = np.deg2rad(np.arange(85,5,-0.5))


    if prt_check:
        for i in range(poly_dots.shape[0]):
            val = (poly_dots[i][0]-param[0])**2 + (poly_dots[i][1]-param[1])**2-r**2
            print(val)


    poly_angle = np.zeros((angle_s.shape[0],3))
    angle_s = angle_s.tolist()
    for i,val in enumerate(angle_s):
        poly_angle[i][0] = param[0]+r*np.cos(val)+random.uniform(-0.1,0.1)
        poly_angle[i][1] = param[1]+r*np.sin(val)+random.uniform(-0.1,0.1)
        poly_angle[i][2] = 4/3.6+random.uniform(-0.3,0.3)

    poly_forward = np.hstack((poly_fx,poly_fy,poly_fv))

    poly_angles =np.vstack((poly_angle,poly_circle))
    sorted_angle = np.argsort(poly_angles[:,0])
    poly_angle_data = poly_angles[sorted_angle]

    poly_data = np.vstack((poly_forward,poly_angle_data))

    np.savetxt("poly_waypoints.txt",poly_data,delimiter=", ",fmt="%.2f")


