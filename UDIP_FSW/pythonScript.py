# -*- coding: utf-8 -*-
"""
Created on Thu Mar  5 22:38:12 2020

@author: jarod
"""
from numpy import *

from struch import unpack

from scipy.io import savemat

import pickle

len_head = 12

len_sens = 25 + len_head

len_sweep;

def readFile(f, out = False) :
    n_dat = 1000
    
    #header arrays
    countArray = tile(0, n_dat)
    tInitialArray = tile(0, n_dat)
    tFinalArray = tile(0, n_dat)
    
    #Sensor Payload arrays
    accelDigArray   = tile(0.,[n_dat,3])
    accelAnaArray   = tile(0.,n_dat)
    gyroArray       = tile(0., [n_dat, 3])
    magArray        = tile(0., [n_dat,3])
    tempDigArray    = tile(0., n_dat)
    tempAnaArray    = tile(0., n_dat)
    accelScaleArray = tile(0, n_dat)
    
    #Sweep Payload arrays
    brst0Array = tile(0., [n_dat, 20])
    brst1Array = tile(0., [n_dat, 20])
    brst2Array = tile(0., [n_dat, 20])
    
    med0Array = tile(0., [n_dat, 100])
    med1Array = tile(0., [n_dat, 100])
    med2Array = tile(0., [n_dat, 100])
    
    lrg0Array = tile(0., [n_dat, 200])
    lrg1Array = tile(0., [n_dat, 200])
    lrg2Array = tile(0., [n_dat, 200])
    
    
    