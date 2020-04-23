# -*- coding: utf-8 -*-
"""
Created on Thu Mar  5 22:38:12 2020

@author: jarod
"""
from numpy import tile

from struct import unpack

from scipy.io import savemat

import pickle

len_head = 13

len_sens = 25

len_brst = 133 * 10 * 2 * 3
len_med = 265 * 2 * 3
len_lrg = 253 * 2 * 3


def readFile(fileName, out = False) :
    n_dat = 1000
    
    #header arrays
    count = tile(0, n_dat)
    tInitial = tile(0, n_dat)
    tFinal = tile(0, n_dat)
    
    #Sensor Payload arrays
    accelDigX  = tile(0., n_dat )
    accelDigY  = tile(0., n_dat )
    accelDigZ  = tile(0., n_dat )
    accelAna   = tile(0., n_dat )
    gyroX      = tile(0., n_dat )
    gyroY      = tile(0., n_dat )
    gyroZ      = tile(0., n_dat )
    magX       = tile(0., n_dat )
    magY       = tile(0., n_dat )
    magZ       = tile(0., n_dat )
    tempDig    = tile(0., n_dat )
    tempAna    = tile(0., n_dat )
    accelScale = tile(0,  n_dat )
    
    #Sweep Payload arrays
    brstADC0 = tile(0., [n_dat, 127 * 10])
    brstADC1 = tile(0., [n_dat, 127 * 10])
    brstADC2 = tile(0., [n_dat, 127 * 10])
    
    medADC0 = tile(0., [n_dat, 255])
    medADC1 = tile(0., [n_dat, 255])
    medADC2 = tile(0., [n_dat, 255])
    
    lrgADC0 = tile(0., [n_dat, 255])
    lrgADC1 = tile(0., [n_dat, 255])
    lrgADC2 = tile(0., [n_dat, 255])
    
    
    myFile = open(fileName, "rb")
    
    raw = myFile.read()
    
    loc = 0
    i = 0
    
    while(loc < len(raw)) :
        #Extract the header
        hedr = raw[loc:(loc+len_head)]
        
        #Confirm UDIP Packet
        sync = unpack( '<H', hedr[0:2] )[0]
        
        if ( sync != 0x5544 ) :
            loc += 1
            continue
        
        count[i] = unpack('<H', hedr[2:4])[0]
        tInitial[i] = unpack('<I', hedr[4:8])[0]
        tFinal[i] = unpack('<I', hedr[8:12])[0]
        
        pcktType = unpack('<c', hedr[12])
        
        #Check the Packet Type
        if(pcktType == 0x01) :
            #Sensor Packet
            arr = raw[(loc + len_head) : (loc + len_head + len_sens)]
            
            accel = unpack('<hhhH', arr[0:8])
            accelDigX[i] = accel[0] #some calibration factor
            accelDigY[i] = accel[1] #some calibration factor
            accelDigZ[i] = accel[2] #some calibration factor
            accelAna [i] = accel[3] #some calibration factor
            
            gyro = unpack('<hhh', arr[8:14])
            gyroX[i] = gyro[0]
            gyroY[i] = gyro[1]
            gyroZ[i] = gyro[2]
            
            mag = unpack('<hhh', arr[14:20])
            magX[i] = mag[0]
            magY[i] = mag[1]
            magZ[i] = mag[2]
            
            temp = unpack('<hH', arr[20:24])
            tempDig[i] = temp[0] #IDK
            tempAna[i] = temp[1] ((3.3/4095.)-0.5)*100
            
            
            accelScale[i] = unpack()
            
            loc += len_head + len_sens
            i += 1
            
            
        elif(pcktType == 0x10) :
            #Medium Sweep Packet
            arr = raw[(loc + len_head) : (loc + len_head + len_med)]
            for x in range(255) :
                adc = unpack('<HHH', arr[x*2*3:x*2*3+6])
                adc = adc2Current(adc)
                medADC0[i][x] = adc[0] * (3.3/4095.)
                medADC1[i][x] = adc[1] * (3.3/4095.)
                medADC2[i][x] = adc[2] * (3.3/4095.)
            
            loc += len_head + len_med
            i += 1
            
        elif(pcktType == 0x11) :
            #Large Sweep Packet
            arr = raw[(loc + len_head) : (loc + len_head + len_lrg)]
            for x in range(255) :
                adc = unpack('<HHH', arr[x*2*3:x*2*3+6])
                lrgADC0[i][x] = adc[0] * (3.3/4095.)
                lrgADC1[i][x] = adc[1] * (3.3/4095.)
                lrgADC2[i][x] = adc[2] * (3.3/4095.)
                
            loc += len_head + len_lrg
            i += 1
            
        elif(pcktType == 0x20) :
            #Burst Sweep Packet
            arr = raw[(loc + len_head) : (loc + len_head + len_brst)]
            for x in range(10) :
                for y in range(127) :
                    adc = unpack('<HHH', arr[y*2*3:y*2*3+6])
                    brstADC0[i*10+x][y] = adc[0] * (3.3/4095.)
                    brstADC1[i*10+x][y] = adc[1] * (3.3/4095.)
                    brstADC2[i*10+x][y] = adc[2] * (3.3/4095.)
                
            loc += len_head + len_brst
            i += 1
            
        else :
            #invalid packet type
            loc += 1
            continue
        
    #truncate the arrays
    n_dat = i
    
    count = count[0:n_dat]
    tInitial = tInitial[0:n_dat]
    tFinal = tFinal[0:n_dat]
    
    accelDigX  =  accelDigX[0:n_dat]
    accelDigY  =  accelDigY[0:n_dat]
    accelDigZ  =  accelDigZ[0:n_dat]
    accelAna   =   accelAna[0:n_dat]
    gyroX      =      gyroX[0:n_dat]
    gyroY      =      gyroY[0:n_dat]
    gyroZ      =      gyroZ[0:n_dat]
    magX       =       magX[0:n_dat]
    magY       =       magY[0:n_dat]
    magZ       =       magZ[0:n_dat]
    tempDig    =    tempDig[0:n_dat]
    tempAna    =    tempAna[0:n_dat]
    accelScale = accelScale[0:n_dat]
        
    brstADC0 = brstADC0[0:n_dat*10]
    brstADC1 = brstADC1[0:n_dat*10]
    brstADC2 = brstADC2[0:n_dat*10]
    
    medADC0 = medADC0[0:n_dat]
    medADC1 = medADC1[0:n_dat]
    medADC2 = medADC2[0:n_dat]
    
    lrgADC0 = lrgADC0[0:n_dat]
    lrgADC1 = lrgADC1[0:n_dat]
    lrgADC2 = lrgADC2[0:n_dat]
    
    
    #package into dictionary????
    
    dat = {           'n':n_dat,
                  'count':count,
               'tInitial':tInitial,
                 'tFinal':tFinal,
              'accelDigX':accelDigX,
              'accelDigY':accelDigY,
              'accelDigZ':accelDigZ,
           '    accelAna':accelAna,
                  'gyroX':gyroX,
                  'gyroY':gyroY,
                  'gyroZ':gyroZ,
                   'magX':magX,
                   'magY':magY,
                   'magZ':magZ,
                'tempDig':tempDig,
                'tempAna':tempAna,
           'brstCurrent0':brstADC0,
           'brstCurrent1':brstADC1,
           'brstCurrent2':brstADC2,
            'medCurrent0':medADC0,
            'medCurrent1':medADC1,
            'medCurrent2':medADC2,
            'lrgCurrent0':lrgADC0,
            'lrgCurrent1':lrgADC1,
            'lrgCurrent2':lrgADC2  }
    
    if(out) :
        savemat((fileName[0:-3] + 'mat'), dat)
#     ????  # pkl = file((fileName[0:-3] + 'pkl'), 'wb')
#     ????    pickle.dump(dat, pkl)
    
    
    
rADC0 = 0
rADC1 = 0
rADC2 = 0


#adc2Current takes an array of Arduino Due Analog values (12 bit resolution)
# and returns the associated 
def adc2Current(adc) :
    convertedADC = {0., 0., 0.}
    for i in range(3) :
        if i == 0 :
            #No gain correction
            convertedADC[i] = adc[i] * 3.3/4095.
            #Voltage Divider takes off 1/4 of the voltage
            convertedADC[i] = convertedADC[i] * 4. #Maybe 12/3.3 ???
            #Now the transimpedance amplifier stuff
            
        if i == 1 :
            #One gain correction
            
        if i == 2 :
            #Two gain correction
    
def adc2Current0(adc) :
    
    
    
    
def adc2Current1(adc) :
    
    
    
    
def adc2Current2(adc) :