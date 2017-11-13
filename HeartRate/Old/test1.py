import sys, getopt
import os.path
import time
import math

import ctypes

beats_list = []

from ctypes import cdll
lib = cdll.LoadLibrary('/home/reach/HeartRate/heartRate.so')

lib.heart_init.restype = ctypes.c_int

lib.heart_init()

#if ADC_result == 0:
#    print "ADC communication succeded"
#else:
#    print "ADC communication failed"

lib.heart_get.restype = ctypes.c_float
beats_average = 0
while True:
    current_beat = lib.heart_get()
    beats_average = 0
    if current_beat < 190 and current_beat > 40 :
        if len(beats_list) is 20:
            del beats_list[0]
        beats_list.append(current_beat)        
        for beats in beats_list:
            beats_average += beats
        beats_average /= len(beats_list)

    print "Heart rate is:",beats_average
    time.sleep(1)

    #beats_average = beats_average + 1
    #print "Heart rate is:",beats_average

