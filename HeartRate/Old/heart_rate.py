#!/usr/bin/python

import sys, getopt
import os.path
import time
import math
import ctypes

from threading import Thread
from ctypes import cdll

lib = cdll.LoadLibrary('/home/reach/HeartRate/heartRate.so')
lib.hrt_updated.restype = ctypes.c_int
lib.heart_init.restype = ctypes.c_int
lib.heart_get.restype = ctypes.c_float


class HeartRateCtr:

    def __init__(self):
        self.heartlib_not_started = True
        lib.heart_init()
        self.beats_list = []
        self.beats_average = 0.0
        self.current_beat = 0.0
        self.hr_thread = Thread(target = self.hr_periodic)
        self.hr_thread.start()
        self.last_beat = 0.0

    def hr_periodic(self):
        self.heartlib_not_started = False
        while True:
                # check for new Heart Beat
                new_hrt = lib.hrt_updated()
                #print "is beat updated", new_hrt
                if new_hrt == 1:
                #if True:
                    current_beat = lib.heart_get()
                    #print "current beat", current_beat

                    # check if the beat within the acceptable range
                    if current_beat < 200.0 and current_beat > 30.0 :

                        if abs(current_beat - self.last_beat) < 30.0:

                            if len(self.beats_list) >= 20:
                                del self.beats_list[0]
                            self.beats_list.append(current_beat)
                            self.beats_average = 0.0
                            if len(self.beats_list) > 0:
                                for beats in self.beats_list:
                                    self.beats_average += beats
                                self.beats_average /= len(self.beats_list)
                    if self.last_beat == current_beat and current_beat == 0.0:
                        self.beats_average = 0.0
                        self.beats_list = []
                    self.last_beat = current_beat
                time.sleep(0.1)



# for test only

if __name__ == "__main__":
    print "started"
    hrc = HeartRateCtr()
    while True:
        print "current beat\Average beat", hrc.last_beat, hrc.beats_average  # for test
        time.sleep(0.7)

