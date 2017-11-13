#include <stdio.h>
#include <iostream>
#include <string>
#include <thread>
#include <chrono>
#include <csignal>
#include <ctime>
#include <cmath>
#include <vector>

#include "RtkLib/src/rtklib.h"
#include "IMU/RTIMULib/RTIMULib.h"
#include "GpsImuFusion/GpsImuKalman.h"
#include "HeartRate/heartRate.h"

struct GoalmaxOutput
{
    double PredictedLat = 0;
    double PredictedLon = 0;
    double PredictedAlt = 0;
    double ResultantMPH = 0;
	float HeartRate = 0;
	float heading;
	float steps;
	playerPosition position;
};


int goalmaxDataProcessingInit(void);
int goalmaxDataProcessingPeriodic(void);
GoalmaxOutput * readGoalmaxData(void);