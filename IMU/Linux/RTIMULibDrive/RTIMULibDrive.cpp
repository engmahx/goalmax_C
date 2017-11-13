////////////////////////////////////////////////////////////////////////////
//
//  This file is part of RTIMULib
//
//  Copyright (c) 2014-2015, richards-tech, LLC
//
//  Permission is hereby granted, free of charge, to any person obtaining a copy of
//  this software and associated documentation files (the "Software"), to deal in
//  the Software without restriction, including without limitation the rights to use,
//  copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the
//  Software, and to permit persons to whom the Software is furnished to do so,
//  subject to the following conditions:
//
//  The above copyright notice and this permission notice shall be included in all
//  copies or substantial portions of the Software.
//
//  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
//  INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
//  PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
//  HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
//  OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
//  SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.


#include "RTIMULib.h"
#include "data_processor.h"
#include <thread>
#include <chrono>
#include <cmath>  
#include <vector>
#define positionDeadband 15

using namespace std;

extern "C"
{
	data_processor_t data_processor_init(int nfft, int hz);
	float data_processor_run(data_processor_t dfft);
	void data_processor_close(data_processor_t dfft);
}

RTIMU_DATA imuData;
RTVector3 Acc_residuals;
imuDataLink imu_data;
int imuRunFlag = 1;
vector<double> accelResiduals;



void calcHeading(imuDataLink * tempImuData)
{
	tempImuData->heading = tempImuData->yaw +270;
	if (tempImuData->heading > 360)
	{
		tempImuData->heading -= 360;
	}
}

void calcPosition(imuDataLink * tempImuData)
{
	static int Deadband = positionDeadband;
	if(abs(tempImuData->pitch) < (45 + Deadband))
		if ((abs(tempImuData->roll) < (135 + Deadband))&& (abs(tempImuData->roll) > (45 - Deadband)))
			tempImuData->position = Standing;
		else if (abs(tempImuData->roll) > (135 + Deadband))
			tempImuData->position = Falling_Face;
		else
			tempImuData->position = Falling_Back;
	else
		tempImuData->position = Falling;

	if (tempImuData->position == Standing)
		Deadband = positionDeadband;
	else
		Deadband = -1 * positionDeadband;
}

void calcAbsAcce(imuDataLink * tempImuData)
{
	double num1  = imuData.fusionQPose.x() * 2;
	double num2  = imuData.fusionQPose.y() * 2;
	double num3  = imuData.fusionQPose.z() * 2;
	double num4  = imuData.fusionQPose.x() * num1;
	double num5  = imuData.fusionQPose.y() * num2;
	double num6  = imuData.fusionQPose.z() * num3;
	double num7  = imuData.fusionQPose.x() * num2;
	double num8  = imuData.fusionQPose.x() * num3;
	double num9  = imuData.fusionQPose.y() * num3;
	double num10 = imuData.fusionQPose.scalar() * num1;
	double num11 = imuData.fusionQPose.scalar() * num2;
	double num12 = imuData.fusionQPose.scalar() * num3;

	double tempAccelerationEast  = (1.0 - (num5 + num6)) * Acc_residuals.x() + (num7 - num12) * Acc_residuals.y() + 
									(num8 + num11) * Acc_residuals.z();
	double tempAccelerationNorth = (num7 + num12) * Acc_residuals.x() + (1.0 - (num4 + num6)) * Acc_residuals.y() + 
									(num9 - num10) * Acc_residuals.z();
	double tempAccelerationUp    = (-1) * ((num8 - num11) * Acc_residuals.x() + (num9 + num10) * Acc_residuals.y() + 
									(1.0 - (num4 + num5)) * Acc_residuals.z());
	
	// Compensate the effect of magnetic Declination Offset
	// magneticDeclinationOffset for suez 4.25
	double magneticDeclinationOffset = 4.25;
	double TO_RADIANS_COEFF = 3.141592653589793238463 / 180;

	double sinMagneticDeclination = sin(magneticDeclinationOffset * TO_RADIANS_COEFF);
    double cosMagneticDeclination = cos(magneticDeclinationOffset * TO_RADIANS_COEFF);

	double easternNorthComponent  = sinMagneticDeclination * tempAccelerationEast;
	double northernEastComponent  = (-1) * sinMagneticDeclination * tempAccelerationNorth;

	double northernNorthComponent = cosMagneticDeclination * tempAccelerationNorth;
	double easternEastComponent   = cosMagneticDeclination * tempAccelerationEast;

	tempImuData->accelerationEast  = easternEastComponent + easternNorthComponent;
	tempImuData->accelerationNorth = northernNorthComponent + northernEastComponent;
	tempImuData->accelerationUp    = tempAccelerationUp;
	// MAHX DEBUG
	/*
	if(tempImuData->accelerationEast > 0.005)
	{
		printf("----- Debug ------\n");
		printf("Debug data :\n accelerationEast = %f\n accelerationNorth = %f\n accelerationUp = %f\n",
			tempImuData->accelerationEast, tempImuData->accelerationNorth, tempImuData->accelerationUp);
		printf("Debug data :\n tempAccelerationEast = %f\n tempAccelerationNorth = %f\n tempAccelerationUp = %f\n",
			tempAccelerationEast, tempAccelerationNorth, tempAccelerationUp);
		printf("sinMagneticDeclination = %f\n cosMagneticDeclination = %f\n",
			sinMagneticDeclination, cosMagneticDeclination);
	}
	*/
	// MAHX DEBUG
}

imuDataLink * get_imu_data(void)
{

	return &imu_data;
}


void shutdownImu(void)
{
	imuRunFlag = 0;
}
void read_data(kiss_fft_cpx *cin)
{
	int i = 0;
	for(auto data:accelResiduals)
	{
		cin[i].i = 0;
		cin[i].r = data;
		i += 1;
	}
}
void calc_steps(imuDataLink * tempImuData)
{
	float hz = 0;
	accelResiduals.push_back(Acc_residuals.y());
	if(accelResiduals.size()>= 1024)
	{
		data_processor_t dfft = NULL;
		dfft = data_processor_init(1024, 256);
		if (!dfft)
		{
			 printf("data_processor_init error\n");
			 return ;
		}
		read_data(dfft->cin);
		hz = data_processor_run(dfft);

		data_processor_close(dfft);
	    dfft = NULL;
		accelResiduals.erase (accelResiduals.begin(),accelResiduals.begin()+255);

		tempImuData->steps = hz;
	}
	
}
int imu_main(void)
{
    //  Using RTIMULib here allows it to use the .ini file generated by RTIMULibDemo.
    //  Or, you can create the .ini in some other directory by using:
    //      RTIMUSettings *settings = new RTIMUSettings("<directory path>", "RTIMULib");
    //  where <directory path> is the path to where the .ini file is to be loaded/saved
	imu_data.newImuData = false;

    RTIMUSettings *settings = new RTIMUSettings("RTIMULib");

    RTIMU * imu = RTIMU::createIMU(settings);

    if ((imu == NULL) || (imu->IMUType() == RTIMU_TYPE_NULL)) {
        printf("No IMU found\n");
        exit(1);
    }

    //  This is an opportunity to manually override any settings before the call IMUInit
	
    //  set up IMU

    imu->IMUInit();

    //  this is a convenient place to change fusion parameters

    imu->setSlerpPower(0.02);
    imu->setGyroEnable(true);
    imu->setAccelEnable(true);
    imu->setCompassEnable(true);

    //  now just process data

    while (imuRunFlag) {
        //  poll at the rate recommended by the IMU

        //usleep(imu->IMUGetPollInterval() * 1000);
		this_thread::sleep_for(chrono::milliseconds(imu->IMUGetPollInterval()));	

        if (imu->IMURead()) {
            imuData = imu->getIMUData();
			Acc_residuals = imu->getAccelResiduals();

			imu_data.roll = imuData.fusionPose.x() * (180.0 / RTMATH_PI);
			imu_data.pitch = imuData.fusionPose.y() * (180.0 / RTMATH_PI);
			imu_data.yaw = imuData.fusionPose.z() * (180.0 / RTMATH_PI);
			calcHeading(&imu_data);
			calcPosition(&imu_data);
			calcAbsAcce(&imu_data);
			calc_steps(&imu_data);

			imu_data.newImuData = true;
        }
    }
	return 0;
}

