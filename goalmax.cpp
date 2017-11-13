#include "goalmax.h"

using namespace std;

extern "C" int rtkrcv_main(void);
extern "C" gps * get_gps_data(void);
extern "C" void sigshut(int sig);

// Declare variables
gps * gps_output;
imuDataLink * imu_output;
sensorData fusion_input_data;
outputPacket * fusion_output_packet;
GoalmaxOutput goalmax_output_data;
float last_heart_rate = 0;
auto gpsLast = chrono::high_resolution_clock::now();
auto gpsNow = chrono::high_resolution_clock::now();
auto elapsed = gpsNow - gpsLast;
long long microseconds;
int runflg = 1;
vector<float> HeartRateList;
float heartRateAverage = 0;

thread rtklibThread;
thread imulibThread;
thread fusionThread;

void goalmaxSigshut(int sig)
{
	cout << "Keyboard int" << "\n";
	runflg = 0;
	sigshut(sig);
	shutdownImu();
}

int goalmaxDataProcessingInit(void)
{	
	// Interrupt handlers
	/*
	signal(SIGINT, goalmaxSigshut);    // keyboard interrupt 
    signal(SIGTERM,goalmaxSigshut);    // external shutdown signal 
    signal(SIGUSR2,goalmaxSigshut);
    signal(SIGHUP ,SIG_IGN);
    signal(SIGPIPE,SIG_IGN);
	*/

	// Create RTKLib, IMU & heart rate threads
	rtklibThread = thread(rtkrcv_main);
	imulibThread = thread(imu_main);
	cout << "Step.0 \n";
	
	if(heart_init())
	{
		cout << "******* Communication with Heart rate ADC failed ******* \n\n";
		return 1;
	}
	
	cout << "Step.1 \n";
	// Read GPS data for fusion initialization
	gps_output = get_gps_data();
	cout << "Step.2 \n";
	// Read GPS data for fusion initialization
	imu_output = get_imu_data();
	cout << "Step.3 \n";
	// Load fusion initialize data
    fusion_input_data.Timestamp = time(nullptr);
    fusion_input_data.GpsLat = gps_output->lat;
    fusion_input_data.GpsLon = gps_output->lon;
    fusion_input_data.GpsAlt = gps_output->height;
    fusion_input_data.Pitch = imu_output->pitch;
    fusion_input_data.Yaw = imu_output->yaw;
    fusion_input_data.Roll = imu_output->roll;
    fusion_input_data.AbsNorthAcc = imu_output->accelerationNorth;
    fusion_input_data.AbsEastAcc = imu_output->accelerationEast;
    fusion_input_data.AbsUpAcc = imu_output->accelerationUp;
    fusion_input_data.VelNorth = gps_output->velNorth;
    fusion_input_data.VelEast = gps_output->velEast;
    fusion_input_data.VelDown = (-1) * gps_output->velUp;
    fusion_input_data.VelError = 0.05;
    fusion_input_data.AltitudeError = 2.5;
	cout << "Step.4 \n";
	doIMUGPSFusion_init(&fusion_input_data);

	gpsLast = chrono::high_resolution_clock::now();

	fusionThread = thread(goalmaxDataProcessingPeriodic);
	cout << "Step.5 \n";
	return 0;
}

int goalmaxDataProcessingPeriodic(void)
{
	bool newGpsData = false;
	cout << "Step.6 \n";

	while(runflg)
	{				
		gpsNow = chrono::high_resolution_clock::now();
	    elapsed = gpsNow - gpsLast;
		microseconds = chrono::duration_cast<chrono::microseconds>(elapsed).count();

		if (microseconds >= 200000)
		{
			gps_output = get_gps_data();			
			gpsLast = chrono::high_resolution_clock::now();
			newGpsData = true;
		}
		else if(!newGpsData)
		{
			gps_output->lat = 0;
			gps_output->lon = 0;
			gps_output->height = 0;
			gps_output->velNorth = 0;
			gps_output->velEast = 0;
			gps_output->velUp = 0;
		}
		

		imu_output = get_imu_data();

		
		if (imu_output->newImuData == true)
		{
			newGpsData = false;
			fusion_input_data.GpsLat = gps_output->lat;
			fusion_input_data.GpsLon = gps_output->lon;
			fusion_input_data.GpsAlt = gps_output->height;
			fusion_input_data.VelNorth = gps_output->velNorth;
			fusion_input_data.VelEast = gps_output->velEast;
			fusion_input_data.VelDown = (-1) * gps_output->velUp;

			fusion_input_data.Timestamp = time(nullptr);
			
			fusion_input_data.Pitch = imu_output->pitch;
			fusion_input_data.Yaw = imu_output->yaw;
			fusion_input_data.Roll = imu_output->roll;
			fusion_input_data.AbsNorthAcc = imu_output->accelerationNorth;
			fusion_input_data.AbsEastAcc = imu_output->accelerationEast;
			fusion_input_data.AbsUpAcc = imu_output->accelerationUp;			
			fusion_input_data.VelError = 0.05;
			fusion_input_data.AltitudeError = 2.5;

			fusion_output_packet = doIMUGPSFusion_periodic(&fusion_input_data);

			imu_output->newImuData = false;
		}
		
		if (hrt_updated())
		{
			float current_heart_rate = heart_get();
			//cout << "******** Heart Rate = " << current_heart_rate << " ********** \n";
			cout << "******** GPS rtkstat: " << gps_output->rtkstat << "\n";
			if((current_heart_rate > 30)&&(current_heart_rate < 200))
			{
				if(abs(current_heart_rate - last_heart_rate) < 30)
				{
					if (HeartRateList.size() == 20)
					{
						HeartRateList.erase (HeartRateList.begin());
					}
					HeartRateList.push_back(current_heart_rate);
					heartRateAverage = 0;
					for(auto beat:HeartRateList)
					{
						heartRateAverage += beat;
					}
					heartRateAverage /= HeartRateList.size();					
				}
			}
			if ((last_heart_rate == 0) && (current_heart_rate == 0))
			{
				heartRateAverage = 0;
				HeartRateList.clear();
			}
			last_heart_rate = current_heart_rate;
		}
		
		this_thread::sleep_for(chrono::milliseconds(2));		
	}
	
	cout << "Main loop end" << "\n";
	
	rtklibThread.join();
	imulibThread.join();
	
	return 0;
}

GoalmaxOutput * readGoalmaxData(void)
{
	
	goalmax_output_data.PredictedLat = fusion_output_packet->PredictedLat;
    goalmax_output_data.PredictedLon = fusion_output_packet->PredictedLon;
    goalmax_output_data.PredictedAlt = fusion_output_packet->PredictedAlt;
    goalmax_output_data.ResultantMPH = fusion_output_packet->ResultantMPH;	
	goalmax_output_data.HeartRate = heartRateAverage;
	goalmax_output_data.heading = imu_output->heading;
	goalmax_output_data.steps = imu_output->steps;
	goalmax_output_data.position = imu_output->position;
	
	return &goalmax_output_data;
}