/****************************************************************
GoalMax Heart rate Library
****************************************************************/
#include "mraa.hpp"
#include <iostream>
#include <unistd.h>
#include <fstream>
#include <sys/time.h>
#include "SparkFunADS1015.h"

#define debug_heart
#define adc_dev_ratio 	1.2

using namespace std;

struct timeval tv;

// Declare a variable for our i2c object. You can create an
//  arbitrary number of these, and pass them to however many
//  slave devices you wish.
mraa::I2c* adc_i2c;
ads1015 * adc;

int adc_window_size = 0;
int dev_window_size = 0;
int max_samples = 0;
float * adc_buffer;
float * dev_buffer;
float dev_threshold = 0;
int sample_rate = 0;


FILE * myfile;


extern "C" 
{
/*
 * read ten ADC samples and calculate the average of the time difference
 * return the sample rate
 */
int get_sample_rate()
{
	unsigned long long timeStamp[10];
	unsigned long long sample_time = 0;
	for (int i = 0; i < 10; i++) {
		adc->getResult(0);
		gettimeofday(&tv, NULL);
		timeStamp[i] = (uint64_t) tv.tv_sec * 1000000 + (uint64_t) tv.tv_usec;
		if (i > 0) {
			sample_time += (timeStamp[i] - timeStamp[i - 1]);
		}
	}
	sample_time = sample_time / 9;
	return (1000000/sample_time);
}


	
	/*
	 * calculate sample rate --->	Revision 1
	 * calculate ADC and deviation window sizes
	 * create ADC and deviation buffers
	 * calculate the first threshold
	 */
	void heart_init()
	{
		adc_i2c = new mraa::I2c(6);
	
		adc = new ads1015(adc_i2c, 0x48);
	
		adc->setRange(_4_096V);
		
		// *************** Debug *******************
		#ifdef debug_heart
			myfile = fopen("/home/heart/BPM_test_local.csv","w");
		#endif

		sample_rate = get_sample_rate();
	
		// Calculate ADC and deviation window sizes
		adc_window_size = sample_rate / 10;
		dev_window_size = adc_window_size * adc_dev_ratio ;
	
		// Create ADC and deviation buffers
		adc_buffer = (float *) malloc(adc_window_size*sizeof(float));
		dev_buffer = (float *) malloc(dev_window_size*sizeof(float));
	
		// Hypothetical value
		dev_threshold = 0.2;
	
		// three seconds max wait for two adjacent heart beats
		max_samples = sample_rate * 3;
	}


/*
 * Find maximum value in the ADC window
 * Find minimum value in the ADC window
 * Return ADC window maximum deviation
 */
float get_window_dev(float * buffer , int windowSize)
{
	float max_value = 0;
	float min_value = buffer[0];
	float window_dev;

	// get the max and min of the window
	for (int i = 0; i < windowSize; i++)
	{
		if (buffer[i] > max_value)
			max_value = adc_buffer[i];

		if (buffer[i] < min_value)
			min_value = adc_buffer[i];
	}

	// Calculate ADC window maximum deviation
	window_dev = max_value - min_value;

	return window_dev;
}


	/*
 	 * Buffer ADC reading window
 	 * Calculate ADC window deviation and load deviation buffer
 	 * Calculate Deviation buffer integration and load integration buffer
 	 * Find first & second beats
 	 * update deviation window threshold
 	 * Return heart rate
 	 */
	float heart_get()
	{
		int sample_index = 0;
		int beaks_found = 0;
		unsigned long long current_time_stamp = 0;
		unsigned long long time_stamp_window[3] = {0,0,0};
		unsigned long long first_beak_time = 0;
		unsigned long long second_beak_time = 0;
		float current_reading = 0;
		float adc_window_dev = 0;
		float integration_point = 0;
		float integration_window[3] = {0,0,0};
		float heart_BPM = 0;
		float threshold_buffer = 0;
		
		// *************** Debug *******************
		#ifdef debug_heart
			fprintf(myfile,"\n");
			fprintf(myfile,"5 \n");
		#endif

		while(sample_index < max_samples)
		{
			// Get ADC reading
			current_reading = adc->getResult(0);
		
			// *************** Debug *******************
			#ifdef debug_heart
				fprintf(myfile,"%f",current_reading);
			#endif

			// Get time stamp
			gettimeofday(&tv, NULL);
			current_time_stamp = (uint64_t) tv.tv_sec * 1000000 + (uint64_t) tv.tv_usec;
	
			// *************** Debug *******************
			#ifdef debug_heart
				fprintf(myfile," , %llu",current_time_stamp);
			#endif

			// Update the ADC buffer (circular)
			adc_buffer[sample_index % adc_window_size] = current_reading;
	
			// Calculate ADC window deviation and update deviation buffer after first ADC buffer
			if(sample_index > (adc_window_size - 1))
			{
				// get the maximum and minimum of the ADC window
				adc_window_dev = get_window_dev(adc_buffer, adc_window_size);
	
				// Accumulate ADC window deviation to update threshold value
				threshold_buffer += adc_window_dev;
	
				// *************** Debug *******************
				#ifdef debug_heart
					fprintf(myfile," , %f",adc_window_dev);
				#endif

				// Check threshold
				if(adc_window_dev < (dev_threshold * 1.1))
					adc_window_dev = 0;
		
				// *************** Debug *******************
				#ifdef debug_heart
					fprintf(myfile," , %f",(dev_threshold*1.1));
					fprintf(myfile," , %f",adc_window_dev);
				#endif

				// Update deviation buffer
				dev_buffer[(sample_index - adc_window_size) % dev_window_size] = adc_window_dev;
			}
	
			// Both ADC and deviation buffers are loaded
			if(sample_index > (adc_window_size + dev_window_size - 1))
			{
				integration_point = 0;
	
				// Calculate the integration of the deviation curve
				for(int i = 0; i < dev_window_size; i++)
				{
					integration_point += dev_buffer[i];
				}
	
				// *************** Debug *******************
				#ifdef debug_heart
					fprintf(myfile," , %f",integration_point);
				#endif

				// Cancel straight lines
				if(integration_window[2] != integration_point)
				{
					// Update integration and time stamp buffer (with correct order)
					integration_window[0] = integration_window[1];
					integration_window[1] = integration_window[2];
					integration_window[2] = integration_point;
	
					time_stamp_window[0] = time_stamp_window[1];
					time_stamp_window[1] = time_stamp_window[2];
					time_stamp_window[2] = current_time_stamp;
				}
			}
	
			// All ADC, deviation and integration buffers are loaded
			if(sample_index > (adc_window_size + dev_window_size + 1))
			{
				// Search for heart beat by finding beaks
				if((integration_window[0] < integration_window[1]) && (integration_window[1] > integration_window[2]))
				{
					// First heart beat
					if(beaks_found == 0)
					{
						first_beak_time = time_stamp_window[1];
		
						// *************** Debug *******************
						#ifdef debug_heart
							fprintf(myfile," , %llu",first_beak_time);
						#endif

						beaks_found = 1;
					}
					// Second heart beat
					else if(beaks_found == 1)
					{
						second_beak_time = time_stamp_window[1];
		
						// *************** Debug *******************
						#ifdef debug_heart
							fprintf(myfile," , %llu",second_beak_time);
						#endif

						// Calculate the new deviation threshold
						dev_threshold = threshold_buffer / (sample_index - adc_window_size);
	
						// Calculate heart BPM
						heart_BPM = 60000000 / (second_beak_time - first_beak_time);
	
						// *************** Debug *******************
						#ifdef debug_heart
							fprintf(myfile," , %f",heart_BPM);
						#endif

						return heart_BPM;
					}
				}
			}
		
			// *************** Debug *******************
			#ifdef debug_heart
				fprintf(myfile,"\n");
			#endif

			// Update the sample index
			sample_index++;

			// Delay till next sample
			usleep(1000000/(sample_rate*1.1));

		}
	
		// Calculate the new deviation threshold
		dev_threshold = threshold_buffer / (sample_index - adc_window_size);
	
		// Did not find to heart beats in 3 seconds return heart rate = 0
		return heart_BPM;
	}

}








































