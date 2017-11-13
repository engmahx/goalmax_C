#ifndef HEARTRATE
#define HEARTRATE

#include "mraa.hpp"
#include <iostream>
#include <thread>
#include <unistd.h>
#include <fstream>
#include <sys/time.h>
#include "SparkFunADS1015.h"

//#define debug_heart
#define adc_dev_ratio 	1.2
#define sample_rate 1600


int heart_init(void);
int hrt_updated(void);
float heart_get(void);

#endif //HEARTRATE
