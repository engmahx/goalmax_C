#ifndef __GpsImuKalman_INCLUDED__
#define __GpsImuKalman_INCLUDED__

#include <math.h>
#include <stdio.h>
#include <iostream>
#include "basicMatrix.h"


const double EARTH_RADIUS = 6371 * 1000.0;  // meters

const double ACTUAL_GRAVITY = 9.80665;

const double pi  = 3.141592653589793238463;

struct GeoPoint
{
    double Latitude = 0;
    double Longitude = 0;
};

struct sensorData
{
    double Timestamp = 0;
    double GpsLat = 0;
    double GpsLon = 0;
    double GpsAlt = 0;
    double Pitch = 0;
    double Yaw = 0;
    double Roll = 0;
    double AbsNorthAcc = 0;
    double AbsEastAcc = 0;
    double AbsUpAcc = 0;
    double VelNorth = 0;
    double VelEast = 0;
    double VelDown = 0;
    double VelError = 0;
    double AltitudeError = 0;
};

struct outputPacket
{
    double PredictedLat = 0;
    double PredictedLon = 0;
    double PredictedAlt = 0;
    double ResultantMPH = 0;
    double GPSLat = 0;
    double GPSLon = 0;
};

double RadiansToDegrees(double radians);

double DegreesToRadians(double degrees);

double geoAngle(double lat_or_lon);

GeoPoint GetPointAhead(GeoPoint fromCoordinate, double distanceMeters, double azimuth);

GeoPoint PointPlusDistanceEast(GeoPoint fromCoordinate, double distance);

GeoPoint PointPlusDistanceNorth(GeoPoint fromCoordinate, double distance);

GeoPoint MetersToGeopoint(double latAsMeters, double lonAsMeters);

double GetDistanceMeters(GeoPoint fromCoordinate, GeoPoint toCoordinate);

double LatitudeToMeters(double latitude);

double LongitudeToMeters(double longitude);

/*
Although these variables aren't expressive, they're based on existing mathematical conventions
        and in reality should be completely abstract.  The variables in my own words expressed below:
H: For our usage, this should just be an identity matrix.  In practice this is meant to be
a transformation matrix to standardize inputs to the system, but I'm enforcing this in the
API itself; This should simplify usage and a bit of performance by not having to use this
P: Newest estimate for average error for each part of state. This value will evolve internally
        from the kalman filter, so initializing as an identity matrix is also acceptable
        Q: Abstractly, the process error variance.  Explicitly for our use case, this is the covariance
        matrix for the accelerometer.  To find, you can leave the accelerometer at rest and take the standard
        deviation, then square that for the variance.  Matrix would then be
[AVariance 0]
[0 AVariance]
Additionally, when computing standard deviation, in this context it would make sense to override
the mean value of the readings to be 0 to account for a blatant offset from the sensor.
R: Abstractly, the measurement error variance. Explicitly for our use case, this is the covariance
        matrix of the GPS.  If you can get the actual standard deviation of the GPS, this might work, but
if you take GPS readings at rest, you might have a GPS lock that results in extremely minimal error.
In practice, I just took the advertised +/- value from the GPS (i.e. uBlock is accurate +/- 1 meter allegedly,
        so you can use that).
u: Overridden during each prediction step; Setting as a struct attribute for performance reasons. This
        is the input matrix of high frequency sensor readings that without subject to any error would give us
        an accurate state of the world.
In our case, it's a 1x1 matrix of accelerometer input in a given direction.
z: Overridden during each prediction step; Setting as a struct attribute for performance reasongs. This
        is the input matrix of low frequency sensor readings that are absolute but presumably high standard
        deviation.
In our case, it's a 2x1 matrix of GPS position and velocity
[ P
        v ]
A: The state transition matrix. Abstractly, this is a matrix that defines a set of of equations that define what the
next step would like given no additional inputs but a "next step" (or more than likely, change in time). Given that
this struct is explicitly for fusing position and acceleration, it's:
[ 1 t
0 1 ]
To explain the above, if you have position, then its next position is the previous position + current velocity * times.
If you have velocity, then its next velocity will be the current velocity.
B: Control matrix. Given input changes to the system, this matrix multiplied by the input will present new deltas
        to the current state.  In our case, these are the equations needed to handle input acceleration.  Specifically:
[ 0.5t^2
t     ]
*/

class KalmanFilterFusedPositionAccelerometer
{
	public:
		Matrix * I;
		Matrix * H;
		Matrix * P;
		Matrix * Q;
		Matrix * R;
		Matrix * u;
		Matrix * z;
		Matrix * A;
		Matrix * B;
		Matrix * currentState;
		double currentStateTimestampSeconds;

		KalmanFilterFusedPositionAccelerometer(void);

		void recreateControlMatrix(double deltaSeconds);

		void recreateStateTransitionMatrix(double deltaSeconds);

		void Predict(double accelerationThisAxis, double timestampNow);

		void Update(double position, double velocityThisAxis, double positionError, double velocityError);

		double GetPredictedPosition(void);

		double GetPredictedVelocityThisAxis(void);
};

KalmanFilterFusedPositionAccelerometer NewKalmanFilterFusedPositionAccelerometer(double initialPosition,
    double initialVelocity, double positionStandardDeviation, double accelerometerStandardDeviation,
    double currentTimestampSeconds);

void doIMUGPSFusion_init(struct sensorData * sensor_data);

struct outputPacket * doIMUGPSFusion_periodic(struct sensorData * sensor_data);


#endif