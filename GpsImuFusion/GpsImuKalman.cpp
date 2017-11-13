#include "GpsImuKalman.h"

GeoPoint testGeo;
sensorData * initialSensorData;
outputPacket outputInstance;
KalmanFilterFusedPositionAccelerometer longitudeEastKalmanFilter;
KalmanFilterFusedPositionAccelerometer latitudeNorthKalmanFilter;
KalmanFilterFusedPositionAccelerometer altitudeUpKalmanFilter;

double RadiansToDegrees(double radians)
{
    return radians * 180 / pi;
}

double DegreesToRadians(double degrees)
{
    return degrees * pi / 180;
}

double geoAngle(double lat_or_lon)
{
    return DegreesToRadians(lat_or_lon);
}

GeoPoint GetPointAhead(GeoPoint fromCoordinate, double distanceMeters, double azimuth)
{
    double radiusFraction = distanceMeters / EARTH_RADIUS;

	double bearing = DegreesToRadians(azimuth);

    double lat1 = geoAngle(fromCoordinate.Latitude);
    double lng1 = geoAngle(fromCoordinate.Longitude);

    double lat2_part1 = sin(lat1) * cos(radiusFraction);
    double lat2_part2 = cos(lat1) * sin(radiusFraction) * cos(bearing);

    double lat2 = asin(lat2_part1 + lat2_part2);

    double lng2_part1 = sin(bearing) * sin(radiusFraction) * cos(lat1);
    double lng2_part2 = cos(radiusFraction) - (sin(lat1) * sin(lat2));

    double lng2 = lng1 + atan2(lng2_part1, lng2_part2);
    lng2 = fmod((lng2 + 3 * pi), (2 * pi)) - pi;  // TODO (MAHX)

    GeoPoint tempGeoPoint;
    tempGeoPoint.Latitude = RadiansToDegrees(lat2);
    tempGeoPoint.Longitude = RadiansToDegrees(lng2);

    return tempGeoPoint;
}

GeoPoint PointPlusDistanceEast(GeoPoint fromCoordinate, double distance)
{
    return GetPointAhead(fromCoordinate, distance, 90.0);
}

GeoPoint PointPlusDistanceNorth(GeoPoint fromCoordinate, double distance)
{
    return GetPointAhead(fromCoordinate, distance, 0.0);
}

GeoPoint MetersToGeopoint(double latAsMeters, double lonAsMeters)
{
    GeoPoint tempPoint;
    GeoPoint pointEast;
    GeoPoint pointNorthEast;

    pointEast = PointPlusDistanceEast(tempPoint, lonAsMeters);
    pointNorthEast = PointPlusDistanceNorth(pointEast, latAsMeters);

    return pointNorthEast;
}

double GetDistanceMeters(GeoPoint fromCoordinate, GeoPoint toCoordinate)
{
    double deltaLon = geoAngle(toCoordinate.Longitude - fromCoordinate.Longitude);
    double deltaLat = geoAngle(toCoordinate.Latitude - fromCoordinate.Latitude);

    double a = pow(sin(deltaLat / 2.0), 2) + cos(geoAngle(fromCoordinate.Latitude)) *
        cos(geoAngle(toCoordinate.Latitude)) * pow(sin(deltaLon / 2.0), 2);
    double c = 2 * atan2(sqrt(a), sqrt(1.0 - a));
    return (EARTH_RADIUS * c);
}

double LatitudeToMeters(double latitude)
{
    GeoPoint temp_g_point1;
    GeoPoint temp_g_point2;

    temp_g_point1.Longitude = 0.0;
    temp_g_point1.Latitude = latitude;

    temp_g_point2.Longitude = 0.0;
    temp_g_point2.Latitude = 0.0;

    double distance = GetDistanceMeters(temp_g_point1, temp_g_point2);

    if(latitude < 0)
        distance *= -1;

    return distance;
}

double LongitudeToMeters(double longitude)
{
    GeoPoint temp_g_point1;
    GeoPoint temp_g_point2;

    temp_g_point1.Longitude = longitude;
    temp_g_point1.Latitude = 0.0;

    temp_g_point2.Longitude = 0.0;
    temp_g_point2.Latitude = 0.0;

    double distance = GetDistanceMeters(temp_g_point1, temp_g_point2);

    if(longitude < 0)
        distance *= -1;

    return distance;
}

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

KalmanFilterFusedPositionAccelerometer::KalmanFilterFusedPositionAccelerometer(void)
{
    this->currentStateTimestampSeconds = 0;
}

void KalmanFilterFusedPositionAccelerometer::recreateControlMatrix(double deltaSeconds)
{
    double dtSquared = 0.5 * deltaSeconds * deltaSeconds;
    this->B->Put(0, 0, dtSquared);
    this->B->Put(1, 0, deltaSeconds);
}

void KalmanFilterFusedPositionAccelerometer::recreateStateTransitionMatrix(double deltaSeconds)
{
    this->A->Put(0, 0, 1.0);
    this->A->Put(0, 1, deltaSeconds);

    this->A->Put(1, 0, 0.0);
    this->A->Put(1, 1, 1.0);
}

void KalmanFilterFusedPositionAccelerometer::Predict(double accelerationThisAxis, double timestampNow)
{
    double deltaTime = timestampNow - this->currentStateTimestampSeconds;

    this->recreateControlMatrix(deltaTime);
    this->recreateStateTransitionMatrix(deltaTime);

    this->u->Put(0, 0, accelerationThisAxis);

	//printf("Matrix A size: %d , %d \n", this->A->rows, this->A->columns);

	//printf("Matrix currentState size: %d , %d \n", this->currentState->rows, this->currentState->columns);

	//printf("Matrix B size: %d , %d \n", this->B->rows, this->B->columns);

	//printf("Matrix u size: %d , %d \n", this->u->rows, this->u->columns);

    this->currentState = (this->A->MultipliedBy(this->currentState))->Add(this->B->MultipliedBy(this->u));

	//printf("Matrix A size: %d , %d \n", this->A->rows, this->A->columns);

	//printf("Matrix P size: %d , %d \n", this->P->rows, this->P->columns);

	//printf("Matrix Q size: %d , %d \n", this->Q->rows, this->Q->columns);

    this->P = ((this->A->MultipliedBy(this->P))->MultipliedBy(this->A->Transpose()))->Add(this->Q);

    this->currentStateTimestampSeconds = timestampNow;
}

void KalmanFilterFusedPositionAccelerometer::Update(double position, double velocityThisAxis,
                                                    double positionError, double velocityError)
{
    this->z->Put(0, 0, position);
    this->z->Put(1, 0, velocityThisAxis);

    if(positionError != 0)
        this->R->Put(0, 0, positionError * positionError);
    //else
	//	this->R->Put(0, 0, 0);

    this->R->Put(1, 1, velocityError * velocityError);

    Matrix * y = this->z->Subtract(this->currentState);
    Matrix * s = this->P->Add(this->R);

    Matrix * sInverse;

    sInverse = s->Inverse();
    if(sInverse == 0)  // matrix has no inverse , abort
        return;

    Matrix * K = this->P->MultipliedBy(sInverse);

    this->currentState = this->currentState->Add(K->MultipliedBy(y));

    this->P = (this->I->Subtract(K))->MultipliedBy(this->P);

    /*
    above is equivalent to:
    updatedP:= k.P.Subtract (K.MultipliedBy (k.P))
    which would explain some confusion on the internets
    */
}

double KalmanFilterFusedPositionAccelerometer::GetPredictedPosition(void)
{
    return this->currentState->Get(0, 0);
}

double KalmanFilterFusedPositionAccelerometer::GetPredictedVelocityThisAxis(void)
{
    return this->currentState->Get(1, 0);
}


KalmanFilterFusedPositionAccelerometer NewKalmanFilterFusedPositionAccelerometer(double initialPosition,
    double initialVelocity, double positionStandardDeviation, double accelerometerStandardDeviation,
    double currentTimestampSeconds)
{
    Matrix * currentState = new Matrix(2, 1, false);

    currentState->Put(0, 0, initialPosition);
    currentState->Put(1, 0, initialVelocity);

    Matrix * u = new Matrix(1, 1, false);
    Matrix * z = new Matrix(2, 1, false);
    Matrix * H = new Matrix(2, 2, true);
    Matrix * P = new Matrix(2, 2, true);
    Matrix * I = new Matrix(2, 2, true);

    Matrix * Q = new Matrix(2, 2, false);
    Q->Put(0, 0, (accelerometerStandardDeviation * accelerometerStandardDeviation));
    Q->Put(1, 1, (accelerometerStandardDeviation * accelerometerStandardDeviation));

    Matrix * R = new Matrix(2, 2, false);
    R->Put(0, 0, (positionStandardDeviation * positionStandardDeviation));
    R->Put(1, 1, (positionStandardDeviation * positionStandardDeviation));

    Matrix * B = new Matrix(2, 1, false);
    Matrix * A = new Matrix(2, 2, false);

    KalmanFilterFusedPositionAccelerometer NewKalmanFilter;
    NewKalmanFilter.I = I;
    NewKalmanFilter.A = A;
    NewKalmanFilter.B = B;
    NewKalmanFilter.z = z;
    NewKalmanFilter.u = u;
    NewKalmanFilter.H = H;
    NewKalmanFilter.P = P;
    NewKalmanFilter.Q = Q;
    NewKalmanFilter.R = R;
    NewKalmanFilter.currentState = currentState;
    NewKalmanFilter.currentStateTimestampSeconds = currentTimestampSeconds;

    return NewKalmanFilter;
}



void doIMUGPSFusion_init(struct sensorData * sensor_data)
{
    initialSensorData = sensor_data;

    double latLonStandardDeviation = 2.0;  // +/- 1m, increased for safety
    double altitudeStandardDeviation = 3.518522417151836;

    // got this value by getting standard deviation from accelerometer, assuming that mean SHOULD be 0
    double accelerometerEastStandardDeviation = ACTUAL_GRAVITY * 0.033436506994600976;
    double accelerometerNorthStandardDeviation = ACTUAL_GRAVITY * 0.05355371135598354;
    double accelerometerUpStandardDeviation = ACTUAL_GRAVITY * 0.2088683796078286;

    longitudeEastKalmanFilter = NewKalmanFilterFusedPositionAccelerometer(
            LongitudeToMeters(initialSensorData->GpsLon),
            initialSensorData->VelEast,
            latLonStandardDeviation,
            accelerometerEastStandardDeviation,
            initialSensorData->Timestamp);

    latitudeNorthKalmanFilter = NewKalmanFilterFusedPositionAccelerometer(
            LatitudeToMeters(initialSensorData->GpsLat),
            initialSensorData->VelNorth,
            latLonStandardDeviation,
            accelerometerNorthStandardDeviation,
            initialSensorData->Timestamp);

    altitudeUpKalmanFilter = NewKalmanFilterFusedPositionAccelerometer(
            initialSensorData->GpsAlt,
            (initialSensorData->VelDown * (-1)),
            altitudeStandardDeviation,
            accelerometerUpStandardDeviation,
            initialSensorData->Timestamp);
}

struct outputPacket * doIMUGPSFusion_periodic(struct sensorData * sensor_data)
{
	
    longitudeEastKalmanFilter.Predict(sensor_data->AbsEastAcc * ACTUAL_GRAVITY, sensor_data->Timestamp);

    latitudeNorthKalmanFilter.Predict(sensor_data->AbsNorthAcc * ACTUAL_GRAVITY, sensor_data->Timestamp);

    altitudeUpKalmanFilter.Predict(sensor_data->AbsUpAcc * ACTUAL_GRAVITY, sensor_data->Timestamp);

    if(sensor_data->GpsLat != 0)
    {
        double defaultPositionErr = 0;
        double longitudeAsMeters = LongitudeToMeters(sensor_data->GpsLon);
        double latitudeAsMeters = LatitudeToMeters(sensor_data->GpsLat);
        double VelUp = sensor_data->VelDown * (-1);

        longitudeEastKalmanFilter.Update(
                longitudeAsMeters,
                sensor_data->VelEast,
                defaultPositionErr,
                sensor_data->VelError);

        latitudeNorthKalmanFilter.Update(
                latitudeAsMeters,
                sensor_data->VelNorth,
                defaultPositionErr,
                sensor_data->VelError);

        altitudeUpKalmanFilter.Update(
                sensor_data->GpsAlt,
                VelUp,
                sensor_data->AltitudeError,
                sensor_data->VelError);
    }

    double predictedLonMeters = longitudeEastKalmanFilter.GetPredictedPosition();
    double predictedLatMeters = latitudeNorthKalmanFilter.GetPredictedPosition();
    double predictedAlt = altitudeUpKalmanFilter.GetPredictedPosition();

    GeoPoint point = MetersToGeopoint(predictedLatMeters, predictedLonMeters);

    double predictedLon = point.Longitude;
    double predictedLat = point.Latitude;

    double predictedVE = longitudeEastKalmanFilter.GetPredictedVelocityThisAxis();
    double predictedVN = latitudeNorthKalmanFilter.GetPredictedVelocityThisAxis();

    double resultantV = sqrt(pow(predictedVE, 2) + pow(predictedVN, 2));

    //double deltaT = sensor_data.Timestamp - initialSensorData.Timestamp;

    outputInstance.PredictedLat = predictedLat;
    outputInstance.PredictedLon = predictedLon;
    outputInstance.PredictedAlt = predictedAlt;
    outputInstance.ResultantMPH = 2.23694 * resultantV;
    outputInstance.GPSLat = sensor_data->GpsLat;
    outputInstance.GPSLon = sensor_data->GpsLon;

    return &outputInstance;
}
