/*
 * stabilisation.h
 *
 *  Created on: Nov 16, 2013
 *      Author: wetzel
 */

#ifndef STABILISATION_H_
#define STABILISATION_H_

/* Kalman filter variables */
double Q_angle; // Process noise variance for the accelerometer
double Q_bias; // Process noise variance for the gyro bias
double R_measure; // Measurement noise variance - this is actually the variance of the measurement noise

double angle[3]; // The angle calculated by the Kalman filter - part of the 2x1 state matrix
double bias[3]; // The gyro bias calculated by the Kalman filter - part of the 2x1 state matrix
double rate; // Unbiased rate calculated from the rate and the calculated bias - you have to call getAngle to update the rate

double P[3][2][2]; // Error covariance matrix - This is a 2x2 matrix
double K[2]; // Kalman gain - This is a 2x1 matrix
double y; // Angle difference - 1x1 matrix
double S; // Estimate error - 1x1 matrix

double accXangle, accYangle; // Angle calculate using the accelerometer
double temp; // Temperature
double gyroXangle, gyroYangle; // Angle calculate using the gyro
double compAngleX, compAngleY; // Calculate the angle using a complementary filter
double kalAngleX, kalAngleY; // Calculate the angle using a Kalman filter
double kal[3];

void Kalman();
// The angle should be in degrees and the rate should be in degrees per second and the delta time in seconds
double getAngle(double newAngle, double newRate, double dt, int key);


#endif /* STABILISATION_H_ */
