/*
 * stabilisation.c
 *
 *  Created on: Nov 16, 2013
 *      Author: wetzel
 */

#include "stabilisation.h"

void Kalman() {
	/* We will set the varibles like so, these can also be tuned by the user */
	Q_angle = 0.001;
	Q_bias = 0.003;
	R_measure = 0.03;
	int i = 0;
	for (; i < 3; i++) {
		bias[i] = 0; // Reset bias
		P[i][0][0] = 0; // Since we assume tha the bias is 0 and we know the starting angle (use setAngle), the error covariance matrix is set like so - see: http://en.wikipedia.org/wiki/Kalman_filter#Example_application.2C_technical
		P[i][0][1] = 0;
		P[i][1][0] = 0;
		P[i][1][1] = 0;
		kal[i] = 0;
	}
}
// The angle should be in degrees and the rate should be in degrees per second and the delta time in seconds
double getAngle(double newAngle, double newRate, double dt, int key) {
	// KasBot V2  -  Kalman filter module - http://www.x-firm.com/?page_id=145
	// Modified by Kristian Lauszus
	// See my blog post for more information: http://blog.tkjelectronics.dk/2012/09/a-practical-approach-to-kalman-filter-and-how-to-implement-it

	// Discrete Kalman filter time update equations - Time Update ("Predict")
	// Update xhat - Project the state ahead
	/* Step 1 */
	rate = newRate - bias[key];
	angle[key] += dt * rate;

	// Update estimation error covariance - Project the error covariance ahead
	/* Step 2 */
	P[key][0][0] += dt
			* (dt * P[key][1][1] - P[key][0][1] - P[key][1][0] + Q_angle);
	P[key][0][1] -= dt * P[key][1][1];
	P[key][1][0] -= dt * P[key][1][1];
	P[key][1][1] += Q_bias * dt;

	// Discrete Kalman filter measurement update equations - Measurement Update ("Correct")
	// Calculate Kalman gain - Compute the Kalman gain
	/* Step 4 */
	S = P[key][0][0] + R_measure;
	/* Step 5 */
	K[0] = P[key][0][0] / S;
	K[1] = P[key][1][0] / S;

	// Calculate angle and bias - Update estimate with measurement zk (newAngle)
	/* Step 3 */
	y = newAngle - angle[key];
	/* Step 6 */
	angle[key] += K[0] * y;
	bias[key] += K[1] * y;

	// Calculate estimation error covariance - Update the error covariance
	/* Step 7 */
	P[key][0][0] -= K[0] * P[key][0][0];
	P[key][0][1] -= K[0] * P[key][0][1];
	P[key][1][0] -= K[1] * P[key][0][0];
	P[key][1][1] -= K[1] * P[key][0][1];

	return angle[key];
}



