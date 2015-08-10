/*
 * kalman.h
 *
 *  Created on: 2015/04/15
 *      Author: taxio
 */

#ifndef KALMAN_H_
#define KALMAN_H_

class Kalman {
public:
	Kalman(){

		Q_angle = 0.001f;
		Q_bias = 0.003f;
		R_measure = 0.03f;

		angle = 0.0f; // Reset the angle
		bias = 0.0f; // Reset bias

		P[0][0] = 0.0f;
		P[0][1] = 0.0f;
		P[1][0] = 0.0f;
		P[1][1] = 0.0f;
	};

	float getAngle(float newAngle, float newRate, float dt){
		rate = newRate - bias;
		angle += dt * rate;

		// Update estimation error covariance - Project the error covariance ahead
		/* Step 2 */
		P[0][0] += dt * (dt*P[1][1] - P[0][1] - P[1][0] + Q_angle);
		P[0][1] -= dt * P[1][1];
		P[1][0] -= dt * P[1][1];
		P[1][1] += Q_bias * dt;

		// Discrete Kalman filter measurement update equations - Measurement Update ("Correct")
		// Calculate Kalman gain - Compute the Kalman gain
		/* Step 4 */
		float S = P[0][0] + R_measure; // Estimate error
		/* Step 5 */
		float K[2]; // Kalman gain - This is a 2x1 vector
		K[0] = P[0][0] / S;
		K[1] = P[1][0] / S;

		float y = newAngle - angle; // Angle difference
		/* Step 6 */
		angle += K[0] * y;
		bias += K[1] * y;

		// Calculate estimation error covariance - Update the error covariance
		/* Step 7 */
		float P00_temp = P[0][0];
		float P01_temp = P[0][1];

		P[0][0] -= K[0] * P00_temp;
		P[0][1] -= K[0] * P01_temp;
		P[1][0] -= K[1] * P00_temp;
		P[1][1] -= K[1] * P01_temp;

		return angle;
	};

	void setAngle(float angle){ this->angle = angle; };
	float getRate(){ return this->rate; };

	/* These are used to tune the Kalman filter */
	void setQangle(float Q_angle) { this->Q_angle = Q_angle; };
	void setQbias(float Q_bias) { this->Q_bias = Q_bias; };
	void setRmeasure(float R_measure) { this->R_measure = R_measure; };

	float getQangle() { return this->Q_angle; };
	float getQbias() { return this->Q_bias; };
	float getRmeasure() { return this->R_measure; };

private:
	/* Kalman filter variables */
	float Q_angle;
	float Q_bias;
	float R_measure;

	float angle;
	float bias;
	float rate;

	float P[2][2];
};


#endif /* KALMAN_H_ */
