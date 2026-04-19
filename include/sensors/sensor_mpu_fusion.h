#pragma once

float wrapAngle180(float a);
float normalize360(float a);
void updateEulerFromQuaternion();
void updateMotionEstimate(float dt);
float kalmanUpdateAxis(KalmanAxis& k, float measuredAngle, float measuredRate, float dt);
void updateOrientationFusion(float dt, float gx, float gy, float gz, float ax, float ay, float az);
