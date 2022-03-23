#pragma once

class PID
{
public:
	float p, i, d;
	float Kp, Ki, Kd;
	float max, min, maxI, minI;
	float prevX;

	float update(float dt, float x, float dx, float target);
	PID(float Kp, float Ki, float Kd, float max, float min, float maxI, float minI);
};