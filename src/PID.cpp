#include "PID.h"
#include <opencv2/opencv.hpp>

PID::PID(float _Kp, float _Ki, float _Kd, float _max, float _min, float _maxI, float _minI){
	Kp = _Kp;
	Ki = _Ki;
	Kd = _Kd;
	max = _max;
	min = _min;
	maxI = _maxI;
	minI = _minI;
	p = 0;
	i = 0;
	d = 0;
	prevX = 0;
}

float PID::update(float dt, float x, float v, float target){
    //error
	float e = target - x;

	//proportional
	p = Kp * e;

	//integral
	i = i + Ki * dt * e;
	if (i > maxI) i = maxI;
	if (i < minI) i = minI;

	//derivative
	d = -Kd * v;

	//sum pid
	float c = p+i+d;
	if(c > max)c = max;
	if(c < min)c = min;

	//prev values
	prevX = x;

	return c;
}