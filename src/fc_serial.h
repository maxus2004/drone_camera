#pragma once

void init_serial();
cv::Vec4f fc_getRotation();
void sendControl(float yaw, float pitch, float roll, float thrust);

struct telemetry_t{
    cv::Vec4f rotation;
    cv::Vec3f target;
};

extern telemetry_t telemetry;

struct controlPacket_t{
	uint8_t header[8];
	float yaw;
	float pitch;
	float roll;
	float thrust;
};