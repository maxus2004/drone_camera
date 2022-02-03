#pragma once

void setAngle(int angle);
void servoDisableLoop();
void cameraLoop();
void networkLoop();
int main();
void stop();
int getCameraAngle();

extern double fps;
extern double load;