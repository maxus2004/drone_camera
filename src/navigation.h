#pragma once

#include <opencv2/opencv.hpp>

void navigationInit();
void detectPosition(cv::Mat frame, float fps);
void setAutopilot(bool enabled);