#ifndef SIGN_CLASSIFY_H_
#define SIGN_CLASSIFY_H_

#include <opencv2/opencv.hpp>

#include "Timer.h"
#include "camera.h"
#include "global.h"
using namespace cv;
using namespace std;

void drawPic(cv::Mat, cv::Mat & , int, int, int ,int );
double getTime();
void initTemplates();
void SignProcess(const cv::Mat & );
int SignClassify();


extern bool sign_flag;

#endif