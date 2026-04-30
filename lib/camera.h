/*
 * @Author: ilikara 3435193369@qq.com
 * @Date: 2024-10-10 08:28:56
 * @LastEditors: ilikara 3435193369@qq.com
 * @LastEditTime: 2025-04-05 09:03:25
 * @FilePath: /smartcar/lib/camera.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#ifndef CAMERA_H_
#define CAMERA_H_

#include <fcntl.h>
#include <linux/fb.h>
#include <opencv2/opencv.hpp>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>

#include "image_cv.h"

int CameraInit(uint8_t camera_id, double dest_fps, int width, int height);
int CameraHandler(void);
void streamCapture(void);
void cameraDeInit(void);

extern bool streamCaptureRunning;

extern double kp;
extern double ki;
extern double kd;

#endif