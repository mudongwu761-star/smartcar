/*
 * @Author: ilikara 3435193369@qq.com
 * @Date: 2024-10-29 12:23:20
 * @LastEditors: ilikara 3435193369@qq.com
 * @LastEditTime: 2025-04-05 09:01:52
 * @FilePath: /smartcar/lib/frame_buffer.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#ifndef _FRAME_BUFFER_H
#define _FRAME_BUFFER_H

#include <opencv2/opencv.hpp>

void convertMatToRGB565(const cv::Mat& frame, uint16_t* buffer, int width, int height);

#endif