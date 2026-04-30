/*
 * @Author: ilikara 3435193369@qq.com
 * @Date: 2024-10-29 12:23:05
 * @LastEditors: ilikara 3435193369@qq.com
 * @LastEditTime: 2025-04-05 09:02:03
 * @FilePath: /smartcar/src/frame_buffer.cpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include "frame_buffer.h"

// 将RGB转换为RGB565格式
uint16_t convertRGBToRGB565(uint8_t r, uint8_t g, uint8_t b)
{
    return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
}

// 将Mat图像转换为RGB565格式
void convertMatToRGB565(const cv::Mat& frame, uint16_t* buffer, int width, int height)
{
    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            cv::Vec3b color = frame.at<cv::Vec3b>(y, x);
            buffer[y * width + x] = convertRGBToRGB565(color[2], color[1], color[0]);
        }
    }
}