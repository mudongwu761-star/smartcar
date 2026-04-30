/*
 * @Author: ilikara 3435193369@qq.com
 * @Date: 2024-11-29 08:11:40
 * @LastEditors: ilikara 3435193369@qq.com
 * @LastEditTime: 2025-04-05 08:57:08
 * @FilePath: /smartcar/lib/video.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#ifndef VIDEO_H_
#define VIDEO_H_

#include <fcntl.h>
#include <linux/fb.h>
#include <opencv2/opencv.hpp>
#include <sys/ioctl.h>
#include <unistd.h>

#include "Timer.h"

class Video {
public:
    Video(const std::string& filename, double fps);
    ~Video(void);

    Timer timer;
    cv::Mat frame;
    std::mutex frameMutex;

private:
    cv::VideoCapture cap;
    cv::Mat fbImage;
    cv::Mat resizedFrame;
    void streamCapture(void);
    int screenHeight, screenWidth;
    int newWidth, newHeight;
    int fb;
    uint16_t* fb_buffer;
};

#endif