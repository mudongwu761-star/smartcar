/*
 * @Author: ilikara 3435193369@qq.com
 * @Date: 2024-11-29 08:11:33
 * @LastEditors: ilikara 3435193369@qq.com
 * @LastEditTime: 2025-04-05 08:57:22
 * @FilePath: /smartcar/src/video.cpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include "video.h"

Video::Video(const std::string& filename, double fps)
    : timer(static_cast<int>(1000 / fps), std::bind(&Video::streamCapture, this))
    , cap(filename)
{
    streamCapture();
}

Video::~Video(void)
{
    timer.stop();
}

void Video::streamCapture(void)
{
    frameMutex.lock();
    cap.read(frame);
    frameMutex.unlock();
}
