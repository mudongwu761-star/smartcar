/*
 * @Author: ilikara 3435193369@qq.com
 * @Date: 2024-10-10 08:16:17
 * @LastEditors: ilikara 3435193369@qq.com
 * @LastEditTime: 2025-04-05 08:57:57
 * @FilePath: /smartcar/lib/Timer.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#ifndef TIMER_H
#define TIMER_H

#include <atomic>
#include <functional>
#include <thread>
#include "global.h"

class Timer {
public:
    Timer(int interval_ms, std::function<void()> task);
    ~Timer();

    void start();
    void stop();

private:
    void run();

    int interval; // Interval in milliseconds
    std::function<void()> task;
    std::atomic<bool> running;
    std::thread worker;
};

#endif // TIMER_H
