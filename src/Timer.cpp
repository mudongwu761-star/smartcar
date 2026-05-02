#include "Timer.h"

#include <chrono>

Timer::Timer(int interval_ms, std::function<void()> task)
    : interval(interval_ms)
    , task(task)
    , running(false)
{
}

Timer::~Timer()
{
    stop();
}

void Timer::start()
{
    running = true;
    worker = std::thread(&Timer::run, this);
}

void Timer::stop()
{
    running = false;
    if (worker.joinable()) {
        worker.join();
    }
}

void Timer::run()
{
    using clock = std::chrono::steady_clock;

    auto next_time = clock::now();

    while (running) {
        next_time += std::chrono::milliseconds(interval);

        if (running) {
            task();
        }

        std::this_thread::sleep_until(next_time);
    }
}