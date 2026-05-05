#ifndef PARKING_MODULE_H_
#define PARKING_MODULE_H_

#include <atomic>

struct ParkingControlParams {
    double stopDuration = 3.0;
    int finalStopThreshold = 3;
};

enum class TrafficLightStatus {
    NONE = 0,
    RED,
    GREEN
};

class ParkingModule {
public:
    ParkingModule();
    ~ParkingModule() = default;

    void setControlParams(const ParkingControlParams& params);
    void setStopDuration(double seconds);
    void setFinalStopThreshold(int threshold);

    void notifyWhiteLine();
    void notifyZebraCrossing();
    void notifyTrafficLight(TrafficLightStatus status);
    void updateDistance(double current_dist);
    bool checkFinalStop(int white_threshold, double stop_dist);
    void update();

    bool isStopped() const;
    bool isFinalStopped() const;
    bool isTrafficLightStopped() const;

private:
    ParkingControlParams controlParams_;

    std::atomic<bool> is_stopped_{false};
    std::atomic<bool> is_final_stopped_{false};
    double stop_start_time_ = 0.0;
    std::atomic<int> zebra_count_{0};
    std::atomic<bool> last_zebra_notified_{false};
    std::atomic<int> white_count_{0};
    double current_distance_ = 0.0;

    std::atomic<bool> is_traffic_light_stopped_{false};
};

#endif
