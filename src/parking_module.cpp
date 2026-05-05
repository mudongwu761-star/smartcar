#include "parking_module.h"
#include <opencv2/opencv.hpp>

ParkingModule::ParkingModule() {}

void ParkingModule::setControlParams(const ParkingControlParams& params) {
    controlParams_ = params;
}

void ParkingModule::setStopDuration(double seconds) {
    controlParams_.stopDuration = seconds;
}

void ParkingModule::setFinalStopThreshold(int threshold) {
    controlParams_.finalStopThreshold = threshold;
}

bool ParkingModule::isStopped() const {
    return is_stopped_.load();
}

bool ParkingModule::isFinalStopped() const {
    return is_final_stopped_.load();
}

bool ParkingModule::isTrafficLightStopped() const {
    return is_traffic_light_stopped_.load();
}

void ParkingModule::notifyWhiteLine() {
    if (is_final_stopped_.load()) return;
    white_count_++;
    printf("[Parking] 经过白线，累计%d次\n", white_count_.load());
}

void ParkingModule::notifyZebraCrossing() {
    if (is_final_stopped_.load() || is_stopped_.load()) {
        return;
    }
    is_stopped_ = true;
    stop_start_time_ = cv::getTickCount() / cv::getTickFrequency();
    last_zebra_notified_ = true;
    printf("[Parking] 检测到斑马线，临时停车%.1f秒\n", controlParams_.stopDuration);
}

void ParkingModule::notifyTrafficLight(TrafficLightStatus status) {
    if (is_final_stopped_.load()) return;

    if (status == TrafficLightStatus::RED) {
        if (!is_traffic_light_stopped_.load()) {
            is_traffic_light_stopped_ = true;
            printf("[Parking] 检测到红灯，停车等待绿灯\n");
        }
    } else if (status == TrafficLightStatus::GREEN) {
        if (is_traffic_light_stopped_.load()) {
            is_traffic_light_stopped_ = false;
            printf("[Parking] 检测到绿灯，恢复行驶\n");
        }
    }
}

void ParkingModule::updateDistance(double current_dist) {
    current_distance_ = current_dist;
}

bool ParkingModule::checkFinalStop(int white_threshold, double stop_dist) {
    if (is_final_stopped_.load()) return true;
    if (white_count_.load() >= white_threshold && current_distance_ >= stop_dist) {
        is_final_stopped_ = true;
        printf("[Parking] 第%d次白线后行驶%.1fcm，彻底停车，比赛结束\n", white_count_.load(), current_distance_);
        return true;
    }
    return false;
}

void ParkingModule::update() {
    if (is_stopped_.load()) {
        double now = cv::getTickCount() / cv::getTickFrequency();
        if (now - stop_start_time_ >= controlParams_.stopDuration) {
            is_stopped_ = false;
            last_zebra_notified_ = false;
            printf("[Parking] 停车结束，恢复行驶\n");
        }
    }
}
