/*
 * @Author: ilikara 3435193369@qq.com
 * @Date: 2024-10-10 09:02:10
 * @LastEditors: ilikara 3435193369@qq.com
 * @LastEditTime: 2025-04-05 09:02:37
 * @FilePath: /smartcar/src/control.cpp
 * @Description: 闭环视觉循迹控制：中线误差 -> 舵机PID -> 差速 -> 电机PID
 */

#include "control.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <thread>

#include "GPIO.h"
#include "MotorController.h"
#include "MotorController1.h"
#include "PwmController.h"
#include "global.h"

// 电机使能 GPIO
GPIO mortorEN(73);

// 左右电机控制器
MotorController* leftMotor = nullptr;
MotorController1* rightMotor = nullptr;

// 电机 PID 参数，由 main.cpp 周期性读取参数文件后更新
double mortor_kp = 0;
double mortor_ki = 0;
double mortor_kd = 0;

bool motorsInitialized = false;

void ControlInit()
{
    mortorEN.setDirection("out");
    mortorEN.setValue(1);

    // 左轮参数
    const int left_pwmchip = 8;
    const int left_pwmnum = 2;
    const int left_gpioNum = 12;
    const int left_encoder_pwmchip = 3;
    const int left_encoder_gpioNum = 72;
    const int left_encoder_dir = -1;

    // 右轮参数
    const int right_pwmchip = 8;
    const int right_pwmnum = 1;
    const int right_gpioNum = 13;
    const int right_encoder_pwmchip = 0;
    const int right_encoder_gpioNum = 75;
    const int right_encoder_dir = 1;

    const unsigned int period_ns = 50000; // 20 kHz

    leftMotor = new MotorController(
        left_pwmchip,
        left_pwmnum,
        left_gpioNum,
        period_ns,
        mortor_kp,
        mortor_ki,
        mortor_kd,
        0,
        left_encoder_pwmchip,
        left_encoder_gpioNum,
        left_encoder_dir
    );

    rightMotor = new MotorController1(
        right_pwmchip,
        right_pwmnum,
        right_gpioNum,
        period_ns,
        mortor_kp,
        mortor_ki,
        mortor_kd,
        0,
        right_encoder_pwmchip,
        right_encoder_gpioNum,
        right_encoder_dir
    );

    motorsInitialized = true;
}

void ControlPause()
{
    servo.setDutyCycle(servo_mid);

    if (motorsInitialized) {
        leftMotor->updateduty(0);
        rightMotor->updateduty1(0);
    }

    mortorEN.setValue(0);
    std::cout << "motors have paused\n";
}

void ControlMain()
{
    if (!readFlag(start_file)) {
        if (motorsInitialized) {
            leftMotor->updateduty(0);
            rightMotor->updateduty1(0);
        }

        mortorEN.setValue(0);
        return;
    }

    if (!motorsInitialized) {
        return;
    }

    mortorEN.setValue(1);

    /*
     * 一、舵机闭环控制
     *
     * servo_error_temp 来自视觉处理：
     * 当前识别到的赛道中线位置 - 图像中心位置
     *
     * ServoControl.update() 根据中线误差输出舵机修正量。
     * 这里不再做 trigger 特殊转向，始终按照中线 PID 控制。
     */
    double servo_percent = -ServoControl.update(servo_error_temp);

    // 舵机输出限幅。单位可以理解为占舵机周期百分比。
    servo_percent = std::clamp(servo_percent, -8.0, 8.0);

    const double servo_period = static_cast<double>(servo.readPeriod());
    const double servo_duty_ns = servo_mid + servo_percent / 100.0 * servo_period;

    servo.setDutyCycle(static_cast<unsigned int>(servo_duty_ns));

    /*
     * 二、根据舵机转向幅度做左右轮差速
     *
     * servo_percent > 0：舵机向一个方向转
     * servo_percent < 0：舵机向另一个方向转
     *
     * 具体正负对应左转还是右转，要看你们舵机安装方向。
     * 如果发现过弯时差速方向反了，只需要交换下面两个分支里的左右轮速度即可。
     */
    double left_speed = target_speed;
    double right_speed = target_speed;

    double diff_ratio = std::abs(servo_percent) * speed_diff_k;

    // 限制最大差速，避免内侧轮速度被压得过低
    diff_ratio = std::clamp(diff_ratio, 0.0, 0.7);

    if (servo_percent > 0.2) {
        // 分支 A：当前认为右轮为内侧轮
        left_speed = target_speed;
        right_speed = target_speed * (1.0 - diff_ratio);
    } else if (servo_percent < -0.2) {
        // 分支 B：当前认为左轮为内侧轮
        left_speed = target_speed * (1.0 - diff_ratio);
        right_speed = target_speed;
    } else {
        // 小误差直行
        left_speed = target_speed;
        right_speed = target_speed;
    }

    /*
     * 三、电机速度闭环控制
     *
     * 左右轮分别根据编码器反馈做 PID。
     */
    leftMotor->pidController.setPID(mortor_kp, mortor_ki, mortor_kd);
    leftMotor->updateTarget(left_speed);
    leftMotor->updateSpeed();

    rightMotor->pidController1.setPID(mortor_kp, mortor_ki, mortor_kd);
    rightMotor->updateTarget1(right_speed);
    rightMotor->updateSpeed1();
}

void ControlExit()
{
    if (motorsInitialized) {
        leftMotor->updateduty(0);
        rightMotor->updateduty1(0);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    mortorEN.setValue(0);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    std::cout << "电机已完全停止" << std::endl;
}