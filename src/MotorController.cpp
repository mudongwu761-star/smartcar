/*
 * @Author: ilikara 3435193369@qq.com
 * @Date: 2024-10-10 14:36:42
 * @LastEditors: ilikara 3435193369@qq.com
 * @LastEditTime: 2025-04-05 09:08:57
 * @FilePath: /smartcar/src/MotorController.cpp
 * @Description: 杩欐槸榛樿璁剧疆,璇疯缃甡customMade`, 鎵撳紑koroFileHeader鏌ョ湅閰嶇疆 杩涜璁剧疆: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include "MotorController.h"

#include <iostream>
#include <chrono>

MotorController::MotorController(int pwmchip, int pwmnum, int gpioNum, unsigned int period_ns,
    double kp, double ki, double kd, double targetSpeed,
    int encoder_pwmNum, int encoder_gpioNum, int encoder_dir_)
    : pidController(kp, ki, kd, targetSpeed, INCREMENTAL, 40)
    , pwmController(pwmchip, pwmnum)
    , encoder(encoder_pwmNum, encoder_gpioNum)
    , directionGPIO(gpioNum)
    , encoder_dir(encoder_dir_)
    , last_encoder_rps(0.0)
{
    pwmController.setPeriod(period_ns); // 璁剧疆 PWM 鍛ㄦ湡
    directionGPIO.setDirection("out");
    pwmController.enable(); // 鍚敤 PWM
}

MotorController::~MotorController(void)
{
    pwmController.disable();
}

void MotorController::updateduty(double dutyCycle)
{
    int newduty = pwmController.readPeriod() * abs(dutyCycle) / 100.0;
    if (newduty != pwmController.readDutyCycle()) {
        pwmController.setDutyCycle(newduty);
    }

    // 鏍规嵁 PID 杈撳嚭璁剧疆 GPIO 鐨勬柟鍚?    if (dutyCycle > 0) {
    if (dutyCycle > 0) {
        directionGPIO.setValue(1); // 姝ｅ悜
    } else {
        directionGPIO.setValue(0); // 鍙嶅悜
    }
    // std::cout << encoder.pulse_counter_update() << std::endl;
}

void MotorController::updateSpeed(void)
{
    double encoderReading = encoder.pulse_counter_update() * encoder_dir;
    last_encoder_rps = encoderReading;
    double output = pidController.update(encoderReading);

    // 璁剧疆 PWM 鍗犵┖姣?    updateduty(output);
    updateduty(output);
    
    // 娣诲姞鏃堕棿鎺у埗锛屾瘡0.5绉掓樉绀轰竴娆?    static auto last_print_time = std::chrono::steady_clock::now();
    static auto last_print_time = std::chrono::steady_clock::now();
    auto current_time = std::chrono::steady_clock::now();
    auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        current_time - last_print_time).count();
    
    if (elapsed_ms >= 500) { // 500姣 = 0.5绉?        // 鑾峰彇PID涓変釜鍒嗛噺鍊?        double p_term = pidController.getPTerm();
        double i_term = pidController.getITerm();
        double d_term = pidController.getDTerm();
        
       /*std::cout << "LEFT:" << encoderReading 
                  << " | 鐩爣:" << pidController.getTarget()
                  << " | P椤?" << p_term 
                  << " | I椤?" << i_term 
                  << " | D椤?" << d_term
                  << " | 杈撳嚭:" << output << std::endl;*/
                  
        last_print_time = current_time;  // 鏇存柊涓婃鎵撳嵃鏃堕棿
    }
}

void MotorController::updateTarget(double speed)
{
    pidController.setTarget(speed);
}

double MotorController::getLastEncoderRps() const
{
    return last_encoder_rps;
}