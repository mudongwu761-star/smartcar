/*
 * @Author: ilikara 3435193369@qq.com
 * @Date: 2024-10-10 14:36:42
 * @LastEditors: ilikara 3435193369@qq.com
 * @LastEditTime: 2025-04-05 09:08:57
 * @FilePath: /smartcar/src/MotorController.cpp
 * @Description: 杩欐槸榛樿璁剧疆,璇疯缃甡customMade`, 鎵撳紑koroFileHeader鏌ョ湅閰嶇疆 杩涜璁剧疆: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include "MotorController1.h"

#include <iostream>
#include <chrono>

MotorController1::MotorController1(int pwmchip, int pwmnum, int gpioNum, unsigned int period_ns,
    double kp, double ki, double kd, double targetSpeed,
    int encoder_pwmNum, int encoder_gpioNum, int encoder_dir_)
    : pidController1(kp, ki, kd, targetSpeed, INCREMENTAL, 40)
    , pwmController1(pwmchip, pwmnum)
    , encoder1(encoder_pwmNum, encoder_gpioNum)
    , directionGPIO1(gpioNum)
    , encoder_dir1(encoder_dir_)
    , last_encoder_rps1(0.0)
{
    pwmController1.setPeriod(period_ns); // 璁剧疆 PWM 鍛ㄦ湡
    directionGPIO1.setDirection("out");
    pwmController1.enable(); // 鍚敤 PWM
}

MotorController1::~MotorController1(void)
{
    pwmController1.disable();
}

void MotorController1::updateduty1(double dutyCycle)
{
    int newduty = pwmController1.readPeriod() * abs(dutyCycle) / 100.0;
    if (newduty != pwmController1.readDutyCycle()) {
        pwmController1.setDutyCycle(newduty);
    }

    // 鏍规嵁 PID 杈撳嚭璁剧疆 GPIO 鐨勬柟鍚?    if (dutyCycle > 0) {
    if (dutyCycle > 0) {
        directionGPIO1.setValue(1); // 姝ｅ悜
    } else {
        directionGPIO1.setValue(0); // 鍙嶅悜
    }
    // std::cout << encoder.pulse_counter_update() << std::endl;
}

void MotorController1::updateSpeed1(void)
{
    double encoderReading = encoder1.pulse_counter_update() * encoder_dir1;
    last_encoder_rps1 = encoderReading;
    double output = pidController1.update(encoderReading);

    // 璁剧疆 PWM 鍗犵┖姣?    updateduty1(output);
    updateduty1(output);
    
    // 娣诲姞鏃堕棿鎺у埗锛屾瘡0.5绉掓樉绀轰竴娆?    static auto last_print_time = std::chrono::steady_clock::now();
    static auto last_print_time = std::chrono::steady_clock::now();
    auto current_time = std::chrono::steady_clock::now();
    auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        current_time - last_print_time).count();
    
    if (elapsed_ms >= 500) { // 500姣 = 0.5绉?        // 鑾峰彇PID涓変釜鍒嗛噺鍊?        double p_term = pidController1.getPTerm();
        double i_term = pidController1.getITerm();
        double d_term = pidController1.getDTerm();
        
     /*  std::cout << "RIGHT:" << encoderReading 
                  << " | 鐩爣:" << pidController1.getTarget()
                  << " | P椤?" << p_term 
                  << " | I椤?" << i_term 
                  << " | D椤?" << d_term
                  << " | 杈撳嚭:" << output << std::endl;*/
                  
        last_print_time = current_time;  // 鏇存柊涓婃鎵撳嵃鏃堕棿
    }
}
void MotorController1::updateTarget1(double speed)
{
    pidController1.setTarget(speed);
}

double MotorController1::getLastEncoderRps1() const
{
    return last_encoder_rps1;
}