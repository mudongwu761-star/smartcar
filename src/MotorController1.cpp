/*
 * @Author: ilikara 3435193369@qq.com
 * @Date: 2024-10-10 14:36:42
 * @LastEditors: ilikara 3435193369@qq.com
 * @LastEditTime: 2025-04-05 09:08:57
 * @FilePath: /smartcar/src/MotorController.cpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
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
{
    pwmController1.setPeriod(period_ns); // 设置 PWM 周期
    directionGPIO1.setDirection("out");
    pwmController1.enable(); // 启用 PWM
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

    // 根据 PID 输出设置 GPIO 的方向
    if (dutyCycle > 0) {
        directionGPIO1.setValue(1); // 正向
    } else {
        directionGPIO1.setValue(0); // 反向
    }
    // std::cout << encoder.pulse_counter_update() << std::endl;
}

void MotorController1::updateSpeed1(void)
{
    double encoderReading = encoder1.pulse_counter_update() * encoder_dir1;
    double output = pidController1.update(encoderReading);

    // 设置 PWM 占空比
    updateduty1(output);
    
    // 添加时间控制，每0.5秒显示一次
    static auto last_print_time = std::chrono::steady_clock::now();
    auto current_time = std::chrono::steady_clock::now();
    auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        current_time - last_print_time).count();
    
    if (elapsed_ms >= 500) { // 500毫秒 = 0.5秒
        // 获取PID三个分量值
        double p_term = pidController1.getPTerm();
        double i_term = pidController1.getITerm();
        double d_term = pidController1.getDTerm();
        
     /*  std::cout << "RIGHT:" << encoderReading 
                  << " | 目标:" << pidController1.getTarget()
                  << " | P项:" << p_term 
                  << " | I项:" << i_term 
                  << " | D项:" << d_term
                  << " | 输出:" << output << std::endl;*/
                  
        last_print_time = current_time;  // 更新上次打印时间
    }
}
void MotorController1::updateTarget1(double speed)
{
    pidController1.setTarget(speed);
}
