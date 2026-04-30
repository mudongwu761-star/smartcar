/*
 * @Author: ilikara 3435193369@qq.com
 * @Date: 2024-10-10 14:36:42
 * @LastEditors: ilikara 3435193369@qq.com
 * @LastEditTime: 2025-04-05 09:08:57
 * @FilePath: /smartcar/src/MotorController.cpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
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
{
    pwmController.setPeriod(period_ns); // 设置 PWM 周期
    directionGPIO.setDirection("out");
    pwmController.enable(); // 启用 PWM
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

    // 根据 PID 输出设置 GPIO 的方向
    if (dutyCycle > 0) {
        directionGPIO.setValue(1); // 正向
    } else {
        directionGPIO.setValue(0); // 反向
    }
    // std::cout << encoder.pulse_counter_update() << std::endl;
}

void MotorController::updateSpeed(void)
{
    double encoderReading = encoder.pulse_counter_update() * encoder_dir;
    double output = pidController.update(encoderReading);

    // 设置 PWM 占空比
    updateduty(output);
    
    // 添加时间控制，每0.5秒显示一次
    static auto last_print_time = std::chrono::steady_clock::now();
    auto current_time = std::chrono::steady_clock::now();
    auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        current_time - last_print_time).count();
    
    if (elapsed_ms >= 500) { // 500毫秒 = 0.5秒
        // 获取PID三个分量值
        double p_term = pidController.getPTerm();
        double i_term = pidController.getITerm();
        double d_term = pidController.getDTerm();
        
       /*std::cout << "LEFT:" << encoderReading 
                  << " | 目标:" << pidController.getTarget()
                  << " | P项:" << p_term 
                  << " | I项:" << i_term 
                  << " | D项:" << d_term
                  << " | 输出:" << output << std::endl;*/
                  
        last_print_time = current_time;  // 更新上次打印时间
    }
}

void MotorController::updateTarget(double speed)
{
    pidController.setTarget(speed);
}
