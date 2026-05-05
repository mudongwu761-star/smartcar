/*
 * @Author: ilikara 3435193369@qq.com
 * @Date: 2024-10-10 14:36:47
 * @LastEditors: ilikara 3435193369@qq.com
 * @LastEditTime: 2025-04-05 08:59:44
 * @FilePath: /smartcar/lib/MotorController.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#ifndef MOTOR_CONTROLLER1_H
#define MOTOR_CONTROLLER1_H

#include "GPIO.h"
#include "PIDController.h"
#include "PwmController.h"
#include "encoder.h"

class MotorController1 {
public:
    MotorController1(int pwmchip, int pwmnum, int gpioNum, unsigned int period_ns,
        double kp, double ki, double kd, double targetSpeed,
        int encoder_pwmNum, int encoder_gpioNum, int encoder_dir_);
    ~MotorController1(void);

    void updateSpeed1(void);
    void updateTarget1(double speed);
    void updateduty1(double dutyCycle);
    double getLastEncoderRps1() const;
    PIDController pidController1;

    ENCODER encoder1;
    int encoder_dir1;

private:
    PwmController pwmController1;
    
    GPIO directionGPIO1;
    double last_encoder_rps1;
};

#endif // MOTOR_CONTROLLER_H