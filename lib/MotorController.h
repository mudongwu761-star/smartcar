/*
 * @Author: ilikara 3435193369@qq.com
 * @Date: 2024-10-10 14:36:47
 * @LastEditors: ilikara 3435193369@qq.com
 * @LastEditTime: 2025-04-05 08:59:44
 * @FilePath: /smartcar/lib/MotorController.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

#include "GPIO.h"
#include "PIDController.h"
#include "PwmController.h"
#include "encoder.h"

class MotorController {
public:
    MotorController(int pwmchip, int pwmnum, int gpioNum, unsigned int period_ns,
        double kp, double ki, double kd, double targetSpeed,
        int encoder_pwmNum, int encoder_gpioNum, int encoder_dir_);
    ~MotorController(void);

    void updateSpeed(void);
    void updateTarget(double speed);
    void updateduty(double dutyCycle);
    PIDController pidController;

    int encoder_dir;
    ENCODER encoder;

private:
    PwmController pwmController;

    GPIO directionGPIO;

};

#endif // MOTOR_CONTROLLER_H
