/*
 * @Author: ilikara 3435193369@qq.com
 * @Date: 2024-10-10 09:02:10
 * @LastEditors: ilikara 3435193369@qq.com
 * @LastEditTime: 2025-04-05 09:02:37
 * @FilePath: /smartcar/src/control.cpp
 * @Description:
 *
 * Copyright (c) 2024 by ${git_name_email}, All Rights Reserved.
 */
#include "control.h"

#include <chrono>
#include <iostream>
#include <thread>

#include <algorithm>

#include "GPIO.h"
#include "MotorController.h"
#include "MotorController1.h"
#include "PwmController.h"
#include "global.h"

// 在文件顶部声明全局对象
GPIO mortorEN(73);
MotorController* leftMotor = nullptr;  // 使用指针
MotorController1* rightMotor = nullptr; // 使用指针
double mortor_kp = 0;
double mortor_ki = 0;
double mortor_kd = 0;
bool motorsInitialized = false;
int STOP_DURATION = 0;

// 在文件开始处添加计数器
static int turn_right_counter = 0;
static int turn_left_counter = 0;
  // 0.3s / 0.008s ≈ 37.5，取38

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

    // 动态分配内存
    leftMotor = new MotorController(left_pwmchip, left_pwmnum, left_gpioNum, period_ns,
        mortor_kp, mortor_ki, mortor_kd, 0,
        left_encoder_pwmchip, left_encoder_gpioNum, left_encoder_dir);
    
    rightMotor = new MotorController1(right_pwmchip, right_pwmnum, right_gpioNum, period_ns,
        mortor_kp, mortor_ki, mortor_kd, 0,
        right_encoder_pwmchip, right_encoder_gpioNum, right_encoder_dir);
        
    motorsInitialized = true;
}

void ControlPause()
{
    servo.setDutyCycle(1530000);
    leftMotor->updateduty(0);
    rightMotor->updateduty1(0);
    std::cout << "mortors have paused\n";
}

void ControlMain()
{
    if (readFlag(start_file)) 
    {
        // 添加舵机控制
        if ((trigger2_fired&&(trigger_count==1))|| (trigger5_fired&&(trigger_count==4))) {
            // 右转持续0.3秒
            if (turn_right_counter < TURN_DURATION) {
                //std::cout << "motorchanged\n";
                //std::cout << turn_right_counter<<std::endl;
                //servo.setDutyCycle(servo_mid + 100000);
                turn_right_counter++;
            } else {
                trigger_count++;
                turn_right_counter = 0;  // 重置计数器
            }
        }
        else if ((trigger3_fired&&(trigger_count==2))|| (trigger6_fired&&(trigger_count==5))) {
            // 左转持续0.3秒
            if (turn_left_counter < TURN_DURATION) {
                //std::cout << "motorchanged\n";
                //std::cout << turn_left_counter<<std::endl;
                servo.setDutyCycle(servo_mid - 250000);
                turn_left_counter++;
            } else {
                trigger_count++;
                turn_left_counter = 0;  // 重置计数器
            }
        }
        else {
            // 重置计数器
            turn_right_counter = 0;
            turn_left_counter = 0;
            
            // 正常视觉控制
            double servoduty = -ServoControl.update(servo_error_temp);
            servoduty = std::clamp(servoduty, -8.0, 8.0);
            double servoduty_ns = (servoduty) / 100 * servo.readPeriod() + servo_mid;
            servo.setDutyCycle(servoduty_ns);
        }

        double judge = (servo.readDutyCycle() - 1530000.0)/1000.0;
        double left_speed=0.0;
        double right_speed=0.0;
        
        // 使用可调节的差速系数
        double s = abs(judge) * speed_diff_k; 
        s = std::min(s, 1.0); // 限制最大差速为100%

        if (judge > 1000000) {
            left_speed = target_speed;
            right_speed = target_speed * (1 - s);
        } else if (judge < -1000000) {
            left_speed = target_speed * (1 - s);
            right_speed = target_speed;
        }
        else {
            left_speed = target_speed;
            right_speed = target_speed;
        }

        if((trigger1_fired&& trigger_count==0)||(trigger4_fired&& trigger_count==3)||(trigger7_fired && trigger_count==6)){
            left_speed = 0.0;
            right_speed = 0.0;
            STOP_DURATION++;
            //std::cout<<STOP_DURATION;
            if(STOP_DURATION > 350)
            {
                STOP_DURATION = 0;
                trigger_count += 1;
            }
        }

        leftMotor->pidController.setPID(mortor_kp, mortor_ki, mortor_kd);
        leftMotor->updateTarget(left_speed);
        leftMotor->updateSpeed();
        
        rightMotor->pidController1.setPID(mortor_kp, mortor_ki, mortor_kd);
        rightMotor->updateTarget1(right_speed);
        rightMotor->updateSpeed1();
       
        // leftMotor->updateduty(target_speed);
        // rightMotor->updateduty1(target_speed);

        mortorEN.setValue(1);
    } else {
        if (motorsInitialized) {
            leftMotor->updateduty(0);
            rightMotor->updateduty1(0);
        }
        mortorEN.setValue(0);
    }
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
    
    // // 释放内存
    // if (leftMotor) {
    //     delete leftMotor;
    //     leftMotor = nullptr;
    // }
    
    // if (rightMotor) {
    //     delete rightMotor;
    //     rightMotor = nullptr;
    // }
    
    std::cout << "电机已完全停止" << std::endl;
}


//1            -0.0795
//2           -0.0455
//3            -0.0955
//4            -0.0285
//5           -0.0285
//6            0.0
//7            0.001
//8            0.0345
//9            0.0
//10           0.324
//11           1.7345
//12           3.1785
//13           4.589
//14           6.5335
//15           7.9475
//16           9.4295
//17           10.181
//18           11.505
//19           11.561
//20           13.6405
//21           14.7265
//22           15.504
//23           16.563
//24           16.933
//25           17.7895
//26           18.944
//27           19.9305
//28           20.207
//29           21.1075
//30           21.2815
//31           21.834
//32           23.6885
//33           24.206
//34           24.2605
//35           25.673
//36           25.8975
//37           26.9065
//38           27.6415
//39           28.681
//40           27.614
//41           29.4575
//42           28.105
//43           30.932
//44           30.9015
//45           30.8295
//46           30.917
//47           32.7185