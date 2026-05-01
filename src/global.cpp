/*
 * @Author: ilikara 3435193369@qq.com
 * @Date: 2024-10-11 06:23:05
 * @LastEditors: ilikara 3435193369@qq.com
 * @LastEditTime: 2025-04-05 08:50:32
 * @FilePath: /smartcar/src/global.cpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include "global.h"

#include <fstream>
#include <iostream>

double target_speed;
int servo_mid;
std::atomic<bool> pause_flag(false);  
PwmController servo(1, 0);
PIDController ServoControl(1.0, 0.0, 2.0, 0.0, POSITION, 1300000);
double servo_error_temp = 0.0;

int trigger_count = 0;

bool trigger1_fired = false;
bool trigger2_fired = false;
bool trigger3_fired = false;
bool trigger4_fired = false;
bool trigger5_fired = false;
bool trigger6_fired = false;
bool trigger7_fired = false;
// ...existing code...

// 从文件读取双精度值
double readDoubleFromFile(const std::string& filename)
{
    std::ifstream file(filename);
    double value = 0.0;
    if (file.is_open()) {
        file >> value; // 读取文件中的值
        file.close();
    } else {
        std::cerr << "Failed to open " << filename << std::endl;
    }
    return value;
}

// 从文件中读取标志
bool readFlag(const std::string& filename)
{
    std::ifstream file(filename);
    int flag = 0;
    if (file.is_open()) {
        file >> flag; // 读取文件中的更新标志
        file.close();
    } else {
        std::cerr << "Failed to open " << filename << std::endl;
    }
    return flag;
}

double speed_diff_k = 0.0005; // 默认差速系数
int TURN_DURATION = 38;
