#ifndef GLOBAL_H
#define GLOBAL_H

#include <atomic>
#include <string>
#include "PwmController.h"
#include "PIDController.h"

const std::string kp_file = "./kp";
const std::string ki_file = "./ki";
const std::string kd_file = "./kd";

const std::string mortor_kp_file = "./mortor_kp";
const std::string mortor_ki_file = "./mortor_ki";
const std::string mortor_kd_file = "./mortor_kd";

const std::string start_file = "./start";
const std::string showImg_file = "./showImg";
const std::string destfps_file = "./destfps";
const std::string foresee_file = "./foresee";
const std::string saveImg_file = "./saveImg";
const std::string speed_file = "./speed";
const std::string servo_mid_file = "./servoMid";

extern double speed_diff_k; // 差速系数
const std::string speed_diff_k_file = "./speed_diff_k";

extern int TURN_DURATION; // 0.3s / 0.008s ≈ 37.5，取38
const std::string turn_duration_file = "./turn_duration";

// 从文件读取双精度值
double readDoubleFromFile(const std::string& filename);

// 从文件中读取标志
bool readFlag(const std::string& filename);

extern std::atomic<double> PID_rotate;
extern std::atomic<bool> pause_flag;   

extern double target_speed;
extern int servo_mid;
extern PwmController servo;

extern PIDController ServoControl;
extern double servo_error_temp;

extern int trigger_count;
extern bool trigger1_fired;
extern bool trigger2_fired;
extern bool trigger3_fired;
extern bool trigger4_fired;
extern bool trigger5_fired;
extern bool trigger6_fired;
extern bool trigger7_fired;

#endif
