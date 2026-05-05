/*
 * @Author: ilikara 3435193369@qq.com
 * @Date: 2024-10-10 09:02:10
 * @LastEditors: ilikara 3435193369@qq.com
 * @LastEditTime: 2025-04-05 09:02:37
 * @FilePath: /smartcar/src/control.cpp
 * @Description: 闂幆瑙嗚寰抗鎺у埗锛氫腑绾胯宸?-> 鑸垫満PID -> 宸€?-> 鐢垫満PID
 */

#include "control.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <thread>

#include "GPIO.h"
#include "MotorController.h"
#include "MotorController1.h"
#include "PwmController.h"
#include "global.h"

// 鐢垫満浣胯兘 GPIO
GPIO mortorEN(73);

// 宸﹀彸鐢垫満鎺у埗鍣?MotorController* leftMotor = nullptr;
MotorController* leftMotor = nullptr;
MotorController1* rightMotor = nullptr;

// 鐢垫満 PID 鍙傛暟锛岀敱 main.cpp 鍛ㄦ湡鎬ц鍙栧弬鏁版枃浠跺悗鏇存柊
double mortor_kp = 0;
double mortor_ki = 0;
double mortor_kd = 0;

bool motorsInitialized = false;

namespace {
constexpr double kControlPeriodSeconds = 0.008;
constexpr double kWheelCircumferenceCm = 20.0;
    constexpr double kTurnTriggerStartCm = 645.0;
    constexpr double kTurnTriggerEndCm = 650.0;
    constexpr double kLeftTurnServoOffsetNs = 100000.0;
constexpr double kTurnInnerWheelRatio = 0.83;
constexpr double kMinValidRps = 0.01;
constexpr double kMaxValidRps = 20.0;
constexpr bool kEnableDistanceLeftTurn = true;
constexpr int kDistancePrintTicks = 63;

enum class DistanceTurnState {
    WaitingDistance,
    TurningLeft,
    Done
};

DistanceTurnState distanceTurnState = DistanceTurnState::WaitingDistance;
double traveledDistanceCm = 0.0;
double previousTraveledDistanceCm = 0.0;
int leftTurnTicksRemaining = 0;
int distancePrintTicks = 0;

double sanitizeEncoderRps(double rps)
{
    if (std::isnan(rps) || std::isinf(rps)) {
        return 0.0;
    }

    const double abs_rps = std::abs(rps);
    if (abs_rps < kMinValidRps || abs_rps > kMaxValidRps) {
        return 0.0;
    }

    return abs_rps;
}

void resetDistanceTurnState()
{
    distanceTurnState = DistanceTurnState::WaitingDistance;
    traveledDistanceCm = 0.0;
    previousTraveledDistanceCm = 0.0;
    leftTurnTicksRemaining = 0;
    distancePrintTicks = 0;
}

void updateTraveledDistance()
{
    if (!motorsInitialized || leftMotor == nullptr || rightMotor == nullptr) {
        return;
    }

    const double left_rps = sanitizeEncoderRps(leftMotor->getLastEncoderRps());
    const double right_rps = sanitizeEncoderRps(rightMotor->getLastEncoderRps1());
    const double average_rps = (left_rps + right_rps) * 0.5;

    previousTraveledDistanceCm = traveledDistanceCm;
    traveledDistanceCm += average_rps * kWheelCircumferenceCm * kControlPeriodSeconds;

    ++distancePrintTicks;
    if (distancePrintTicks >= kDistancePrintTicks) {
        distancePrintTicks = 0;
        std::cout << "Distance: " << std::fixed << std::setprecision(1)
                  << traveledDistanceCm << " cm" << std::endl;
    }
}
}

void ControlInit()
{
    mortorEN.setDirection("out");
    mortorEN.setValue(1);

    // 宸﹁疆鍙傛暟
    const int left_pwmchip = 8;
    const int left_pwmnum = 2;
    const int left_gpioNum = 12;
    const int left_encoder_pwmchip = 3;
    const int left_encoder_gpioNum = 72;
    const int left_encoder_dir = -1;

    // 鍙宠疆鍙傛暟
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

void ResetTraveledDistance()
{
    resetDistanceTurnState();
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
        resetDistanceTurnState();

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

    if (g_parking.isFinalStopped()) {
        if (motorsInitialized) {
            leftMotor->updateTarget(0);
            rightMotor->updateTarget1(0);
            leftMotor->pidController.setPID(2.0, 0.1, 0.5);
            rightMotor->pidController1.setPID(2.0, 0.1, 0.5);
            leftMotor->updateSpeed();
            rightMotor->updateSpeed1();
        }
        mortorEN.setValue(0);
        return;
    }

    bool need_brake = g_parking.isStopped() || g_parking.isTrafficLightStopped();
    if (need_brake) {
        if (motorsInitialized) {
            leftMotor->updateTarget(0);
            rightMotor->updateTarget1(0);
            leftMotor->pidController.setPID(8.0, 0.2, 1.0);
            rightMotor->pidController1.setPID(8.0, 0.2, 1.0);
            leftMotor->updateSpeed();
            rightMotor->updateSpeed1();
        }
        servo.setDutyCycle(servo_mid);
        return;
    }

    updateTraveledDistance();
    g_parking.updateDistance(traveledDistanceCm);

    if (g_parking.checkFinalStop(3, 150.0)) {
        if (motorsInitialized) {
            leftMotor->updateTarget(0);
            rightMotor->updateTarget1(0);
            leftMotor->pidController.setPID(2.0, 0.1, 0.5);
            rightMotor->pidController1.setPID(2.0, 0.1, 0.5);
            leftMotor->updateSpeed();
            rightMotor->updateSpeed1();

        }
        mortorEN.setValue(0);
        return;
    }

    if (kEnableDistanceLeftTurn &&
        distanceTurnState == DistanceTurnState::WaitingDistance &&
        previousTraveledDistanceCm <= kTurnTriggerEndCm &&
        traveledDistanceCm >= kTurnTriggerStartCm) {
        distanceTurnState = DistanceTurnState::TurningLeft;
        leftTurnTicksRemaining = std::max(1, TURN_DURATION);
        std::cout << "Distance reached: " << traveledDistanceCm



                  << " cm, start slight left turn." << std::endl;
    }

    /*
     * 涓€銆佽埖鏈洪棴鐜帶鍒?     *
     * servo_error_temp 鏉ヨ嚜瑙嗚澶勭悊锛?     * 褰撳墠璇嗗埆鍒扮殑璧涢亾涓嚎浣嶇疆 - 鍥惧儚涓績浣嶇疆
     *
     * ServoControl.update() 鏍规嵁涓嚎璇樊杈撳嚭鑸垫満淇閲忋€?     * 杩欓噷涓嶅啀鍋?trigger 鐗规畩杞悜锛屽缁堟寜鐓т腑绾?PID 鎺у埗銆?     */
    double servo_percent = -ServoControl.update(servo_error_temp);

    // 鑸垫満杈撳嚭闄愬箙銆傚崟浣嶅彲浠ョ悊瑙ｄ负鍗犺埖鏈哄懆鏈熺櫨鍒嗘瘮銆?    servo_percent = std::clamp(servo_percent, -8.0, 8.0);
    servo_percent = std::clamp(servo_percent, -8.0, 8.0);

    const double servo_period = static_cast<double>(servo.readPeriod());
    const double servo_duty_ns = servo_mid + servo_percent / 100.0 * servo_period;

    servo.setDutyCycle(static_cast<unsigned int>(servo_duty_ns));

    /*
     * 浜屻€佹牴鎹埖鏈鸿浆鍚戝箙搴﹀仛宸﹀彸杞樊閫?     *
     * servo_percent > 0锛氳埖鏈哄悜涓€涓柟鍚戣浆
     * servo_percent < 0锛氳埖鏈哄悜鍙︿竴涓柟鍚戣浆
     *
     * 鍏蜂綋姝ｈ礋瀵瑰簲宸﹁浆杩樻槸鍙宠浆锛岃鐪嬩綘浠埖鏈哄畨瑁呮柟鍚戙€?     * 濡傛灉鍙戠幇杩囧集鏃跺樊閫熸柟鍚戝弽浜嗭紝鍙渶瑕佷氦鎹笅闈袱涓垎鏀噷鐨勫乏鍙宠疆閫熷害鍗冲彲銆?     */
    double left_speed = target_speed;
    double right_speed = target_speed;

    if (distanceTurnState == DistanceTurnState::TurningLeft) {
        const double servo_duty_ns = servo_mid - kLeftTurnServoOffsetNs;
        servo.setDutyCycle(static_cast<unsigned int>(servo_duty_ns));

        left_speed = target_speed * kTurnInnerWheelRatio;
        right_speed = target_speed;

        --leftTurnTicksRemaining;
        if (leftTurnTicksRemaining <= 0) {
            distanceTurnState = DistanceTurnState::Done;
            servo.setDutyCycle(servo_mid);
            std::cout << "Slight left turn finished, resume line tracking." << std::endl;
        }

        leftMotor->pidController.setPID(mortor_kp, mortor_ki, mortor_kd);
        leftMotor->updateTarget(left_speed);
        leftMotor->updateSpeed();

        rightMotor->pidController1.setPID(mortor_kp, mortor_ki, mortor_kd);
        rightMotor->updateTarget1(right_speed);
        rightMotor->updateSpeed1();
        return;
    }

    double diff_ratio = std::abs(servo_percent) * speed_diff_k;

    // 闄愬埗鏈€澶у樊閫燂紝閬垮厤鍐呬晶杞€熷害琚帇寰楄繃浣?    diff_ratio = std::clamp(diff_ratio, 0.0, 0.7);
    diff_ratio = std::clamp(diff_ratio, 0.0, 0.7);

    if (servo_percent > 0.2) {
        // 鍒嗘敮 A锛氬綋鍓嶈涓哄彸杞负鍐呬晶杞?        left_speed = target_speed;
        left_speed = target_speed;
        right_speed = target_speed * (1.0 - diff_ratio);
    } else if (servo_percent < -0.2) {
        // 鍒嗘敮 B锛氬綋鍓嶈涓哄乏杞负鍐呬晶杞?        left_speed = target_speed * (1.0 - diff_ratio);
        left_speed = target_speed * (1.0 - diff_ratio);
        right_speed = target_speed;
    } else {
        // 灏忚宸洿琛?        left_speed = target_speed;
        left_speed = target_speed;
        right_speed = target_speed;
    }

    /*
     * 涓夈€佺數鏈洪€熷害闂幆鎺у埗
     *
     * 宸﹀彸杞垎鍒牴鎹紪鐮佸櫒鍙嶉鍋?PID銆?     */
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

    std::cout << "Motors stopped." << std::endl;
}