#ifndef VL53L0X_H
#define VL53L0X_H

#include <atomic>

extern double total_distance;
extern double total_left_rotations;
extern double total_right_rotations;

void distanceMeasureInit();
void* distanceMeasureThread(void* arg);
int distanceMeasureStop();

#endif



/*#ifndef VL53L0X_H
#define VL53L0X_H

#include <atomic>      // 原子操作支持
#include <cstdint>    // 固定宽度整型
#include <fcntl.h>    // 文件控制选项
#include <stdlib.h>   // 标准库函数
#include <sys/ioctl.h> // IO控制接口
#include <unistd.h>   // UNIX标准函数
#include <cstring>  // 添加这行，用于strerror
#include <errno.h>  // 添加这行，用于errno
#include <pthread.h>  // 添加pthread头文件

#include "vl53l0x_def.h"

#define MODE_RANGE 0      // 测距模式
#define MODE_XTAKCALIB 1  // 串扰校准模式
#define MODE_OFFCALIB 2   // 偏移校准模式
#define MODE_HELP 3       // 帮助模式
#define MODE_PARAMETER 6  // 参数设置模式

//******************************** IOCTL definitions
#define VL53L0X_IOCTL_INIT _IO('p', 0x01)       // 初始化命令
#define VL53L0X_IOCTL_XTALKCALB _IOW('p', 0x02, unsigned int)  // 串扰校准
#define VL53L0X_IOCTL_OFFCALB _IOW('p', 0x03, unsigned int)    // 偏移校准
#define VL53L0X_IOCTL_STOP _IO('p', 0x05)       // 停止命令
#define VL53L0X_IOCTL_SETXTALK _IOW('p', 0x06, unsigned int)   // 设置串扰
#define VL53L0X_IOCTL_SETOFFSET _IOW('p', 0x07, int8_t)       // 设置偏移
#define VL53L0X_IOCTL_GETDATAS _IOR('p', 0x0b, VL53L0X_RangingMeasurementData_t) // 获取测距数据
#define VL53L0X_IOCTL_PARAMETER _IOWR('p', 0x0d, struct stmvl53l0x_parameter)

// modify the following macro accoring to testing set up
#define OFFSET_TARGET 100  // 偏移校准目标距离(mm)
#define XTALK_TARGET 600  // 串扰校准目标距离(mm)
#define NUM_SAMPLES 20    // 采样次数

typedef enum {
    OFFSET_PAR = 0,         // 偏移参数
    XTALKRATE_PAR = 1,      // 串扰率
    XTALKENABLE_PAR = 2,    // 串扰使能
    GPIOFUNC_PAR = 3,       // GPIO功能
    LOWTHRESH_PAR = 4,      // 低阈值
    HIGHTHRESH_PAR = 5,     // 高阈值
    DEVICEMODE_PAR = 6,     // 设备模式
    INTERMEASUREMENT_PAR = 7,// 测量间隔
    REFERENCESPADS_PAR = 8, // 参考SPAD
    REFCALIBRATION_PAR = 9, // 参考校准
} parameter_name_e;

struct stmvl53l0x_parameter {
    uint32_t is_read;      // 1:读取 0:设置
    parameter_name_e name; // 参数名
    int32_t value;        // 参数值
    int32_t value2;       // 参数值2
    int32_t status;       // 状态
};

class RangeFilter {
private:
    static const int WINDOW_SIZE = 3;  // 减小窗口大小降低内存占用
    uint16_t values[WINDOW_SIZE];
    int index = 0;
    uint32_t sum = 0;

public:
    uint16_t filter(uint16_t new_value) {
        // 更新滑动窗口和总和
        sum -= values[index];
        sum += new_value;
        values[index] = new_value;
        index = (index + 1) % WINDOW_SIZE;
        
        return sum / WINDOW_SIZE;
    }
};

extern int vl53l0xInit();  // 初始化函数
extern int vl53l0xGetData(int fd, VL53L0X_RangingMeasurementData_t* data); // 获取测距数据
extern int vl53l0xStop();  // 停止测距


// 激光控制相关的全局变量声明
extern std::atomic<bool> laser_control_active;  // 激光是否处于控制状态

// 添加线程控制变量声明
extern std::atomic<bool> thread_running;
extern int sensor_fd;
extern pthread_t read_thread;  // 添加线程句柄声明

class LaserControl {
private:
    const int obstacle_threshold_mm;   // 障碍物检测阈值
    bool is_avoiding;    // 当前是否正在避障
    bool has_avoided;    // 是否已经完成避障
    int clear_count;     // 清除计数器

public:
    // 构造函数，只需要障碍物阈值参数
    LaserControl(int threshold_mm = 1000) 
        : obstacle_threshold_mm(threshold_mm)
        , is_avoiding(false)
        , has_avoided(false)
        , clear_count(0)
    {}

    bool isAvoiding() const {
        return is_avoiding;
    }

    void update(uint16_t distance_mm);
};

#endif // VL53L0X_H*/