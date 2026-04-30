#include "encoder.h"
#include <iostream>
#include <iomanip>
#include <cmath>
#include <unistd.h>
#include <pthread.h>
#include "vl53l0x.h"
#include "global.h"
#include "wonderEcho.h"

// 只保留圈数的原子变量
// std::atomic<double> total_left_rotations(0.0);
// std::atomic<double> total_right_rotations(0.0);

double total_left_rotations = 0.0;
double total_right_rotations = 0.0;

static ENCODER* left_encoder = nullptr;
static ENCODER* right_encoder = nullptr;
static pthread_t measure_thread;
static std::atomic<bool> thread_running(true);

/*
 ki:0
 kd:0.4
 mortor_kp:2.4
 mortor_ki:0.8
 mortor_kd:0
------------------------------
 speed:9 (42 s)
 -------
 kp=0.16
 foresee=24

const double TRIGGER_DISTANCES[] = {
    8.5,
    25.38,
    31.5,
    158.67,
    167.68,
    183.0,
    304.0        // 新添加的触发距离
};

-------------------------------
 speed:10 (39 s)
 --------
 kp=0.165/0.17
 foresee=24

 const double TRIGGER_DISTANCES[] = {
    8.5,
    25.38,
    30.0,
    152.67,
    165.68,
    174.0,
    297.0        // 新添加的触发距离
};

-------------------------------
 speed:12.1 (34 s)
 --------
 kp=0.17
 foresee=24
 speed_diff_k=0.003

 // 修改距离触发值数组
const double TRIGGER_DISTANCES[] = {
    8.5,
    25.38,
    32.0,
    153.67,
    165.68,
    174.0,
    297.0        // 新添加的触发距离
};
*/
// 修改距离触发值数组
const double TRIGGER_DISTANCES[] = {
    10.0,
    12.38,
    30.0,
    157.5,
    160.68,
    179.5,
    298.0        // 新添加的触发距离
};

void distanceMeasureInit() {
    left_encoder = new ENCODER(3, 72);
    right_encoder = new ENCODER(0, 75);
    
    pthread_create(&measure_thread, NULL, distanceMeasureThread, NULL);
}

void* distanceMeasureThread(void* arg) {
    double left_accumulated = 0.0;
    double right_accumulated = 0.0;
    int print_counter = 0;
    
    const double ROTATION_THRESHOLD = 0.95;
    const double MIN_RPS = 0.01;    // 增大最小速度阈值
    const double MAX_RPS = 20.0;    // 添加最大速度限制
    
    usleep(100000);  // 等待100ms让编码器稳定

    while (thread_running.load()) {
        double left_rps = left_encoder->pulse_counter_update() * (-1);
        double right_rps = right_encoder->pulse_counter_update();
        
        // 添加合理性检查
        if (std::abs(left_rps) < MIN_RPS || std::abs(left_rps) > MAX_RPS) left_rps = 0.0;
        if (std::abs(right_rps) < MIN_RPS || std::abs(right_rps) > MAX_RPS) right_rps = 0.0;
        
        left_accumulated += left_rps * 0.005;
        right_accumulated += right_rps * 0.005;

        if (std::abs(left_accumulated) >= ROTATION_THRESHOLD || 
            std::abs(right_accumulated) >= ROTATION_THRESHOLD) {
            
            if (std::abs(left_accumulated) >= ROTATION_THRESHOLD) {
                // 移除原子操作，直接赋值
                total_left_rotations += left_accumulated;
                left_accumulated = 0.0;
            }
            if (std::abs(right_accumulated) >= ROTATION_THRESHOLD) {
                // 移除原子操作，直接赋值
                total_right_rotations += right_accumulated;
                right_accumulated = 0.0;
            }

            // // Display rotations - 移除.load()调用
            // std::cout << "\r左轮: " << std::fixed << std::setprecision(2) 
            //          << total_left_rotations << " 圈  右轮: " 
            //          << total_right_rotations << " 圈" << std::flush;
             

            // 检查触发点 - 移除.load()调用
            if (!trigger1_fired && total_left_rotations >= TRIGGER_DISTANCES[0]) {
                trigger1_fired = true;
                wonderEchoSend(0xff, 0x0b);
                std::cout << "\n触发点1已达到: " << total_left_rotations << " cm" << std::endl;
            }
            if (!trigger2_fired && total_left_rotations >= TRIGGER_DISTANCES[1]) {
                trigger2_fired = true;
                std::cout << "\n触发点2已达到: " << total_left_rotations << " cm" << std::endl;
            }
            if (!trigger3_fired && total_left_rotations >= TRIGGER_DISTANCES[2]) {
                trigger3_fired = true;
                std::cout << "\n触发点3已达到: " << total_left_rotations << " cm" << std::endl;
            }
            if (!trigger4_fired && total_left_rotations >= TRIGGER_DISTANCES[3]) {
                trigger4_fired = true;
                wonderEchoSend(0xff, 0x0b);
                std::cout << "\n触发点4已达到: " << total_left_rotations << " cm" << std::endl;
            }
            if (!trigger5_fired && total_left_rotations >= TRIGGER_DISTANCES[4]) {
                trigger5_fired = true;
                std::cout << "\n触发点5已达到: " << total_left_rotations << " cm" << std::endl;
            }
            if (!trigger6_fired && total_left_rotations >= TRIGGER_DISTANCES[5]) {
                trigger6_fired = true;
                std::cout << "\n触发点6已达到: " << total_left_rotations << " cm" << std::endl;
            }
            // 添加第7个触发点检查
            if (!trigger7_fired && total_left_rotations >= TRIGGER_DISTANCES[6]) {
                trigger7_fired = true;
                //std::cout << "\n触发点7已达到: " << total_left_rotations << " cm" << std::endl;
            }

            // // 定期显示输出
            // if (++print_counter >= 20) {
            //     print_counter = 0;
            //     std::cout  << "左轮: " 
            //              << total_left_rotations << " 圈  右轮: " 
            //              << total_right_rotations << " 圈" << std::endl;
            //}
        }

        usleep(5000);  // 5ms sampling interval
    }

    delete left_encoder;
    delete right_encoder;
    return NULL;
}

int distanceMeasureStop()
{
    fprintf(stderr, "Stopping distance measurement\n");

    // 停止测量线程
    if (thread_running.load()) {
        thread_running.store(false);
        pthread_join(measure_thread, NULL);
    }

    // 清理编码器资源
    if (left_encoder) {
        delete left_encoder;
        left_encoder = nullptr;
    }
    if (right_encoder) {
        delete right_encoder;
        right_encoder = nullptr;
    }

    fprintf(stderr, "Distance measurement stopped\n");
    return 0;
}

/*#include "vl53l0x.h"
#include <iostream>

// 全局变量定义
std::atomic<bool> thread_running(true);
std::atomic<bool> laser_control_active(false, std::memory_order_relaxed);
int sensor_fd = -1;
pthread_t read_thread;  // 定义线程句柄

void LaserControl::update(uint16_t distance_mm) {
    if (distance_mm < obstacle_threshold_mm) {
        // 检测到障碍物
        if (!is_avoiding && !has_avoided) {
            laser_control_active.store(true, std::memory_order_relaxed);
            is_avoiding = true;
            has_avoided = true;
            clear_count = 0;
            std::cout << "激光避障：检测到障碍物" << std::endl;
        }
    } else if (distance_mm > obstacle_threshold_mm) {
        if (is_avoiding) {
            // 正在避障且检测到无障碍，结束避障
            laser_control_active.store(false, std::memory_order_relaxed);
            is_avoiding = false;
            std::cout << "激光避障：障碍物已清除" << std::endl;
        }
        
        // 连续5次检测到无障碍，重置避障标志
        if (has_avoided && ++clear_count >= 5) {
            has_avoided = false;
            clear_count = 0;
            is_avoiding = false;
            std::cout << "激光避障：重置避障状态" << std::endl;
        }
    } else {
        clear_count = 0;  // 重置计数器
    }
}

// 修改读取线程函数
static void* read_thread_func(void* arg) {
    static LaserControl laser_control(1050);
    static RangeFilter filter;
    static int print_counter = 0;
    
    while (thread_running.load()) {
        VL53L0X_RangingMeasurementData_t data;
        if (vl53l0xGetData(sensor_fd, &data) == 0) {
            uint16_t range_value;
            if (data.SignalRateRtnMegaCps >= 20000&& 
                data.RangeStatus == 0 && 
                data.RangeMilliMeter >= 50 && 
                data.RangeMilliMeter <= 2000) {
                range_value = data.RangeMilliMeter;
            } else {
                range_value = 3150;  // 当数据无效时使用3150mm
            }
            
            uint16_t filtered_range = filter.filter(range_value);
            laser_control.update(filtered_range);

            if (++print_counter >= 5) {
                print_counter = 0;
                std::cout << "激光测距: " << filtered_range << "mm" << std::endl;
            }
        }
        usleep(200000);
    }
    return NULL;
}

int vl53l0xInit() {
    int fd;  // 设备文件描述符
    struct stmvl53l0x_parameter parameter;  // 传感器参数结构体
    unsigned int low_threshold = 0, high_threshold = 0;  // 测距阈值
    int configure_int_thresholds = 0;  // 中断阈值配置
    int gpio_functionnality_threshold = 0;  // GPIO功能配置
    
    // 2. 打开设备
    fd = open("/dev/stmvl53l0x_ranging", O_RDWR | O_SYNC);
    if (fd <= 0) {
        fprintf(stderr, "Error open stmvl53l0x_ranging device: %s\n", strerror(errno));
        return -1;
    }
    // 3. 停止可能的之前操作
    if (ioctl(fd, VL53L0X_IOCTL_STOP, NULL) < 0) {
        fprintf(stderr, "Error: Could not perform VL53L0X_IOCTL_STOP : %s\n", strerror(errno));
        close(fd);
        return -1;
    }
    switch (configure_int_thresholds) {
    case 1:
        gpio_functionnality_threshold = VL53L0X_GPIOFUNCTIONALITY_THRESHOLD_CROSSED_OUT;
        break;
    case 2:
        gpio_functionnality_threshold = VL53L0X_GPIOFUNCTIONALITY_THRESHOLD_CROSSED_LOW;
        break;
    case 3:
        gpio_functionnality_threshold = VL53L0X_GPIOFUNCTIONALITY_THRESHOLD_CROSSED_HIGH;
        break;
    default:
        gpio_functionnality_threshold = VL53L0X_GPIOFUNCTIONALITY_NEW_MEASURE_READY;
    }

    if (configure_int_thresholds) {

        parameter.is_read = 0;

        parameter.name = DEVICEMODE_PAR;
        parameter.value = VL53L0X_DEVICEMODE_CONTINUOUS_TIMED_RANGING;
        // parameter.value = VL53L0X_DEVICEMODE_CONTINUOUS_RANGING;
        if (ioctl(fd, VL53L0X_IOCTL_PARAMETER, &parameter) < 0) {
            fprintf(stderr, "Error: Could not perform VL53L0X_IOCTL_PARAMETER(CONTINOUS_TIMED_RANGING) : %s\n",
                strerror(errno));
            close(fd);
            return -1;
        }

        parameter.name = GPIOFUNC_PAR;
        parameter.value = gpio_functionnality_threshold;
        if (ioctl(fd, VL53L0X_IOCTL_PARAMETER, &parameter) < 0) {
            fprintf(stderr, "Error: Could not perform VL53L0X_IOCTL_PARAMETER : %s, low_threshold = %u\n",
                strerror(errno),
                low_threshold);
            close(fd);
            return -1;
        }

        if (configure_int_thresholds != 3) {
            parameter.name = LOWTHRESH_PAR;
            parameter.value = low_threshold;
            if (ioctl(fd, VL53L0X_IOCTL_PARAMETER, &parameter) < 0) {
                fprintf(stderr, "Error: Could not perform VL53L0X_IOCTL_PARAMETER : %s, low_threshold = %u\n",
                    strerror(errno),
                    low_threshold);
                close(fd);
                return -1;
            }
        }

        if (configure_int_thresholds != 2) {
            parameter.name = HIGHTHRESH_PAR;
            parameter.value = high_threshold;
            if (ioctl(fd, VL53L0X_IOCTL_PARAMETER, &parameter) < 0) {
                fprintf(stderr, "Error: Could not perform VL53L0X_IOCTL_PARAMETER : %s, high_threshold = %u\n",
                    strerror(errno),
                    high_threshold);
                close(fd);
                return -1;
            }
        }

    } else {
        parameter.name = DEVICEMODE_PAR;
        parameter.value = VL53L0X_DEVICEMODE_CONTINUOUS_RANGING;

        if (ioctl(fd, VL53L0X_IOCTL_PARAMETER, &parameter) < 0) {
            fprintf(stderr, "Error: Could not perform VL53L0X_IOCTL_PARAMETER(CONTINUOUS_RANGING) : %s\n",
                strerror(errno));
            close(fd);
            return -1;
        }
        parameter.name = GPIOFUNC_PAR;
        parameter.value = gpio_functionnality_threshold;
        if (ioctl(fd, VL53L0X_IOCTL_PARAMETER, &parameter) < 0) {
            fprintf(stderr, "Error: Could not perform VL53L0X_IOCTL_PARAMETER : %s, low_threshold = %u\n",
                strerror(errno),
                low_threshold);
            close(fd);
            return -1;
        }
    }
    // to init
    if (ioctl(fd, VL53L0X_IOCTL_INIT, NULL) < 0) {
        fprintf(stderr, "Error: Could not perform VL53L0X_IOCTL_INIT : %s\n", strerror(errno));
        close(fd);
        return -1;
    }

    // 启动读取线程
    sensor_fd = fd;
    thread_running = true;
    if (pthread_create(&read_thread, NULL, read_thread_func, NULL)) {
        fprintf(stderr, "Error: Could not create read thread\n");
        thread_running = false;
        close(fd);
        return -1;
    }

    return 0;
}

int vl53l0xGetData(int fd, VL53L0X_RangingMeasurementData_t* data)
{
    if (fd <= 0 || !data) {
        return -1;
    }

    if (ioctl(fd, VL53L0X_IOCTL_GETDATAS, data) < 0) {
        fprintf(stderr, "VL53L0X_IOCTL_GETDATAS failed: %s\n", strerror(errno));
        return -1;
    }

    // fprintf(stdout, "Range:%4d, Error:%u, SigRate_mcps:%7d, AmbRate_mcps:%7d\r",
    //     data->RangeMilliMeter, data->RangeStatus, data->SignalRateRtnMegaCps, data->AmbientRateRtnMegaCps);

    return 0;
}

int vl53l0xStop()
{
    int fd = sensor_fd;
    fprintf(stderr, "Stop driver\n");

    // 停止读取线程
    if (thread_running) {
        thread_running = false;
        pthread_join(read_thread, NULL);
    }

    if (ioctl(fd, VL53L0X_IOCTL_STOP, NULL) < 0) {
        fprintf(stderr, "Error: Could not perform VL53L0X_IOCTL_STOP : %s\n", strerror(errno));
        close(fd);
        return -1;
    }

    close(fd);

    return 0;
}*/