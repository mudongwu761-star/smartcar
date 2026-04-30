
---

# 1. 这个 smartcar 工程的整体组成

这个工程本质上是一个 **基于摄像头巡线 + 电机速度闭环 + 舵机转向控制 + 触发点控制** 的智能车程序。

它的主程序名是：

```cmake
smartcar_demo
```

在 `main/CMakeLists.txt` 中，工程会生成 `smartcar_demo` 可执行文件，并链接 `Threads::Threads`、`common_lib` 和 OpenCV 库。

整体结构可以理解为：

```text
smartcar
├── CMakeLists.txt          // 总构建入口
├── cross.cmake             // 龙芯 loongarch64 交叉编译工具链
├── main/
│   ├── CMakeLists.txt
│   └── main.cpp            // 主程序入口
├── src/
│   ├── camera.cpp          // 摄像头采集、图像处理调度、舵机误差计算
│   ├── image_cv.cpp        // OpenCV 图像处理、赛道中线提取
│   ├── control.cpp         // 电机、舵机、速度差速控制
│   ├── MotorController.cpp // 左电机控制类
│   ├── PwmController.cpp   // PWM sysfs 控制
│   ├── encoder.cpp         // 编码器测速
│   ├── GPIO.cpp            // GPIO sysfs 控制
│   └── global.cpp          // 全局参数与文件参数读取
└── lib/
    ├── global.h
    ├── camera.h
    ├── control.h
    ├── MotorController.h
    ├── MotorController1.h
    ├── PwmController.h
    ├── encoder.h
    ├── GPIO.h
    ├── Timer.h
    ├── vl53l0x.h
    ├── wonderEcho.h
    └── sign_classify.h
```

---

# 2. 构建系统：CMake + OpenCV + 交叉编译

顶层 `CMakeLists.txt` 里做了几件核心事情：

```cmake
include(cross.cmake)
project(smartcar_demo ...)
set(CMAKE_CXX_STANDARD 17)
find_package(Threads REQUIRED)
set(OpenCV_DIR "/home/mudong/smartcar/opencv-4.13.0/build")
find_package(OpenCV REQUIRED)
file(GLOB_RECURSE COMMON_SOURCES "src/*.cpp" "src/*.c")
add_subdirectory(main)
```

也就是说，这个工程：

1. 默认启用 `cross.cmake` 里的交叉编译；
2. 使用 C++17；
3. 依赖 OpenCV；
4. 把 `src/` 下面的 `.cpp/.c` 都编译进 `common_lib`；
5. 最后在 `main/` 下生成主程序。

`cross.cmake` 里明确写了目标平台：

```cmake
set(CMAKE_SYSTEM_PROCESSOR loongarch64)
set(TOOLCHAIN_DIR "/home/mudong/smartcar/GUN/loongson-gnu-toolchain-8.3-x86_64-loongarch64-linux-gnu-rc1.6")
set(CMAKE_CXX_COMPILER "${TOOLCHAIN_DIR}/bin/loongarch64-linux-gnu-g++")
set(CMAKE_C_COMPILER "${TOOLCHAIN_DIR}/bin/loongarch64-linux-gnu-gcc")
```

说明这套代码是面向 **LoongArch64 / 龙芯平台** 交叉编译的。

所以迁移时，第一类要改的就是：

```text
cross.cmake 里的工具链路径
CMAKE_SYSTEM_PROCESSOR
OpenCV_DIR
sysroot 路径
```

如果你是在板子上本地编译，可以把：

```cmake
option(CROSS_COMPILE "Enable cross-compilation" ON)
```

改成：

```cmake
option(CROSS_COMPILE "Enable cross-compilation" OFF)
```

或者命令行指定：

```bash
cmake .. -DCROSS_COMPILE=OFF
```

---

# 3. 主程序运行逻辑

主入口在 `main/main.cpp`。

它的流程大概是：

```cpp
读取目标帧率 destfps
初始化摄像头 CameraInit(0, dest_fps, 320, 300)
读取舵机中值 servo_mid
初始化超声波/语音/触发相关模块 wonderEchoInit()
启动摄像头采集线程 streamCapture
初始化控制系统 ControlInit()
启动 CameraTimer
启动 MortorTimer
初始化测距 distanceMeasureInit()
主循环中反复读取参数文件
退出时停止定时器、电机、摄像头、测距
```

代码中摄像头初始化分辨率写死为：

```cpp
CameraInit(0, dest_fps, 320, 300);
```

也就是摄像头设备编号 `0`，目标分辨率 `320 × 300`。

主循环每 500 ms 读取一次外部参数文件：

```cpp
target_speed = readDoubleFromFile(speed_file);
servo_mid = readDoubleFromFile(servo_mid_file);
speed_diff_k = readDoubleFromFile(speed_diff_k_file);
TURN_DURATION = readDoubleFromFile(turn_duration_file);

mortor_kp = readDoubleFromFile(mortor_kp_file);
mortor_ki = readDoubleFromFile(mortor_ki_file);
mortor_kd = readDoubleFromFile(mortor_kd_file);

kp = readDoubleFromFile(kp_file);
ki = readDoubleFromFile(ki_file);
kd = readDoubleFromFile(kd_file);
```

所以这套工程的参数不是写死在代码里，而是大量通过当前目录下的文本文件动态读取。

这些文件名在 `lib/global.h` 里定义，包括：

```cpp
./kp
./ki
./kd
./mortor_kp
./mortor_ki
./mortor_kd
./start
./showImg
./destfps
./foresee
./saveImg
./speed
./servoMid
./speed_diff_k
./turn_duration
```



这说明二次开发时，你不一定每次都要重新编译，可以直接改这些运行时参数文件。

---

# 4. 图像处理部分

图像处理主要在两个文件中：

```text
src/camera.cpp
src/image_cv.cpp
```

## 4.1 摄像头初始化

`CameraInit()` 里做了这些事：

1. 初始化舵机 PWM；
2. 打开 `/dev/fb0` 帧缓冲；
3. 打开 OpenCV 摄像头；
4. 设置摄像头宽高；
5. 设置 MJPG 格式；
6. 读取摄像头实际 FPS；
7. 根据屏幕分辨率计算缩放比例；
8. 得到巡线图像尺寸。

关键代码包括：

```cpp
cap.open(camera_id);
cap.set(cv::CAP_PROP_FRAME_WIDTH, width);
cap.set(cv::CAP_PROP_FRAME_HEIGHT, height);
cap.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
cap.set(cv::CAP_PROP_AUTO_EXPOSURE, -1);
```



这里说明新车模迁移时，如果摄像头编号、分辨率、曝光方式不一样，优先改这里。

---

## 4.2 摄像头线程

代码中有一个独立线程不断采集图像：

```cpp
void streamCapture(void)
{
    cv::Mat frame;
    while (streamCaptureRunning) {
        cap.read(frame);
        frameMutex.lock();
        pubframe = frame;
        frameMutex.unlock();
    }
}
```

主处理函数 `CameraHandler()` 再从 `pubframe` 中取最新帧。

这个设计是：

```text
摄像头采集线程：不断读图
CameraTimer 定时任务：定时处理图像
ControlTimer 定时任务：定时控制电机和舵机
```

这是比较典型的多线程/定时器架构。

---

## 4.3 巡线算法

真正的图像处理在 `src/image_cv.cpp` 里。

主流程是：

```cpp
image_main()
```

大致流程：

```text
原图 raw_frame
    ↓ resize
缩放到 line_tracking_width × line_tracking_height
    ↓ HSV
二值化
    ↓ 形态学开运算
    ↓ floodFill 找道路区域
    ↓ 逐行扫描
找每一行左边界 left_line 和右边界 right_line
    ↓
计算中线 mid_line
```

代码里使用 HSV 做二值化：

```cpp
cv::cvtColor(frame, hsvImage, cv::COLOR_BGR2HSV);
cv::threshold(hsvChannels[0], binarizedFrame, 0, 255, cv::THRESH_BINARY_INV | cv::THRESH_OTSU);
cv::threshold(hsvChannels[1], output, 0, 255, cv::THRESH_BINARY_INV | cv::THRESH_OTSU);
cv::bitwise_or(output, binarizedFrame, output);
```

然后用 `floodFill` 从图像底部中间找可行驶区域：

```cpp
cv::Point seedPoint(line_tracking_width / 2, line_tracking_height - 10);
cv::circle(morphologyExFrame, seedPoint, 10, 255, -1);
cv::floodFill(morphologyExFrame, mask, seedPoint, newVal, 0, loDiff, upDiff, 8);
```



逐行扫描后，会根据左右边界求中线：

```cpp
mid_line[row] = (left_line[row] + right_line[row]) / 2;
```



---

# 5. 舵机控制逻辑

舵机控制误差是在 `CameraHandler()` 里算的。

它读取 `foresee` 文件作为前瞻行：

```cpp
int foresee = readDoubleFromFile(foresee_file);
```

然后用中线位置相对于图像中心的偏差作为舵机误差：

```cpp
servo_error_temp += mid_line[foresee] * calc_scale - newWidth / 2.0;
```

接着在 `ControlMain()` 里用 PID 算舵机输出：

```cpp
double servoduty = -ServoControl.update(servo_error_temp);
servoduty = std::clamp(servoduty, -8.0, 8.0);
double servoduty_ns = (servoduty) / 100 * servo.readPeriod() + servo_mid;
servo.setDutyCycle(servoduty_ns);
```



所以舵机控制链路是：

```text
图像中线 mid_line
    ↓
前瞻行 foresee
    ↓
横向偏差 servo_error_temp
    ↓
ServoControl PID
    ↓
servo_mid + 修正量
    ↓
PWM 输出给舵机
```

二次开发时，舵机相关重点参数是：

```text
servo_mid
kp / ki / kd
foresee
舵机 PWM chip 和 channel
舵机周期
舵机方向
舵机限幅
```

---

# 6. 电机控制逻辑

电机控制在 `src/control.cpp` 中。

## 6.1 电机硬件参数

`ControlInit()` 中写死了左右电机的 PWM、GPIO、编码器参数：

```cpp
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
```



这几个参数是迁移新车模时最关键的硬件适配点。

含义大概是：

| 参数                                               | 作用                           |
| ------------------------------------------------ | ---------------------------- |
| `left_pwmchip` / `right_pwmchip`                 | 使用哪个 PWM 控制器                 |
| `left_pwmnum` / `right_pwmnum`                   | PWM 通道号                      |
| `left_gpioNum` / `right_gpioNum`                 | 电机方向控制 GPIO                  |
| `left_encoder_pwmchip` / `right_encoder_pwmchip` | 编码器计数使用的 PWM/计数资源            |
| `left_encoder_gpioNum` / `right_encoder_gpioNum` | 编码器方向 GPIO                   |
| `left_encoder_dir` / `right_encoder_dir`         | 编码器方向修正                      |
| `period_ns`                                      | 电机 PWM 周期，50000 ns 对应 20 kHz |

---

## 6.2 电机闭环

电机控制类 `MotorController` 内部包含：

```cpp
PIDController pidController;
PwmController pwmController;
GPIO directionGPIO;
ENCODER encoder;
```



电机速度更新逻辑是：

```cpp
double encoderReading = encoder.pulse_counter_update() * encoder_dir;
double output = pidController.update(encoderReading);
updateduty(output);
```



也就是说，这个工程不是简单开环给 PWM，而是：

```text
目标速度 target_speed
    ↓
编码器读取实际速度
    ↓
速度 PID
    ↓
输出 PWM 占空比
```

迁移新车模时，速度闭环需要重点检查：

```text
编码器线数是否仍然是 1024
编码器方向是否相反
速度单位是否一致
电机 PID 参数是否需要重调
PWM 最大输出是否安全
左右电机方向是否一致
```

---

## 6.3 差速控制

代码会根据舵机偏转量做左右轮差速：

```cpp
double judge = (servo.readDutyCycle() - 1530000.0)/1000.0;
double s = abs(judge) * speed_diff_k;
s = std::min(s, 1.0);

if (judge > 1000000) {
    left_speed = target_speed;
    right_speed = target_speed * (1 - s);
} else if (judge < -1000000) {
    left_speed = target_speed * (1 - s);
    right_speed = target_speed;
} else {
    left_speed = target_speed;
    right_speed = target_speed;
}
```



这里有一个值得注意的问题：
`judge = (servo.readDutyCycle() - 1530000.0)/1000.0`，但是后面判断用的是：

```cpp
judge > 1000000
judge < -1000000
```

这个阈值非常大，正常情况下可能很难触发差速。迁移和调试时建议重点检查这里，可能原作者想写的是：

```cpp
judge > 1000
judge < -1000
```

或者根据实际舵机 PWM 范围重新设定阈值。

---

# 7. GPIO / PWM / 编码器底层

## 7.1 GPIO

`GPIO.cpp` 使用 Linux sysfs：

```cpp
/sys/class/gpio/export
/sys/class/gpio/gpioXX/value
/sys/class/gpio/gpioXX/direction
/sys/class/gpio/gpioXX/edge
```



所以这套代码依赖传统 sysfs GPIO。新平台如果不支持 `/sys/class/gpio`，就要换成：

```text
libgpiod
厂商 GPIO SDK
直接寄存器操作
```

---

## 7.2 PWM

`PwmController.cpp` 使用：

```cpp
/sys/class/pwm/pwmchipX/export
/sys/class/pwm/pwmchipX/pwmY/period
/sys/class/pwm/pwmchipX/pwmY/duty_cycle
/sys/class/pwm/pwmchipX/pwmY/enable
/sys/class/pwm/pwmchipX/pwmY/polarity
```



迁移新车模或新板子时，要先确认：

```bash
ls /sys/class/pwm/
```

然后看实际是：

```bash
pwmchip0
pwmchip1
pwmchip2
...
```

再修改代码里的：

```cpp
PwmController servo(2, 0);
left_pwmchip = 8;
left_pwmnum = 2;
right_pwmchip = 8;
right_pwmnum = 1;
```

`servo` 的默认 PWM 在 `src/global.cpp`：

```cpp
PwmController servo(2, 0);
```



---

## 7.3 编码器

编码器使用了 `/dev/mem` 映射寄存器：

```cpp
#define PWM_BASE_ADDR 0x1611B000
#define PWM_OFFSET 0x10
```

并且速度计算里写死了：

```cpp
double value = 100000000.0 / full_buffer_value / 1024.0 * 
               (directionGPIO.readValue() * 2 - 1);
```




这里的 `1024.0` 很可能是编码器线数或每圈脉冲数相关参数。

迁移新车模时，这里必须检查：

```text
新编码器是不是 1024 线？
编码器输出是单路还是 AB 相？
方向 GPIO 是否还存在？
寄存器基地址 0x1611B000 是否仍然正确？
PWM 计数资源是否仍然能用于编码器测速？
```

如果换了开发板，这部分大概率不能直接用。

---

# 8. 触发点 / 特殊动作逻辑

`main.cpp` 和 `control.cpp` 里有很多触发变量：

```cpp
trigger1_fired
trigger2_fired
trigger3_fired
trigger4_fired
trigger5_fired
trigger6_fired
trigger7_fired
trigger_count
```

在 `ControlMain()` 里，不同触发点会执行暂停、左转、右转等逻辑：

```cpp
if ((trigger2_fired&&(trigger_count==1)) || (trigger5_fired&&(trigger_count==4))) {
    // 右转持续 TURN_DURATION
}
else if ((trigger3_fired&&(trigger_count==2)) || (trigger6_fired&&(trigger_count==5))) {
    // 左转持续 TURN_DURATION
    servo.setDutyCycle(servo_mid - 250000);
}
```



这说明这套车的任务逻辑不是纯巡线，还包含“遇到某些触发事件后执行固定动作”的流程。触发来源可能来自 `wonderEcho`、`vl53l0x`、标志识别或距离检测。

迁移新车模时，如果你的任务场景不同，可以先把这些触发逻辑屏蔽，只保留基础巡线：

```cpp
正常视觉控制
    ↓
舵机 PID
    ↓
电机速度 PID
```

等基础跑稳后，再恢复特殊动作。

---

# 9. 如果要二次开发，建议你重点改这些地方

## 第一类：编译环境相关

文件：

```text
CMakeLists.txt
cross.cmake
main/CMakeLists.txt
```

重点检查：

```text
OpenCV_DIR
工具链路径 TOOLCHAIN_DIR
CMAKE_SYSTEM_PROCESSOR
sysroot
是否交叉编译
```

现在 `OpenCV_DIR` 是绝对路径：

```cmake
set(OpenCV_DIR "/home/mudong/smartcar/opencv-4.13.0/build")
```

这在别人电脑上很容易失效。建议改成可配置方式：

```cmake
set(OpenCV_DIR "" CACHE PATH "Path to OpenCV build directory")
```

然后编译时指定：

```bash
cmake .. -DOpenCV_DIR=/你的/OpenCV/build
```

---

## 第二类：车模硬件参数

文件：

```text
src/control.cpp
src/global.cpp
src/encoder.cpp
```

需要改：

```cpp
GPIO mortorEN(73);
PwmController servo(2, 0);

left_pwmchip
left_pwmnum
left_gpioNum
left_encoder_pwmchip
left_encoder_gpioNum
left_encoder_dir

right_pwmchip
right_pwmnum
right_gpioNum
right_encoder_pwmchip
right_encoder_gpioNum
right_encoder_dir
```

建议你把这些硬件参数从 `control.cpp` 里抽出来，新建：

```text
lib/car_config.h
```

例如：

```cpp
#ifndef CAR_CONFIG_H
#define CAR_CONFIG_H

#define MOTOR_EN_GPIO 73

#define SERVO_PWMCHIP 2
#define SERVO_PWMNUM  0
#define SERVO_PERIOD_NS 3000000

#define LEFT_MOTOR_PWMCHIP 8
#define LEFT_MOTOR_PWMNUM  2
#define LEFT_MOTOR_DIR_GPIO 12
#define LEFT_ENCODER_PWMCHIP 3
#define LEFT_ENCODER_DIR_GPIO 72
#define LEFT_ENCODER_DIR -1

#define RIGHT_MOTOR_PWMCHIP 8
#define RIGHT_MOTOR_PWMNUM  1
#define RIGHT_MOTOR_DIR_GPIO 13
#define RIGHT_ENCODER_PWMCHIP 0
#define RIGHT_ENCODER_DIR_GPIO 75
#define RIGHT_ENCODER_DIR 1

#define MOTOR_PWM_PERIOD_NS 50000
#define ENCODER_PPR 1024.0

#endif
```

这样以后换车模只改一个配置文件。

---

## 第三类：摄像头参数

文件：

```text
main/main.cpp
src/camera.cpp
src/image_cv.cpp
```

当前主程序中摄像头初始化是：

```cpp
CameraInit(0, dest_fps, 320, 300);
```



迁移时你要确认：

```text
摄像头设备编号是不是 0
支持不支持 320×300
是否支持 MJPG
是否需要关闭/开启自动曝光
摄像头安装高度和角度变化后，foresee 是否要改
```

建议把：

```cpp
CameraInit(0, dest_fps, 320, 300);
```

改成从文件或配置读取，例如：

```cpp
CameraInit(camera_id, dest_fps, camera_width, camera_height);
```

---

## 第四类：巡线算法参数

文件：

```text
src/image_cv.cpp
```

要重点调：

```text
HSV 二值化方式
floodFill 种子点
line_start / line_end
foresee
黄色块检测阈值
红色检测阈值
图像缩放比例 calc_scale
```

现在 `calc_scale` 写死：

```cpp
#define calc_scale 2
```

图像处理尺寸来自：

```cpp
line_tracking_width = newWidth / calc_scale;
line_tracking_height = newHeight / calc_scale;
```



如果换更高分辨率摄像头或更强平台，可以减小缩放比例；如果算力不够，可以增大缩放比例。

---

## 第五类：控制参数

运行时参数文件包括：

```bash
kp
ki
kd
mortor_kp
mortor_ki
mortor_kd
speed
servoMid
foresee
speed_diff_k
turn_duration
start
```

这些文件要放在程序运行目录下。

例如：

```bash
echo 1 > start
echo 20 > destfps
echo 60 > speed
echo 1530000 > servoMid
echo 80 > foresee
echo 0.8 > kp
echo 0.0 > ki
echo 2.0 > kd
echo 0.5 > mortor_kp
echo 0.0 > mortor_ki
echo 0.1 > mortor_kd
echo 0.0005 > speed_diff_k
echo 38 > turn_duration
```

迁移新车模时，不建议一开始开很高速度。先：

```bash
echo 20 > speed
```

基础巡线稳定后再提高。

---

# 10. 迁移到新车模的推荐步骤

## 第一步：先只验证编译

```bash
git clone https://github.com/mudongwu761-star/smartcar.git
cd smartcar
rm -rf build
mkdir build
cd build
cmake ..
make -j$(nproc)
```

如果你之前遇到 CMakeCache 路径错误，直接删 `build` 重新生成即可。

---

## 第二步：确认 PWM 和 GPIO

在新车模/新板子上查：

```bash
ls /sys/class/pwm/
ls /sys/class/gpio/
```

手动测试 PWM：

```bash
echo 0 > /sys/class/pwm/pwmchip2/export
echo 3000000 > /sys/class/pwm/pwmchip2/pwm0/period
echo 1530000 > /sys/class/pwm/pwmchip2/pwm0/duty_cycle
echo 1 > /sys/class/pwm/pwmchip2/pwm0/enable
```

如果舵机能回中，说明 `PwmController servo(2,0)` 这组参数大概率正确。否则要换 `pwmchip` 和 `pwmnum`。

---

## 第三步：单独测试舵机

先不要跑整车，只测：

```text
servo_mid
左极限
右极限
舵机方向
```

建议确认三个值：

```text
中值：servoMid
左转：servoMid - delta
右转：servoMid + delta
```

如果方向反了，可以改：

```cpp
double servoduty = -ServoControl.update(servo_error_temp);
```

里的负号，或者在舵机输出处反向。

---

## 第四步：单独测试电机

重点检查：

```text
电机使能 GPIO：73
左电机方向 GPIO：12
右电机方向 GPIO：13
左电机 PWM：pwmchip8/pwm2
右电机 PWM：pwmchip8/pwm1
```

这些都在 `ControlInit()` 里。

如果新车模接线不同，就改这里。

先低占空比测试：

```text
左轮正转
左轮反转
右轮正转
右轮反转
双轮同向前进
双轮同向后退
```

如果某个轮子方向相反，不要改上层 PID，先改底层方向 GPIO 逻辑或方向修正参数。

---

## 第五步：单独测试编码器

编码器目前依赖：

```text
/dev/mem
PWM_BASE_ADDR 0x1611B000
1024 脉冲参数
方向 GPIO
```



新车模如果编码器不同，最可能出问题的是：

```cpp
double value = 100000000.0 / full_buffer_value / 1024.0;
```



如果新编码器不是 1024 线，需要改这个值。

---

## 第六步：只跑摄像头和图像处理

先不要让车动，先看图像处理是否正常。

重点观察：

```text
raw_frame 是否有图
binarizedFrame 是否能分出赛道
track 是否能找到可行驶区域
left_line / right_line 是否稳定
mid_line 是否在道路中间
foresee 行是否选得合适
```

如果新车模摄像头角度变了，最先调：

```bash
echo 合适的行号 > foresee
```

---

## 第七步：低速闭环

把速度设低：

```bash
echo 10 > speed
echo 1 > start
```

先只看车能不能沿线慢慢走。
如果出现问题：

| 现象       | 优先检查                   |
| -------- | ---------------------- |
| 车直接冲出去   | `speed` 太大，电机 PID 输出太猛 |
| 舵机反着打    | `servoduty` 符号或舵机接线方向  |
| 车左右剧烈摆动  | `kp` 太大或 `kd` 不合适      |
| 车反应迟钝    | `kp` 太小或 `foresee` 太远  |
| 直线跑偏     | `servoMid` 不准          |
| 左右轮速度差很大 | 编码器方向、PID、差速逻辑         |

---

# 11. 我建议你优先重构的几个点

这套代码现在可以跑，但对二次开发来说有几个地方不够清晰。

## 1. 把硬件参数集中管理

现在硬件参数散落在：

```text
src/control.cpp
src/global.cpp
src/encoder.cpp
main/main.cpp
```

建议抽到：

```text
lib/car_config.h
```

否则换车模时很容易漏改。

---

## 2. 修正差速判断阈值

当前：

```cpp
double judge = (servo.readDutyCycle() - 1530000.0)/1000.0;

if (judge > 1000000) {
    ...
} else if (judge < -1000000) {
    ...
}
```

这个判断阈值很可疑。建议你打印：

```cpp
std::cout << "judge = " << judge << std::endl;
```

看实际范围，再把阈值改成合理值。

---

## 3. 电机类不应分成 MotorController 和 MotorController1

代码里左轮用：

```cpp
MotorController
```

右轮用：

```cpp
MotorController1
```

这通常说明左右轮代码重复了一份。更好的方式是保留一个 `MotorController`，通过参数区分左右轮。

---

## 4. 参数文件要有默认值检查

现在如果文件不存在，`readDoubleFromFile()` 会返回 `0.0`，这可能很危险。

比如 `servoMid` 文件不存在时，舵机中值会变成 `0`，可能导致舵机异常动作。

建议改成：

```cpp
double readDoubleFromFile(const std::string& filename, double default_value)
```

文件不存在时返回默认值。

---

## 5. 编码器参数不要写死 1024

建议改成：

```cpp
#define ENCODER_PPR 1024.0
```

或者放到 `car_config.h` 里。

---

# 12. 最关键的迁移修改清单

如果你要把这套代码迁移到新的车模上，最低限度要检查这些：

```text
1. cross.cmake
   - 工具链路径
   - 目标架构
   - sysroot

2. CMakeLists.txt
   - OpenCV_DIR

3. main/main.cpp
   - CameraInit(0, dest_fps, 320, 300)
   - 摄像头编号
   - 摄像头分辨率

4. src/global.cpp
   - PwmController servo(2, 0)
   - ServoControl 默认 PID
   - speed_diff_k
   - TURN_DURATION

5. src/control.cpp
   - mortorEN GPIO 73
   - 左电机 PWM/GPIO/编码器参数
   - 右电机 PWM/GPIO/编码器参数
   - 电机 PWM 周期 50000 ns
   - 差速逻辑
   - 特殊触发逻辑

6. src/encoder.cpp / lib/encoder.h
   - PWM_BASE_ADDR
   - 编码器线数 1024
   - 编码器方向 GPIO
   - /dev/mem 权限

7. src/image_cv.cpp
   - 二值化方式
   - HSV 阈值
   - floodFill 种子点
   - foresee 前瞻行
   - line_start / line_end
```

结论：
这套工程的核心是 **OpenCV 巡线 + 舵机位置 PID + 左右电机编码器速度 PID + 文件动态调参 + 特殊触发动作**。如果只是换新车模但开发板不变，主要改 `src/control.cpp`、`src/global.cpp`、`main/main.cpp` 和运行时参数文件；如果开发板也变了，那么 `GPIO/PWM/encoder` 底层接口和 `cross.cmake` 也要大改。
