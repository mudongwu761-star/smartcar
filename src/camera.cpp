/*
 * @Author: ilikara 3435193369@qq.com
 * @Date: 2025-03-22 08:10:58
 * @LastEditors: ilikara 3435193369@qq.com
 * @LastEditTime: 2025-04-12 11:35:16
 * @FilePath: /smartcar/src/camera.cpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include "camera.h"

#include <algorithm>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <sstream>

#include "PIDController.h"
#include "PwmController.h"
#include "frame_buffer.h"
#include "global.h"
#include "vl53l0x.h"
#include "wonderEcho.h"
#include "sign_classify.h"
#include "encoder.h"

cv::VideoCapture cap;

double kp = 0;
double ki = 0;
double kd = 0;

int screenWidth, screenHeight;
int newWidth, newHeight;
int fb;
double servo_error[3] = {0,0,0};

// 创建帧缓冲区
uint16_t* fb_buffer;

bool cross_mark = 0;
bool range_mark = 0;

#define calc_scale 2


int CameraInit(uint8_t camera_id, double dest_fps, int width, int height)
{
    servo.setPeriod(3000000);
    servo.setDutyCycle(servo_mid);
    servo.enable();

    // 打开帧缓冲区设备
    fb = open("/dev/fb0", O_RDWR);
    if (fb == -1) {
        std::cerr << "无法打开帧缓冲区设备" << std::endl;
        return -1;
    }

    // 获取帧缓冲区设备信息
    struct fb_var_screeninfo vinfo;
    if (ioctl(fb, FBIOGET_VSCREENINFO, &vinfo) == -1) {
        std::cerr << "无法获取帧缓冲区信息" << std::endl;
        close(fb);
        return -1;
    }

    // 动态设置屏幕分辨率
    screenWidth = vinfo.xres;
    screenHeight = vinfo.yres;

    // 计算帧缓冲区大小
    size_t fb_size = vinfo.yres_virtual * vinfo.xres_virtual * vinfo.bits_per_pixel / 8;

    // 使用 mmap 映射帧缓冲区到内存
    fb_buffer = (uint16_t*)mmap(
    NULL,                    // 系统自动选择映射的起始地址
    fb_size,                 // 映射的大小（前面计算的帧缓冲区大小）
    PROT_READ | PROT_WRITE, // 映射区域的保护权限：可读可写
    MAP_SHARED,             // 映射类型：共享映射
    fb,                     // 要映射的文件描述符（帧缓冲设备）
    0                       // 映射起始偏移量
    );
    if (fb_buffer == MAP_FAILED) {
        std::cerr << "无法映射帧缓冲区到内存" << std::endl;
        close(fb);
        return -1;
    }

    // 打开默认摄像头（设备编号 0）
    cap.open(camera_id);

    // 检查摄像头是否成功打开
    if (!cap.isOpened()) {
        printf("无法打开摄像头\n");
        munmap(fb_buffer, fb_size);
        close(fb);
        return -1;
    }

    cap.set(cv::CAP_PROP_FRAME_WIDTH, width); // 宽度
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, height); // 高度
    cap.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G')); // 视频流格式
    cap.set(cv::CAP_PROP_AUTO_EXPOSURE, -1); // 设置自动曝光

    // 获取摄像头实际分辨率
    int cameraWidth = cap.get(cv::CAP_PROP_FRAME_WIDTH);
    int cameraHeight = cap.get(cv::CAP_PROP_FRAME_HEIGHT);
    printf("摄像头分辨率: %d x %d\n", cameraWidth, cameraHeight);

    // 计算 newWidth 和 newHeight，确保图像适应屏幕
    double widthRatio = static_cast<double>(screenWidth) / cameraWidth;
    double heightRatio = static_cast<double>(screenHeight) / cameraHeight;
    double scale = std::min(widthRatio, heightRatio); // 选择较小的比例，确保图像不超出屏幕

    newWidth = static_cast<int>(cameraWidth * scale);
    newHeight = static_cast<int>(cameraHeight * scale);
    printf("自适应分辨率: %d x %d\n", newWidth, newHeight);

    // 计算帧率
    double fps = cap.get(cv::CAP_PROP_FPS);
    printf("Camera fps:%lf\n", fps);

    line_tracking_width = newWidth / calc_scale;
    line_tracking_height = newHeight / calc_scale;

    // 计算每帧的延迟时间（ms）
    return static_cast<int>(1000.0 / std::min(fps, dest_fps));
}

void cameraDeInit(void)
{
    cap.release();

    // 获取帧缓冲区设备信息
    struct fb_var_screeninfo vinfo;
    if (ioctl(fb, FBIOGET_VSCREENINFO, &vinfo) == -1) {
        std::cerr << "无法获取帧缓冲区信息" << std::endl;
    } else {
        // 计算帧缓冲区大小
        size_t fb_size = vinfo.yres_virtual * vinfo.xres_virtual * vinfo.bits_per_pixel / 8;

        // 取消映射
        munmap(fb_buffer, fb_size);
    }

    close(fb);
}

int saved_frame_count = 0;
bool saveCameraImage(cv::Mat frame, const std::string& directory)
{
    if (frame.empty()) {
        std::cerr << "Save Error: Frame is empty." << std::endl;
        return 0;
    }

    // 构建文件名
    std::ostringstream filename;
    filename << directory           // 添加目录路径（如"./image"）
        << "/"                 // 添加路径分隔符
        << "image_"           // 添加文件名前缀
        << std::setw(5)       // 设置数字宽度为5位
        << std::setfill('0')  // 用0填充空位
        << saved_frame_count   // 添加图像序号
        << ".jpg";            // 添加文件扩展名
    saved_frame_count++;
    // 保存图像
    return cv::imwrite(filename.str(), frame);
}

std::mutex frameMutex;
cv::Mat pubframe;
bool streamCaptureRunning;
void streamCapture(void)
{
    cv::Mat frame;
    while (streamCaptureRunning) {
        cap.read(frame);
        frameMutex.lock();
        pubframe = frame;
        frameMutex.unlock();
    }
    return;
}


int CameraHandler(void)
{
    //std::cout<<std::endl;
    //std::cout<<trigger_count<<std::endl;
    
    static int frame_count = 0;
    static double total_time = 0.0;
    static double max_time = 0.0;
    static double min_time = std::numeric_limits<double>::max();
    
    auto start_time = std::chrono::high_resolution_clock::now();
    
    cv::Mat resizedFrame;

    frameMutex.lock();
    raw_frame = pubframe;
    frameMutex.unlock();

    if (raw_frame.empty()) // 检查是否成功捕获图像
    {
        printf("无法捕获图像\n");
        return -1;
    }


    { // 图像计算
        image_main();
        if (yellowAreaCount > 5) {
            if (!cross_mark) {
                cross_mark = 1;
            }
        } else {
            cross_mark = 0;
        }
    }

    // 视觉控制部分
    if (readFlag(start_file)) {
        int foresee = readDoubleFromFile(foresee_file);
        int error_count = 0;
        if (mid_line[foresee] != 255) {
            ServoControl.setPID(kp, ki, kd);
            servo_error_temp = 0;
            for (int i = line_start;i>line_end;i--)
            {
                servo_error_temp += mid_line[foresee] * calc_scale - newWidth / 2.0;
            }
            servo_error_temp = servo_error_temp/10;
            
            servo_error[2] = servo_error[1];
            servo_error[1] = servo_error[0]; 
            servo_error[0] = servo_error_temp;
        
        }
    }

    
    if (readFlag(saveImg_file)) {
        if (saveCameraImage(raw_frame, "./image")) {
            printf("图像%d已保存\n", saved_frame_count);
        } else {
            printf("图像保存失败\n");
            return -1;
        }
    }

    // 结束计时并计算
    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
    double current_ms = duration.count() / 1000.0;
    
    // 更新统计数据
    frame_count++;
    total_time += current_ms;
    max_time = std::max(max_time, current_ms);
    min_time = std::min(min_time, current_ms);
    
    // // 每100帧输出一次统计信息
    // if (frame_count % 100 == 0) {
    //     std::cout << "处理统计(ms) - "
    //               << "平均: " << total_time/frame_count 
    //               << " 最大: " << max_time 
    //               << " 最小: " << min_time << std::endl;
    // }

    // 显示图片
    
    // if (readFlag(showImg_file)) {
    //     cv::Mat fbImage(screenHeight, screenWidth, CV_8UC3, cv::Scalar(0, 0, 0));
    //     // 缩放视频到新尺寸
    //     cv::resize(binarizedFrame,resizedFrame, cv::Size(newWidth, newHeight));
    //     //convertMatToRGB565(resizedFrame,fb_buffer,screenWidth,screenHeight);
        
    //     // 将单通道的二值化图像转换为三通道的彩色图像
    //     cv::Mat coloredResizedFrame;
    //     cv::cvtColor(resizedFrame, coloredResizedFrame, cv::COLOR_GRAY2BGR); // 转换为彩色图像

    //     // 将缩放后的图像居中放置在帧缓冲区图像中，填充黑色边框
    //     fbImage.setTo(cv::Scalar(0, 0, 0)); // 清空缓冲区（填充黑色）
    //     cv::Rect roi((screenWidth - newWidth) / 2, (screenHeight - newHeight) / 2, newWidth, newHeight);
    //     coloredResizedFrame.copyTo(fbImage(roi));

    //     // 绘制左右边界线和中线
    //     int scaledLeftX, scaledRightX, scaledMidX, scaledY;

    //     for (int y = 0; y < line_tracking_height; y++) {
    //         // 根据缩放比例调整X坐标
    //         scaledLeftX = static_cast<int>(left_line[y] * calc_scale);
    //         scaledRightX = static_cast<int>(right_line[y] * calc_scale);
    //         scaledMidX = static_cast<int>(mid_line[y] * calc_scale);
    //         scaledY = static_cast<int>(y * calc_scale);

    //         // 绘制左边界（红）
    //         cv::line(fbImage(roi), cv::Point(scaledLeftX, scaledY), cv::Point(scaledLeftX, scaledY), cv::Scalar(0, 0, 255), calc_scale);
    //         // 绘制右边界（绿）
    //         cv::line(fbImage(roi), cv::Point(scaledRightX, scaledY), cv::Point(scaledRightX, scaledY), cv::Scalar(0, 255, 0), calc_scale);
    //         // 绘制中线  （蓝）
    //         cv::line(fbImage(roi), cv::Point(scaledMidX, scaledY), cv::Point(scaledMidX, scaledY), cv::Scalar(255, 0, 0), calc_scale);
    //     }

    //     // 将帧缓冲区图像转换为RGB565格式
    //     convertMatToRGB565(fbImage, fb_buffer, screenWidth, screenHeight);
    // }


    return 0;
}
