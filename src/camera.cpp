/*
 * @Author: ilikara 3435193369@qq.com
 * @Date: 2025-03-22 08:10:58
 * @LastEditors: ilikara 3435193369@qq.com
 * @LastEditTime: 2025-04-12 11:35:16
 * @FilePath: /smartcar/src/camera.cpp
 * @Description: Camera capture, vision control and framebuffer display
 */
#include "camera.h"

#include <algorithm>
#include <chrono>
#include <iomanip>
#include <iostream>
#include <limits>
#include <mutex>
#include <sstream>
#include <thread>

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
double servo_error[3] = {0, 0, 0};

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
        NULL,
        fb_size,
        PROT_READ | PROT_WRITE,
        MAP_SHARED,
        fb,
        0
    );

    if (fb_buffer == MAP_FAILED) {
        std::cerr << "无法映射帧缓冲区到内存" << std::endl;
        close(fb);
        return -1;
    }

    // 打开默认摄像头
    cap.open(camera_id);

    // 检查摄像头是否成功打开
    if (!cap.isOpened()) {
        printf("无法打开摄像头\n");
        munmap(fb_buffer, fb_size);
        close(fb);
        return -1;
    }

    cap.set(cv::CAP_PROP_FRAME_WIDTH, width);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, height);

    // 如果你的摄像头 MJPG 不稳定，可以先注释掉这一行
    cap.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));

    cap.set(cv::CAP_PROP_AUTO_EXPOSURE, -1);

    // 获取摄像头实际分辨率
    int cameraWidth = cap.get(cv::CAP_PROP_FRAME_WIDTH);
    int cameraHeight = cap.get(cv::CAP_PROP_FRAME_HEIGHT);
    printf("摄像头分辨率: %d x %d\n", cameraWidth, cameraHeight);

    // 计算 newWidth 和 newHeight，确保图像适应屏幕
    double widthRatio = static_cast<double>(screenWidth) / cameraWidth;
    double heightRatio = static_cast<double>(screenHeight) / cameraHeight;
    double scale = std::min(widthRatio, heightRatio);

    newWidth = static_cast<int>(cameraWidth * scale);
    newHeight = static_cast<int>(cameraHeight * scale);
    printf("自适应分辨率: %d x %d\n", newWidth, newHeight);

    // 计算帧率
    double fps = cap.get(cv::CAP_PROP_FPS);
    printf("Camera fps:%lf\n", fps);

    if (fps <= 0) {
        fps = dest_fps;
    }

    line_tracking_width = newWidth / calc_scale;
    line_tracking_height = newHeight / calc_scale;

    // 计算每帧的延迟时间 ms
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
    filename << directory
             << "/"
             << "image_"
             << std::setw(5)
             << std::setfill('0')
             << saved_frame_count
             << ".jpg";

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
    int fail_count = 0;

    while (streamCaptureRunning) {
        bool ok = cap.read(frame);

        if (!ok || frame.empty()) {
            fail_count++;

            // 启动阶段偶尔空帧正常，连续失败才提示
            if (fail_count % 60 == 0) {
                std::cerr << "cap.read failed, fail_count = "
                          << fail_count
                          << ", cap.isOpened() = "
                          << cap.isOpened()
                          << std::endl;
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            continue;
        }

        fail_count = 0;

        {
            std::lock_guard<std::mutex> lock(frameMutex);
            pubframe = frame.clone();
        }
    }
}


int CameraHandler(void)
{
    static int frame_count = 0;
    static double total_time = 0.0;
    static double max_time = 0.0;
    static double min_time = std::numeric_limits<double>::max();

    static int empty_frame_count = 0;
    static bool camera_ready = false;

    auto start_time = std::chrono::high_resolution_clock::now();

    cv::Mat resizedFrame;

    {
        std::lock_guard<std::mutex> lock(frameMutex);
        raw_frame = pubframe.clone();
    }

    if (raw_frame.empty())
    {
        empty_frame_count++;

        // 刚启动时允许前几帧为空，避免误报
        if (camera_ready || empty_frame_count > 10) {
            printf("无法捕获图像\n");
        }

        return -1;
    }

    camera_ready = true;
    empty_frame_count = 0;


    // 图像计算
    {
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
        int foresee = static_cast<int>(readDoubleFromFile(foresee_file));

        foresee = std::max(0, std::min(foresee, line_tracking_height - 1));

        ServoControl.setPID(kp, ki, kd);

        double error_sum = 0.0;
        int valid_count = 0;

        // 不只使用 foresee 单行，而是使用 line_start 到 line_end 范围内的有效中线平均值
        for (int i = line_start; i >= line_end; i--) {
            if (i >= 0 &&
                i < static_cast<int>(mid_line.size()) &&
                mid_line[i] >= 0 &&
                mid_line[i] < line_tracking_width)
            {
                error_sum += mid_line[i] * calc_scale - newWidth / 2.0;
                valid_count++;
            }
        }

        if (valid_count > 0) {
            servo_error_temp = error_sum / valid_count;

            servo_error[2] = servo_error[1];
            servo_error[1] = servo_error[0];
            servo_error[0] = servo_error_temp;
        } else {
            // 当前帧没有识别到有效中线时，不让舵机乱打
            servo_error_temp = 0;
        }
    }


    // 保存图像
    if (readFlag(saveImg_file)) {
        if (saveCameraImage(raw_frame, "./image")) {
            printf("图像%d已保存\n", saved_frame_count);
        } else {
            printf("图像保存失败\n");
            return -1;
        }
    }


    // 显示图像：二值化赛道图 + 左边界 + 右边界 + 中线
    if (readFlag(showImg_file)) {
        cv::Mat fbImage(screenHeight, screenWidth, CV_8UC3, cv::Scalar(0, 0, 0));
        cv::Mat displayFrame;

        // 推荐调试阶段显示二值化图，这样最容易看清楚赛道识别是否稳定
        if (!binarizedFrame.empty()) {
            cv::resize(binarizedFrame, displayFrame, cv::Size(newWidth, newHeight));

            if (displayFrame.channels() == 1) {
                cv::cvtColor(displayFrame, displayFrame, cv::COLOR_GRAY2BGR);
            }
        } else {
            // 如果二值图为空，则退回显示原始图像
            cv::resize(raw_frame, displayFrame, cv::Size(newWidth, newHeight));

            if (displayFrame.channels() == 1) {
                cv::cvtColor(displayFrame, displayFrame, cv::COLOR_GRAY2BGR);
            } else if (displayFrame.channels() == 4) {
                cv::cvtColor(displayFrame, displayFrame, cv::COLOR_BGRA2BGR);
            }
        }

        int offsetX = (screenWidth - newWidth) / 2;
        int offsetY = (screenHeight - newHeight) / 2;

        cv::Rect roi(offsetX, offsetY, newWidth, newHeight);
        displayFrame.copyTo(fbImage(roi));

        // 绘制左右边界和中线
        // 红色：左边界
        // 绿色：右边界
        // 蓝色：中线，即期望轨迹
        for (int y = 0; y < line_tracking_height; y++) {
            int scaledY = y * calc_scale;

            if (scaledY < 0 || scaledY >= newHeight) {
                continue;
            }

            // 左边界
            if (y < static_cast<int>(left_line.size()) &&
                left_line[y] >= 0 &&
                left_line[y] < line_tracking_width)
            {
                int scaledLeftX = left_line[y] * calc_scale;

                if (scaledLeftX >= 0 && scaledLeftX < newWidth) {
                    cv::circle(
                        fbImage(roi),
                        cv::Point(scaledLeftX, scaledY),
                        1,
                        cv::Scalar(0, 0, 255),
                        -1
                    );
                }
            }

            // 右边界
            if (y < static_cast<int>(right_line.size()) &&
                right_line[y] >= 0 &&
                right_line[y] < line_tracking_width)
            {
                int scaledRightX = right_line[y] * calc_scale;

                if (scaledRightX >= 0 && scaledRightX < newWidth) {
                    cv::circle(
                        fbImage(roi),
                        cv::Point(scaledRightX, scaledY),
                        1,
                        cv::Scalar(0, 255, 0),
                        -1
                    );
                }
            }

            // 中线
            if (y < static_cast<int>(mid_line.size()) &&
                mid_line[y] >= 0 &&
                mid_line[y] < line_tracking_width)
            {
                int scaledMidX = mid_line[y] * calc_scale;

                if (scaledMidX >= 0 && scaledMidX < newWidth) {
                    cv::circle(
                        fbImage(roi),
                        cv::Point(scaledMidX, scaledY),
                        1,
                        cv::Scalar(255, 0, 0),
                        -1
                    );
                }
            }
        }

        // 写入 framebuffer
        convertMatToRGB565(fbImage, fb_buffer, screenWidth, screenHeight);
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

    // 如需调试处理耗时，可取消注释
    // if (frame_count % 100 == 0) {
    //     std::cout << "处理统计(ms) - "
    //               << "平均: " << total_time / frame_count
    //               << " 最大: " << max_time
    //               << " 最小: " << min_time << std::endl;
    // }

    return 0;
}