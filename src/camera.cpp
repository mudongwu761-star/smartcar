/*
 * @Author: ilikara 3435193369@qq.com
 * @Date: 2025-03-22 08:10:58
 * @LastEditors: ilikara 3435193369@qq.com
 * @LastEditTime: 2025-04-12 11:35:16
 * @FilePath: /smartcar/src/camera.cpp
 * @Description: Camera capture, vision control, lightweight element detection and framebuffer display
 */

#include "camera.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <limits>
#include <mutex>
#include <sstream>
#include <thread>
#include <vector>

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


/*
 * =========================
 * 轻量级元素识别参数
 * =========================
 *
 * 目标：
 * 1. 不影响闭环寻迹；
 * 2. 只打印识别结果；
 * 3. 不停车、不改速度、不发语音。
 *
 * 本版修改重点：
 * - 根据现场数据，普通白线 max ratio 接近 1.00，斑马线 max ratio 接近 0.75；
 * - 收窄斑马线的 partial-ratio 上限，减少普通亮斑/边缘误检；
 * - 增加斑马线连续确认帧数，过滤偶发误检点；
 * - 斑马线候选出现时，暂时不报白线，避免同一目标被重复分类。
 */
static constexpr int ELEMENT_WHITE_THRESHOLD = 180;
static constexpr int ELEMENT_DETECT_INTERVAL = 3;
static constexpr double ELEMENT_PRINT_COOLDOWN_SEC = 1.5;
static constexpr bool ELEMENT_DEBUG_PRINT = true;

/*
 * 根据你的实测数据：
 * 白线：   wMaxRatio ≈ 1.00
 * 斑马线： wMaxRatio ≈ 0.75
 */
static constexpr double WHITE_LINE_MIN_RATIO = 0.90;
static constexpr double ZEBRA_PARTIAL_MIN_RATIO = 0.50;
static constexpr double ZEBRA_PARTIAL_MAX_RATIO = 0.88;   // 斑马线检测不稳定时先放宽上限；若误检增多再降到 0.84/0.82
static constexpr double ZEBRA_MIN_TRANSITIONS = 1.80;
static constexpr int ZEBRA_MIN_BAND_HEIGHT = 8;
static constexpr int ZEBRA_CONFIRM_FRAMES = 2;            // 保持 2 帧确认；通过提高检测频率来提升稳定性

struct ElementDebugInfo {
    double max_white_ratio = 0.0;
    double avg_transitions = 0.0;
    int stripe_count = 0;
    int band_count = 0;
    int max_band_height = 0;
};

/*
 * 统计某一行中的白色像素比例。
 */
static double whiteRatioOnRow(const cv::Mat& gray, int y, int x_start, int x_end, int step)
{
    if (gray.empty() || y < 0 || y >= gray.rows || x_start >= x_end) {
        return 0.0;
    }

    int white_count = 0;
    int sample_count = 0;

    for (int x = x_start; x < x_end; x += step) {
        if (x < 0 || x >= gray.cols) {
            continue;
        }

        if (gray.at<uchar>(y, x) > ELEMENT_WHITE_THRESHOLD) {
            white_count++;
        }

        sample_count++;
    }

    if (sample_count <= 0) {
        return 0.0;
    }

    return static_cast<double>(white_count) / static_cast<double>(sample_count);
}

/*
 * 统计某一列上的黑白跳变次数。
 */
static int transitionCountOnColumn(const cv::Mat& gray, int x, int y_start, int y_end, int step)
{
    if (gray.empty() || x < 0 || x >= gray.cols || y_start >= y_end) {
        return 0;
    }

    bool current_white = gray.at<uchar>(y_start, x) > ELEMENT_WHITE_THRESHOLD;
    int transitions = 0;

    for (int y = y_start + step; y < y_end; y += step) {
        if (y < 0 || y >= gray.rows) {
            continue;
        }

        bool pixel_white = gray.at<uchar>(y, x) > ELEMENT_WHITE_THRESHOLD;

        if (pixel_white != current_white) {
            transitions++;
            current_white = pixel_white;
        }
    }

    return transitions;
}

/*
 * 检测斑马线候选。
 *
 * 注意：这里返回的是 candidate，不是最终输出结果。
 * 最终输出还要在 ElementDetectAndPrintLowRate() 里做连续帧确认。
 */
static bool detectZebraCrossingLightweight(const cv::Mat& gray, ElementDebugInfo* debug_info = nullptr)
{
    if (gray.empty()) {
        return false;
    }

    const int height = gray.rows;
    const int width = gray.cols;

    const int x_start = static_cast<int>(width * 0.12);
    const int x_end = static_cast<int>(width * 0.88);

    const int y_start = static_cast<int>(height * 0.35);
    const int y_end = static_cast<int>(height * 0.90);

    if (x_end <= x_start || y_end <= y_start) {
        return false;
    }

    int total_transitions = 0;
    int valid_columns = 0;

    const int column_count = 9;

    for (int i = 0; i < column_count; ++i) {
        int x = x_start + (x_end - x_start) * i / std::max(1, column_count - 1);
        int transitions = transitionCountOnColumn(gray, x, y_start, y_end, 3);

        total_transitions += transitions;
        valid_columns++;
    }

    double avg_transitions = 0.0;
    if (valid_columns > 0) {
        avg_transitions = static_cast<double>(total_transitions) / static_cast<double>(valid_columns);
    }

    int stripe_count = 0;
    bool in_stripe = false;
    int stripe_height = 0;
    int max_stripe_height = 0;
    double max_white_ratio = 0.0;

    for (int y = y_start; y < y_end; y += 2) {
        double ratio = whiteRatioOnRow(gray, y, x_start, x_end, 2);
        max_white_ratio = std::max(max_white_ratio, ratio);

        bool row_is_white_band = ratio > 0.28;

        if (row_is_white_band) {
            if (!in_stripe) {
                in_stripe = true;
                stripe_height = 1;
            } else {
                stripe_height++;
            }
        } else {
            if (in_stripe) {
                if (stripe_height >= 1) {
                    stripe_count++;
                    max_stripe_height = std::max(max_stripe_height, stripe_height);
                }

                in_stripe = false;
                stripe_height = 0;
            }
        }
    }

    if (in_stripe && stripe_height >= 1) {
        stripe_count++;
        max_stripe_height = std::max(max_stripe_height, stripe_height);
    }

    if (debug_info != nullptr) {
        debug_info->avg_transitions = avg_transitions;
        debug_info->stripe_count = stripe_count;
        debug_info->max_white_ratio = max_white_ratio;
        debug_info->max_band_height = max_stripe_height;
    }

    /*
     * 判据一：典型斑马线，多条横向白带。
     */
    if (stripe_count >= 2 &&
        max_white_ratio >= ZEBRA_PARTIAL_MIN_RATIO &&
        max_white_ratio <= ZEBRA_PARTIAL_MAX_RATIO &&
        avg_transitions >= ZEBRA_MIN_TRANSITIONS)
    {
        return true;
    }

    /*
     * 判据二：当前摄像头视角下，斑马线有时只露出一条宽白带。
     * 这时主要依靠 max_white_ratio 区分：
     * - 普通白线接近 1.00；
     * - 斑马线约 0.75。
     */
    if (stripe_count == 1 &&
        max_white_ratio >= ZEBRA_PARTIAL_MIN_RATIO &&
        max_white_ratio <= ZEBRA_PARTIAL_MAX_RATIO &&
        avg_transitions >= ZEBRA_MIN_TRANSITIONS &&
        max_stripe_height >= ZEBRA_MIN_BAND_HEIGHT)
    {
        return true;
    }

    return false;
}

/*
 * 检测普通白线。
 */
static bool detectWhiteLineLightweight(const cv::Mat& gray, ElementDebugInfo* debug_info = nullptr)
{
    if (gray.empty()) {
        return false;
    }

    const int height = gray.rows;
    const int width = gray.cols;

    const int x_start = static_cast<int>(width * 0.12);
    const int x_end = static_cast<int>(width * 0.88);

    const int y_start = static_cast<int>(height * 0.45);
    const int y_end = static_cast<int>(height * 0.90);

    if (x_end <= x_start || y_end <= y_start) {
        return false;
    }

    int band_count = 0;
    bool in_band = false;
    int band_height = 0;
    int max_band_height = 0;

    double max_white_ratio = 0.0;

    for (int y = y_start; y < y_end; y += 2) {
        double ratio = whiteRatioOnRow(gray, y, x_start, x_end, 2);
        max_white_ratio = std::max(max_white_ratio, ratio);

        bool row_is_white_band = ratio > 0.45;

        if (row_is_white_band) {
            if (!in_band) {
                in_band = true;
                band_height = 1;
            } else {
                band_height++;
            }
        } else {
            if (in_band) {
                if (band_height >= 1) {
                    band_count++;
                    max_band_height = std::max(max_band_height, band_height);
                }

                in_band = false;
                band_height = 0;
            }
        }
    }

    if (in_band && band_height >= 1) {
        band_count++;
        max_band_height = std::max(max_band_height, band_height);
    }

    if (debug_info != nullptr) {
        debug_info->band_count = band_count;
        debug_info->max_band_height = max_band_height;
        debug_info->max_white_ratio = max_white_ratio;
    }

    /*
     * 普通白线必须接近横贯 ROI。
     * 根据你的数据，真实白线 wMaxRatio = 1.00，所以这里用 0.90 比较安全。
     */
    if (band_count >= 1 && band_count <= 2 && max_white_ratio >= WHITE_LINE_MIN_RATIO) {
        return true;
    }

    return false;
}

/*
 * 元素识别总入口。
 */
static void ElementDetectAndPrintLowRate(const cv::Mat& frame)
{
    if (frame.empty()) {
        return;
    }

    static int skip_counter = 0;
    static bool last_zebra_detected = false;
    static bool last_white_detected = false;
    static int zebra_candidate_count = 0;

    static double last_zebra_print_time = -100.0;
    static double last_white_print_time = -100.0;
    static double last_debug_print_time = -100.0;

    skip_counter++;

    if (skip_counter < ELEMENT_DETECT_INTERVAL) {
        return;
    }

    skip_counter = 0;

    double now = static_cast<double>(cv::getTickCount()) / cv::getTickFrequency();

    cv::Mat gray;

    if (frame.channels() == 3) {
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
    } else if (frame.channels() == 4) {
        cv::cvtColor(frame, gray, cv::COLOR_BGRA2GRAY);
    } else {
        gray = frame;
    }

    ElementDebugInfo zebra_debug;
    ElementDebugInfo white_debug;

    bool zebra_candidate = detectZebraCrossingLightweight(gray, &zebra_debug);

    if (zebra_candidate) {
        zebra_candidate_count++;
    } else {
        zebra_candidate_count = 0;
    }

    bool zebra_detected = zebra_candidate_count >= ZEBRA_CONFIRM_FRAMES;

    bool white_detected = false;

    /*
     * 如果已经出现斑马线候选，即使还没有连续确认，也先不要报白线。
     * 这样能避免斑马线进入画面的第一帧被误报为普通白线。
     */
    if (!zebra_candidate && !zebra_detected) {
        white_detected = detectWhiteLineLightweight(gray, &white_debug);
    }

    if (ELEMENT_DEBUG_PRINT && now - last_debug_print_time >= 1.0) {
        std::cout << std::fixed << std::setprecision(2)
                  << "[元素调试] zebraCandidate=" << zebra_candidate
                  << " zebra=" << zebra_detected
                  << " zCnt=" << zebra_candidate_count
                  << " stripe=" << zebra_debug.stripe_count
                  << " trans=" << zebra_debug.avg_transitions
                  << " zMaxRatio=" << zebra_debug.max_white_ratio
                  << " zMaxHeight=" << zebra_debug.max_band_height
                  << " | white=" << white_detected
                  << " band=" << white_debug.band_count
                  << " wMaxRatio=" << white_debug.max_white_ratio
                  << " wMaxHeight=" << white_debug.max_band_height
                  << std::endl;
        last_debug_print_time = now;
    }

    bool zebra_rising_edge = zebra_detected && !last_zebra_detected;
    bool white_rising_edge = white_detected && !last_white_detected;

    if (zebra_rising_edge && now - last_zebra_print_time >= ELEMENT_PRINT_COOLDOWN_SEC) {
        std::cout << "[元素识别] 检测到斑马线" << std::endl;
        last_zebra_print_time = now;
    }

    if (white_rising_edge && now - last_white_print_time >= ELEMENT_PRINT_COOLDOWN_SEC) {
        std::cout << "[元素识别] 检测到白线" << std::endl;
        last_white_print_time = now;
    }

    last_zebra_detected = zebra_detected;
    last_white_detected = white_detected;
}

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

    /*
     * 一、先做寻迹图像处理。
     *
     * 注意：
     * 元素识别不能放在 image_main() 前面，
     * 否则会拖慢中线更新，影响闭环控制。
     */
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

    /*
     * 二、立刻更新舵机误差 servo_error_temp。
     *
     * 这一步是闭环寻迹的关键。
     * 必须优先保证它及时执行。
     */
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

    /*
     * 三、低频元素识别。
     *
     * 注意：
     * 1. 放在 servo_error_temp 更新之后；
     * 2. 每 ELEMENT_DETECT_INTERVAL 帧才检测一次；
     * 3. 只打印，不控制小车。
     */
    ElementDetectAndPrintLowRate(raw_frame);

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