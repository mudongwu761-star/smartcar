/*
 * @Author: ilikara 3435193369@qq.com
 * @Date: 2025-01-04 06:50:56
 * @LastEditors: ilikara 3435193369@qq.com
 * @LastEditTime: 2025-04-12 09:49:22
 * @FilePath: /2k300_smartcar/src/image_cv.cpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include "image_cv.h"
#include <iostream>
#include "global.h"
#include "sign_classify.h"

cv::Mat raw_frame;
cv::Mat grayFrame;
cv::Mat binarizedFrame;
cv::Mat binary;
cv::Mat morphologyExFrame;
cv::Mat track;
cv::Mat last_frame;

std::vector<int> left_line; // 左边缘列号数组
std::vector<int> right_line; // 右边缘列号数组
std::vector<int> mid_line; // 中线列号数组

int line_tracking_width;
int line_tracking_height;
int line_start,line_end;

int yellowAreaCount;
int RedAreaCount;
bool all_line = false;
int left_down_flag = 0, right_down_flag = 0, left_up_flag = 0, right_up_flag = 0;

int detectYellowBlocks(const cv::Mat& inputImage)
{
    // 检查输入图像是否有效
    if (inputImage.empty()) {
        std::cerr << "Error: Input image is empty!" << std::endl;
        return -1;
    }

    // 将图像从BGR转换为HSV颜色空间（HSV更适合颜色检测）
    cv::Mat hsvImage;
    cvtColor(inputImage, hsvImage, cv::COLOR_BGR2HSV);

    // 定义黄色的HSV范围（可能需要根据实际情况调整）
    // 黄色在HSV中的范围大约为：H:20-30, S:100-255, V:100-255
    cv::Scalar lowerYellow(10, 100, 100);
    cv::Scalar upperYellow(40, 255, 255);

    // 创建黄色区域的掩膜
    cv::Mat yellowMask;
    inRange(hsvImage, lowerYellow, upperYellow, yellowMask);

    // 可选：进行形态学操作以去除噪声和小点
    cv::Mat kernel = getStructuringElement(cv::MORPH_RECT, cv::Size(2, 2));
    morphologyEx(yellowMask, yellowMask, cv::MORPH_OPEN, kernel);
    morphologyEx(yellowMask, yellowMask, cv::MORPH_CLOSE, kernel);

    // 查找轮廓
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    findContours(yellowMask, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    // 过滤掉太小的轮廓（根据实际需求调整面积阈值）
    int minContourArea = 4; // 最小轮廓面积阈值
    int yellowBlockCount = 0;

    for (size_t i = 0; i < contours.size(); i++) {
        double area = contourArea(contours[i]);
        if (area > minContourArea) {
            yellowBlockCount++;
        }
    }

    return yellowBlockCount;
}

void detectRedBlocks(const cv::Mat& inputImage)
{
    // 检查输入图像是否有效
    if (inputImage.empty()) {
        std::cerr << "Error: Input image is empty!" << std::endl;
    }

    // 将图像从BGR转换为HSV颜色空间（HSV更适合颜色检测）
    cv::Mat hsvImage;
    cvtColor(inputImage, hsvImage, cv::COLOR_BGR2HSV);

    // 定义红色的HSV范围（可能需要根据实际情况调整）
    // 黄色在HSV中的范围大约为：H:0-10\156-180, S:50-255, V:46-150
    cv::Scalar lowerRed(156, 100, 100);
    cv::Scalar upperRed(180, 255, 255);

    // 创建红色区域的掩膜
    cv::Mat RedMask;
    inRange(hsvImage, lowerRed, upperRed, RedMask);

    // 可选：进行形态学操作以去除噪声和小点
    cv::Mat kernel = getStructuringElement(cv::MORPH_RECT, cv::Size(2, 2));
    morphologyEx(RedMask, RedMask, cv::MORPH_OPEN, kernel);
    morphologyEx(RedMask, RedMask, cv::MORPH_CLOSE, kernel);

    // 查找轮廓
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    findContours(RedMask, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    if (contours.size())
    {
        printf("%zu\n",contours.size());
    }
}

static inline cv::Mat image_binerize(cv::Mat& frame)
{
    cv::Mat output;
    cv::Mat binarizedFrame;
    cv::Mat hsvImage;
    cv::cvtColor(frame, hsvImage, cv::COLOR_BGR2HSV);
    //split hsvImage into three hsvChannel
    std::vector<cv::Mat> hsvChannels;
    cv::split(hsvImage, hsvChannels);

    cv::threshold(hsvChannels[0], binarizedFrame, 0, 255, cv::THRESH_BINARY_INV | cv::THRESH_OTSU);
    cv::threshold(hsvChannels[1], output, 0, 255, cv::THRESH_BINARY_INV | cv::THRESH_OTSU);
    cv::bitwise_or(output, binarizedFrame, output);

    return output;
}

static inline cv::Mat find_road(cv::Mat& frame)
{
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_CROSS, cv::Size(2, 2));
    cv::morphologyEx(binarizedFrame, morphologyExFrame, cv::MORPH_OPEN, kernel);

    cv::Mat mask = cv::Mat::zeros(line_tracking_height + 2, line_tracking_width + 2, CV_8UC1);

    cv::Point seedPoint(line_tracking_width / 2, line_tracking_height - 10);

    cv::circle(morphologyExFrame, seedPoint, 10, 255, -1);

    cv::Scalar newVal(128);

    cv::Scalar loDiff = cv::Scalar(20);
    cv::Scalar upDiff = cv::Scalar(20);

    cv::floodFill(morphologyExFrame, mask, seedPoint, newVal, 0, loDiff, upDiff, 8);

    cv::Mat outputImage = cv::Mat::zeros(line_tracking_width, line_tracking_height, CV_8UC1);

    mask(cv::Rect(1, 1, line_tracking_width, line_tracking_height)).copyTo(outputImage);

    return outputImage;
}

void cross_dir()
{
    left_down_flag = 0, right_down_flag = 0, left_up_flag = 0, right_up_flag = 0;
    for(int i = line_start; i > line_end; i--){
        if(left_line[i] - left_line[i-2] > 3 && left_line[i] - left_line[i+1] < 3 && left_line[i+1] - left_line[i+2] < 3)
        {
            //left_down_flag = i;
            break;
            // k_l = 0.3;
        }
        if(left_line[i] - left_line[i+2] > 3 && left_line[i-1] - left_line[i] < 3 && left_line[i-2] - left_line[i-1] < 3)
        {
            left_up_flag = i;
            if(left_down_flag)
                break;
            }
        }
    for(int i = line_start; i > line_end ; i--){
        if(right_line[i-2] - right_line[i] > 3 && right_line[i+1] - right_line[i] < 3 && right_line[i+2] - right_line[i+1] < 3)
        {
            //right_down_flag = i;
            break;
        }
         if(right_line[i+2] - right_line[i] > 3 && right_line[i] - right_line[i-1] < 3 && right_line[i-1] - right_line[i-2] < 3)
        {
            right_up_flag = i;
            if(right_down_flag)
              break;
        }
    }
    if(left_down_flag){
        //std::cout<<"find left down flag"<<std::endl;
        for(int i = left_down_flag; i > line_end; i--){
            left_line[i] = left_line[left_down_flag] - 0.3 * (i - left_down_flag);
        }
    }
    else if(left_up_flag){
        //std::cout<<"find left up flag"<<std::endl;
        for(int i = left_up_flag; i < line_start; i++){
            left_line[i] = left_line[left_up_flag] - 0.3 *(i - left_up_flag);
        } 
    }
    if(right_down_flag){
        //std::cout<<"find right down flag"<<std::endl;
        for(int i = right_down_flag; i > line_end; i--){
            right_line[i] = right_line[right_down_flag] + 0.3 * (i - right_down_flag);
        }
    }
    else if(right_up_flag){
        //std::cout<<"find right up flag"<<std::endl;
        for(int i = right_up_flag; i < line_start; i++){
            right_line[i] = right_line[right_up_flag] + 0.3 * (i - right_up_flag);
        } 
    }
}

void image_main()
{

    cv::Mat resizedFrame;

    cv::resize(raw_frame, resizedFrame, cv::Size(line_tracking_width, line_tracking_height));

    yellowAreaCount = detectYellowBlocks(resizedFrame);
    // std::cout << " Yellow Area:" << yellowAreaCount << std::endl;
    //detectRedBlocks(resizedFrame);

    binarizedFrame = image_binerize(resizedFrame);

    track = find_road(binarizedFrame);

    uchar(*IMG)[line_tracking_width] = reinterpret_cast<uchar(*)[line_tracking_width]>(track.data);//将图像转换为二维数组

    bool use_last_frame = true;
    for (int i = 0; i < line_tracking_height; i=i+10)
    {
        for (int j = 0; j < line_tracking_width; j=j+10)
        {
            if (!binarizedFrame.at<uchar>(i, j))
            {
                use_last_frame = false;
                break;
            }
        }
    }
    if (!last_frame.empty() && use_last_frame) {
        // 如果当前帧的二值化图像中有足够的白色像素，则使用上一帧的二值化图像
        binarizedFrame = last_frame;
    } else {
        // 否则，使用当前帧的二值化图像
        last_frame = binarizedFrame;
    
    
    
    
    left_line.clear();
    right_line.clear();
    mid_line.clear();

    left_line.resize(line_tracking_height, -1);
    right_line.resize(line_tracking_height, -1);
    mid_line.resize(line_tracking_height, -1);

    

    /*逐行扫线并记录每一行白色像素的起始点和终止点*/

    if (readFlag(start_file)) {
        int foresee = readDoubleFromFile(foresee_file);
        if (all_line)
        {
            line_start = line_tracking_height - 1;
            line_end = 10;
        } else {
            line_start = foresee + 5;
            line_end = foresee - 5;
        }
                
    }


    for (int i = line_start; i >= line_end; --i) {
        int max_start = -1;
        int max_end = -1;
        int current_start = -1;
        int current_length = 0;
        int max_length = 0;

        for (int j = 0; j < line_tracking_width; ++j) {
            if (IMG[i][j]) {
                if (current_length == 0) {
                    current_start = j;
                    current_length = 1;
                } else {
                    current_length++;
                }
                if (j == line_tracking_width - 1 || IMG[i][j + 1] == 0) {
                    if (i == line_start || !(left_line[i + 1] > j || right_line[i + 1] < current_start)) {
                        if (current_length >= max_length) {
                            max_length = current_length;
                            max_start = current_start;
                            max_end = j;
                        }
                    }
                }
            } else {
                current_length = 0;
                current_start = -1;
            }
        }
        if (max_length > 0) {
            left_line[i] = max_start;
            right_line[i] = max_end;
        } else {
            left_line[i] = -1;
            right_line[i] = -1;
        }
    }

    //cross_dir();

    // if (yellowAreaCount >= 5) {
        
    //     all_line = true;
    //     int left_mark = 0, right_mark = 0;
    //     for (int row = line_tracking_height - 10; row >= 0; --row) {
    //         if (left_line[row] == -1 || right_line[row] == -1) {
    //             break;
    //         }
    //         if (!left_mark && left_line[row] - left_line[row + 1] > 5) {
    //             left_mark = row;
    //         }
    //         if (!right_mark && right_line[row + 1] - right_line[row] > 5) {
    //             right_mark = row;
    //         }
    //         if (left_mark && right_mark) {
    //             double out_x = (left_line[left_mark] + right_line[right_mark]) / 2.0, out_y = (left_mark + right_mark) / 2.0;
    //             double k = (out_x - line_tracking_width / 2.0) / (out_y - (line_tracking_height - 1));
    //             for (int i = line_tracking_height - 1; i >= MIN(left_mark, right_mark) - 5; --i) {
    //                 mid_line[i] = (i - (line_tracking_height - 1)) * k + (line_tracking_width / 2.0);
    //             }
    //             break;
    //         }
    //     }
    // } else {
    //     all_line = false;
    // }
    //}

    for (int row = line_start ; row >= line_end; --row) {
        if (left_line[row] != -1) {
            mid_line[row] = (left_line[row] + right_line[row]) / 2;
        } else {
            mid_line[row] = mid_line[row + 1];
        }
    }

    { //sign classify
        // cv::Mat gray;
        // cvtColor(raw_frame,gray, COLOR_BGR2GRAY);
        // threshold(gray, binary, 160, 255, THRESH_BINARY);
        //SignProcess(binary);
    }
    }
}