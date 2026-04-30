/*
 * @Author: ilikara 3435193369@qq.com
 * @Date: 2025-01-04 06:51:37
 * @LastEditors: ilikara 3435193369@qq.com
 * @LastEditTime: 2025-04-12 08:33:43
 * @FilePath: /smartcar/lib/image_main.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include <opencv2/opencv.hpp>
#include <vector>

void image_main();

extern cv::Mat raw_frame;
extern cv::Mat grayFrame;
extern cv::Mat binarizedFrame;
extern cv::Mat binary;
extern cv::Mat morphologyExFrame;
extern cv::Mat track;

extern std::vector<int> left_line; // 左边缘列号数组
extern std::vector<int> right_line; // 右边缘列号数组
extern std::vector<int> mid_line; // 中线列号数组

extern int line_tracking_height;
extern int line_tracking_width;
extern int line_start;
extern int line_end;

extern int yellowAreaCount;
extern int RedAreaCount;