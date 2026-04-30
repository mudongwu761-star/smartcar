#include "sign_classify.h"


bool sign_flag=1;
std::chrono::time_point<std::chrono::high_resolution_clock> last_zebra_time;

std::vector<cv::Mat> landmarrk_templates;

void drawPic(Mat pic, Mat &output, int x = 0, int y = 140, int width = 100,
             int height = 100) {
  cv::resize(pic, pic, cv::Size(width, height));
  if (pic.channels() == 1) {
    cv::cvtColor(pic, pic, cv::COLOR_GRAY2BGR);
  }
  cv::Rect roi(cv::Point(x, y), pic.size());
  if (roi.x >= 0 && roi.y >= 0 && roi.x + roi.width <= output.cols &&
      roi.y + roi.height <= output.rows) {
    pic.copyTo(output(roi));
  } else {
    if (roi.x + roi.width > output.cols) {
      roi.width = output.cols - roi.x;
    }
    if (roi.y + roi.height > output.rows) {
      roi.height = output.rows - roi.y;
    }
    cv::Rect srcRoi(0, 0, roi.width, roi.height);
    cv::Mat srcCropped = pic(srcRoi);
    srcCropped.copyTo(output(roi));
  }
}
//return current time (per second)
double getTime() { return  static_cast<double>(cv::getTickCount() / cv::getTickFrequency()); }

void initTemplates() {
  landmarrk_templates.push_back(
      cv::imread("img/both.png", cv::IMREAD_GRAYSCALE));
  landmarrk_templates.push_back(
      cv::imread("img/left.png", cv::IMREAD_GRAYSCALE));
  landmarrk_templates.push_back(
      cv::imread("img/right.png", cv::IMREAD_GRAYSCALE));
  landmarrk_templates.push_back(
      cv::imread("img/light.png", cv::IMREAD_GRAYSCALE));

  for (auto &img : landmarrk_templates) {
    cv::bitwise_not(img, img);
  }
}
void SignProcess(const Mat &img) {
  if(img.empty())
  {
    cout<<"transport wrong!"<<endl;
  }
  Mat result_img = img.clone();
  static int height = img.rows;
  static int width = img.cols;
  
  // Mat gray;
  // cvtColor(img, gray, COLOR_BGR2GRAY);
  // Mat binary;
  // threshold(gray, binary, 0, 255, THRESH_BINARY | THRESH_OTSU);

  // 检测斑马线
  if(sign_flag)
  {
    bool now_black = false;
    bool last_black = false;
    int chang_count = 0;
    for (int i = width * 0.2; i < width * 0.8; i += 5) {
      now_black = binary.at<uchar>(height-32, i) == 0;
      if (last_black != now_black) {
        last_black = now_black;
        chang_count++;
      }
    }
    if (chang_count > 6) {
      cout << "检测到斑马线,停车3s" << endl;
      pause_flag.store(true);
      sign_flag = 0;
      last_zebra_time = std::chrono::high_resolution_clock::now();
    }
  }
  else{
    auto current_time = std::chrono::high_resolution_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(current_time - last_zebra_time).count();
    if (elapsed >= 20) {  // 20秒后重新启用
      sign_flag = true;
      std::cout << "斑马线检测重新启用" << std::endl;
    }
  }



//   // 截取binary中间一片区域
//   cv::Rect mid_region(80, 60, 160, 120);
//   float black_ratio =
//       1.0f - (countNonZero(binary(mid_region))) / (float)mid_region.area();
//   cout << "黑色比例: " << black_ratio << endl;
//   if (black_ratio > 0.2) // 黑色多，有地标
//   {
//     cv::Mat hsv_img;
//     cv::cvtColor(img, hsv_img, cv::COLOR_BGR2HSV);
//     cv::Mat mask;
//     cv::inRange(hsv_img, cv::Scalar(95, 100, 0), cv::Scalar(130, 255, 255),
//                 mask); // 蓝色
//     cv::Mat bull_img;
//     cv::bitwise_and(img, img, bull_img, mask); // 提取处蓝色部分

//     cv::Mat gray_bull_img;
//     cv::cvtColor(bull_img, gray_bull_img, cv::COLOR_BGR2GRAY);
//     std::vector<std::vector<cv::Point>> contours;
//     cv::findContours(gray_bull_img, contours, cv::RETR_EXTERNAL,
//                      cv::CHAIN_APPROX_SIMPLE);
//     cv::Mat filled_mask = cv::Mat::zeros(img.size(), CV_8UC1);
//     contours.erase(
//         std::remove_if(contours.begin(), contours.end(),
//                        [=](const std::vector<cv::Point> &contour) {
//                          const int tolerance = 10;
//                          Rect rect = boundingRect(contour);
//                          if (rect.x <= tolerance ||
//                              rect.y + rect.width >= width - tolerance ||
//                              rect.y <= tolerance ||
//                              rect.y + rect.height >= height - tolerance)
//                            return true;
//                          else
//                            return false;
//                        }),
//         contours.end());
//     cv::drawContours(filled_mask, contours, -1, cv::Scalar(255),
//                      cv::FILLED); // 将所有闭合蓝色图标填充起来，作为遮罩选中
//     // drawPic(filled_mask, result_img); // debug
//     std::cerr<<"3"<<std::endl;
//     cv::Mat icon_img, white_img;
//     cv::bitwise_and(img, img, icon_img, filled_mask);
//     cv::cvtColor(icon_img, icon_img, cv::COLOR_BGR2GRAY);
//     threshold(icon_img, white_img, 0, 255,
//               THRESH_BINARY | THRESH_OTSU); // 找出白色部分

//     cv::findContours(white_img, contours, cv::RETR_EXTERNAL,
//                      cv::CHAIN_APPROX_SIMPLE);
//     std::vector<std::vector<cv::Point>> filtered_contours;
//     for (const auto &contour : contours) {
//       double perimeter = cv::arcLength(contour, true);
//       if (perimeter > 100) {
//         filtered_contours.push_back(contour);
//       }
//     }
//     cv::Mat landmark_img = cv::Mat::zeros(img.size(), CV_8UC1);
//     cv::drawContours(landmark_img, filtered_contours, -1, cv::Scalar(255),
//                      cv::FILLED); // 过滤高光
//     if (landmark_img.empty()) {
//       throw std::runtime_error("Input image is empty!");
//     }
// std::cerr<<"4"<<std::endl;
//     std::vector<cv::Point> non_zero_points;
//     cv::findNonZero(landmark_img, non_zero_points);
//     std::cerr<<"4.0"<<std::endl;
//     if (!non_zero_points.empty()) {
//       std::cerr<<"4.1"<<std::endl;
//       cv::Rect bounding_rect = cv::boundingRect(non_zero_points);
      
//       if (bounding_rect.width * bounding_rect.height > 800) {
//         landmark_img = landmark_img(bounding_rect);
//         drawPic(landmark_img, result_img); // debug

//         int min_diff = landmark_img.size().area();
//         int match_index = -1;
//         int confidence = 0.0;
// std::cerr<<"4.2"<<std::endl;
//         for (size_t i = 0; i < landmarrk_templates.size(); ++i) {
//           cv::Mat resized_template;
//           cv::resize(landmarrk_templates[i], resized_template,
//                      landmark_img.size());

//           cv::Mat diff;
//           cv::absdiff(resized_template, landmark_img, diff);
//           int sum_diff = cv::countNonZero(diff);
//           if (sum_diff < min_diff) {
//             min_diff = sum_diff;
//             match_index = i;
//             confidence =
//                 100 - ((float)sum_diff / (float)(landmarrk_templates[i].rows *
//                                                  landmarrk_templates[i].cols)) *
//                           100.0;
//           }
//         }
// std::cerr<<"5"<<std::endl;
//         if (match_index != -1 && confidence > 40) {
//           std::cout << "找到地标: " << match_index << " : " << confidence << "%"
//                     << std::endl;
//           cv::Mat i = landmarrk_templates[match_index];
//           cv::rectangle(i, cv::Point(0, 0), cv::Point(i.cols - 1, i.rows - 1),
//                         255, 2);
//           drawPic(i, result_img, bounding_rect.x, bounding_rect.y, 40, 40);
//           cout << "停车3s" << endl;
//         }
//       }
//     }
//     else{
//       std::cerr<<"something wrong"<<std::endl;
//     }
//   }
}

int SignClassify(void)
{
    SignProcess(raw_frame);
    return 0;
}
