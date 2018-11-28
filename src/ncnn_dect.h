
#ifndef DETECTOR_H
#define DETECTOR_H

#include "./include/net.h"  // ncnn 头文件
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>// gui 画图等
#include <opencv2/imgproc/imgproc.hpp>
#include <vector>

// 检测结果类====
typedef struct Object
{
    cv::Rect_<float> rect;// 边框
    std::string object_name;// 物体类别名
    int class_id; // 类别id
    float prob;   // 置信度
} Object;

class Detector
{
public:
  /** Default constructor */
  Detector();// 网络初始化

  /** Default destructor */
  ~Detector();// 析构函数
  
  void Run(const cv::Mat& bgr_img, std::vector<Object>& objects);
  void Show(const cv::Mat& bgr_img, std::vector<Object>& objects);
private:
   // ncnn::Net * det_net_mobile;  
   ncnn::Net * det_net_ptr;// 检测网络指针
   ncnn::Mat * net_in_ptr; // 网络输入指针
};

#endif
