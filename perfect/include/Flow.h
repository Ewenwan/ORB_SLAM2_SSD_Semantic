/* This file is part of ORB-SLAM2-SSD-Semantic.
* 根据光流计算 二值动静 mask
*/
#ifndef FLOW_H
#define FLOW_H

#include <iostream> 
#include <opencv2/opencv.hpp>

#include "Frame.h"

using namespace cv; 
using namespace std; 

namespace FlowSLAM
{

class Flow
{

private:
   cv::Mat mImGrayLast;
   cv::Mat mImGrayCurrent;
   //cv::Mat mImSource;
   // float mBInaryThreshold;

public:
   Flow();
    ~Flow() = default;
  void ComputeMask(const cv::Mat& GrayImg, cv::Mat& mask, float BInaryThreshold);
  void ComputeMask(const cv::Mat& GrayImg, const cv::Mat& Homo, cv::Mat& mask, float BInaryThreshold);
private:
  void myWarpPerspective(const cv::Mat& src, cv::Mat& dst, const cv::Mat& H);// 传入单应变换矩阵===
};
}



#endif// FLOW_H
