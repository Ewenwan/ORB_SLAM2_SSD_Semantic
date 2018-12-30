/**
* This file is part of ORB-SLAM2.
* 运行目标检测 的类
*/

#ifndef RUNDETECT_H
#define RUNDETECT_H

#include "System.h"
#include <condition_variable>// 多线程锁 状态变量
#include "KeyFrame.h"
#include "Detector.h"        // 2d目标检测结果===
#include <boost/make_shared.hpp>
using namespace ORB_SLAM2;
class ORB_SLAM2::KeyFrame;

class Detector; // 声明目标检测类

class RunDetect
{

public:
    void insertKFColorImg(KeyFrame* kf, cv::Mat color);// 插入一个关键帧的彩色图像，检测过后可以删除===
    void Run(void);// 线程运行函数====
    RunDetect();
    ~RunDetect();

protected:
    std::shared_ptr<thread>  mRunThread; // 执行线程==

    condition_variable  colorImgUpdated; 
// 关键帧更新 <condition_variable> 头文件主要包含了与条件变量相关的类和函数。
// 全局条件变量. 用于多线程之间的 相互等待！！！！！！！
    // condition_variable 类 参考 https://msdn.microsoft.com/zh-cn/magazine/hh874752(v=vs.120)
    mutex               colorImgMutex;// 关键帧更新  互斥锁
    std::vector<cv::Mat>     colorImgs;   // 灰度图    数组
    std::vector<KeyFrame*> mvKeyframes;  // 关键帧指针 数组
    mutex mvKeyframesMutex;

    //std::vector<std::vector<Object>> mvvObjects;// 保持每张关键帧图像的 2d检测结果==== 
    //mutex  mvvObjectsMutex;


    uint16_t          lastKeyframeSize =0;
    Detector* mDetector;// 目标检测对象====
};
#endif
