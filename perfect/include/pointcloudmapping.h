/*
 *
 */

#ifndef POINTCLOUDMAPPING_H
#define POINTCLOUDMAPPING_H

#include "System.h"

#include <pcl/common/transforms.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <condition_variable>

using namespace ORB_SLAM2;

class PointCloudMapping
{
public:
    typedef pcl::PointXYZRGBA PointT;// 点类型 xyzrgba 点+颜色+透明度
    typedef pcl::PointCloud<PointT> PointCloud;// 点云类型

    PointCloudMapping( double resolution_ );// 类初始化(构造)函数

    // 插入一个keyframe，会更新一次地图
    void insertKeyFrame( KeyFrame* kf, cv::Mat& color, cv::Mat& depth );
    void shutdown();// 相当于类析构函数
    void viewer();  // 可视化点云函数

protected:
    PointCloud::Ptr generatePointCloud(KeyFrame* kf, cv::Mat& color, cv::Mat& depth);

    PointCloud::Ptr globalMap;        // 点云地图指针
    shared_ptr<thread>  viewerThread; // 点云可视化线程

    bool    shutDownFlag    =false;   // 关闭标志
    mutex   shutDownMutex;            // 关闭 线程互斥锁
 
    condition_variable  keyFrameUpdated; 
// 关键帧更新 <condition_variable> 头文件主要包含了与条件变量相关的类和函数。
// 全局条件变量. 用于多线程之间的 相互等待！！！！！！！
    // condition_variable 类 参考 https://msdn.microsoft.com/zh-cn/magazine/hh874752(v=vs.120)
    mutex               keyFrameUpdateMutex;// 关键帧更新  互斥锁

    // data to generate point clouds
    vector<KeyFrame*>       keyframes;  // 关键帧指针 数组
    vector<cv::Mat>         colorImgs;  // 灰度图    数组
    vector<cv::Mat>         depthImgs;  // 深度图    数组
    mutex                   keyframeMutex; // 关键帧 互斥锁
    uint16_t                lastKeyframeSize =0;

    double resolution = 0.04;      // 默认点云地图精度    用于设置体素格子的边长大小
    pcl::VoxelGrid<PointT>  voxel; // 点对应的 体素格滤波对象
};

#endif // POINTCLOUDMAPPING_H
