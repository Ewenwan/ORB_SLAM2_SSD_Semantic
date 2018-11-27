/* This file is part of ORB-SLAM2-SSD-Semantic.
 * 新添加的语义地图构建 
 * 
 */

#ifndef POINTCLOUDMAPPING_H
#define POINTCLOUDMAPPING_H

#include "System.h"

#include <pcl/common/transforms.h>
#include <pcl/common/common.h>
#include <pcl/point_types.h>

#include <pcl/filters/voxel_grid.h> // 体素格滤波
#include <pcl/filters/statistical_outlier_removal.h>// 统计滤波器 
#include <pcl/filters/extract_indices.h>            // 根据点云索引提取对应的点云
#include <pcl/filters/filter.h>                     // 取出nan点 变无序点
#include <pcl/io/pcd_io.h> // 读写点云
#include <pcl/visualization/pcl_visualizer.h> // PCLVisualizer 点云可视化器

#include <condition_variable>

#include "Thirdparty/ncnn/ncnn_dect.h"// ncnn ssd目标检测

#include <Eigen/Core>

#include <vector>

using namespace ORB_SLAM2;

// 目标语义信息
typedef struct Cluster
{
 std::string object_name; // 物体类别名
 int class_id;            // 对应类别id
 float prob;              // 置信度
 Eigen::Vector3f minPt;   // 所有点中最小的x值，y值，z值
 Eigen::Vector3f maxPt;   // 所有点中最大的x值，y值，z值
 Eigen::Vector3f centroid;// 点云中心点 
 bool operator ==(const std::string &x);
} Cluster;

class PointCloudMapping
{
public:
    typedef pcl::PointXYZRGB PointT;// 点类型 xyzrgb 点+颜色
    typedef pcl::PointCloud<PointT> PointCloud;// 点云类型

    PointCloudMapping( double resolution_ );// 类初始化(构造)函数
  
    // 插入一个keyframe，会更新一次地图
    void insertKeyFrame( KeyFrame* kf, cv::Mat& color, cv::Mat& depth, cv::Mat& imgRGB);
    void shutdown();// 相当于类析构函数
    void viewer();  // 可视化点云函数
    void update();

protected:
    PointCloud::Ptr generatePointCloud(KeyFrame* kf, cv::Mat& color, cv::Mat& depth);

    void draw_rect_with_depth_threshold(cv::Mat bgr_img, 
                                    cv::Mat depth_img, 
                                    const cv::Rect_<float>& rect, const cv::Scalar& scalar,
                                    pcl::PointIndices & indices);
    void sem_merge(Cluster cluster); // 融合点云团
    void add_cube(void);// 可视化器中加入 3d标识框

    PointCloud::Ptr globalMap;        // 点云地图指针   boost::share_ptr
    std::shared_ptr<thread>  viewerThread; // 点云可视化线程 std::shared_ptr

    bool    shutDownFlag    =false;   // 关闭标志
    mutex   shutDownMutex;            // 关闭 线程互斥锁
 
    condition_variable  keyFrameUpdated; 
// 关键帧更新 <condition_variable> 头文件主要包含了与条件变量相关的类和函数。
// 全局条件变量. 用于多线程之间的 相互等待！！！！！！！
    // condition_variable 类 参考 https://msdn.microsoft.com/zh-cn/magazine/hh874752(v=vs.120)
    mutex               keyFrameUpdateMutex;// 关键帧更新  互斥锁

    // data to generate point clouds
    std::vector<KeyFrame*>       keyframes;  // 关键帧指针 数组
    std::vector<cv::Mat>         colorImgs;  // 灰度图    数组
    std::vector<cv::Mat>         depthImgs;  // 深度图    数组
    std::vector<cv::Mat>         RGBImgs;    // 深度图    数组
    mutex                   keyframeMutex;   // 关键帧 互斥锁
    uint16_t                lastKeyframeSize =0;

    double resolution = 0.04;      // 默认点云地图精度    用于设置体素格子的边长大小
    pcl::VoxelGrid<PointT>  voxel; // 点对应的 体素格滤波对象

    //shared_ptr<std::vector<cv::Scalar>> colors_ptr;// 不同物体颜色对象   std::shared_ptr
    std::vector<cv::Scalar> colors_;   // 每种物体的颜色
    std::vector<float>      obj_size_; // 每种物体的大小
    std::shared_ptr<Detector> ncnn_detector_ptr; // ncnn ssd目标检测std::shared_ptr
    

    std::vector<Cluster> clusters;// 语义点云指针数组
    pcl::ExtractIndices<PointT> extract_indices;// 索引提取点云器  设置为成员对象
    // pcl::VoxelGrid<PointT>  voxel_; // 体素格滤波对象  上面已经设置了
    pcl::StatisticalOutlierRemoval<PointT> stat; // 统计学滤波，剔除离群点

    
    std::shared_ptr<pcl::visualization::PCLVisualizer> pcl_viewer_prt;
    std::shared_ptr<pcl::PCDWriter>                    pcd_writer_ptr;


    //std::shared_ptr<thread>  showThread; // 点云更新线程 std::shared_ptr
    int map_state_ok;// 地图是否准备好
};

#endif // POINTCLOUDMAPPING_H
