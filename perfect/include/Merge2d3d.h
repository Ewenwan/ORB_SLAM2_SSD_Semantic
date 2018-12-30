/* This file is part of ORB-SLAM2-SSD-Semantic.
 * 2D检测结果 和 3d点云 获取 3D目标信息
 */

#ifndef MERGE2D3D_H
#define MERGE2D3D_H

#include <pcl/common/common.h>
#include <pcl/point_types.h>

#include <pcl/filters/voxel_grid.h> // 体素格滤波
#include <pcl/filters/statistical_outlier_removal.h>// 统计滤波器 
#include <pcl/filters/extract_indices.h>            // 根据点云索引提取对应的点云
#include <pcl/filters/filter.h>                     // 取出nan点 变无序点

#include <Eigen/Core>

#include <vector>

#include "ObjectDatabase.h"// 目标数据库
#include "Detector.h"      // 2d检测器

struct Cluster;

class ObjectDatabase;

class Merge2d3d
{

public:
    typedef pcl::PointXYZRGB PointT;// 点类型 xyzrgb 点+颜色
    typedef pcl::PointCloud<PointT> PointCloud;// 点云类型
    Merge2d3d();
    ~Merge2d3d();
    void merge(std::vector<Object>& objects, cv::Mat depth, PointCloud::Ptr pclMap);

    ObjectDatabase* mpOD;// 需要定义成指针===   公开 方便可视化器获取====

protected:
    bool mergeOne(Object& object, Cluster& cluster, cv::Mat depth_img, PointCloud::Ptr pclMap);

    pcl::ExtractIndices<PointT> mExtractInd;// 索引提取点云器  设置为成员对象
    pcl::VoxelGrid<PointT>  mVoxel;             // 体素格滤波对象   
    pcl::StatisticalOutlierRemoval<PointT> mStat; // 统计学滤波，剔除离群点
    
    // ObjectDatabase mOD;// 目标数据库==此时编译器还不知道定义，无法知道类内部成员，所以无法构造实例对象
    // ObjectDatabase* mpOD;// 需要定义成指针=== 
};

#endif // MERGE2D3D_H
