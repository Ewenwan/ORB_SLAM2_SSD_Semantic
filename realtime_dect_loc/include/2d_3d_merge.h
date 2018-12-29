#ifndef MERGE_H
#define MERGE_H

#include "./ncnn_dect.h"// 2d检测结果
#include "./ty_camera.h"// 点云

#include <pcl/filters/voxel_grid.h> // 体素格滤波
#include <pcl/filters/statistical_outlier_removal.h>// 统计滤波器 
#include <pcl/filters/extract_indices.h>            // 根据点云索引提取对应的点云

#include <pcl/common/transforms.h>
#include <pcl/common/common.h>
#include <Eigen/Core>
#include <vector>

//typedef pcl::PointCloud<pcl::PointXYZ> Cloud;
typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> Cloud;

// 检测结果类====
typedef struct Cluster
{
    Object object; // 2d 检测框
    // Cloud::Ptr c_ptr;// 对应点云指针, 不需要记录，只需要在总点云中显示包围盒
    // pcl::PointIndices indices;                 // 点云团对应的 点云索引
    Eigen::Vector4f minPt;                       // 所有点中最小的x值，y值，z值
    Eigen::Vector4f maxPt;                       // 所有点中最大的x值，y值，z值
    Eigen::Vector4f centroid;                    // 点云中心点 齐次表示 
    Eigen::Vector3f sizePt;                      // 长宽高=== 
    Eigen::Vector3f boxCenter;                   // 包围盒中心点

} Cluster;

class Merge
{

public:
  /** Default constructor */
  Merge();

  /** Default destructor */
  ~Merge();
// 按2d框获取 点云索引，提取对应点云，点云滤波，聚类。
  void extract(std::vector<Object>& objects, 
               Cloud::Ptr point_ptr, 
               std::vector<Cluster>& clusters);
// 计算 点云 坐标，3d 包围盒


private:
  pcl::VoxelGrid<PointT>  voxel_; // 体素格滤波对象
  pcl::ExtractIndices<PointT> extract_indices_;// 索引提取点云器  设置为成员对象
  //pcl::PointIndices indices_;    // 取得需要的索引
  pcl::StatisticalOutlierRemoval<PointT> sta_; // 统计学滤波，剔除离群点
  Cloud::Ptr last_;   // 智能指针
  Cloud::Ptr tmp_;    // 智能指针
};



#endif

