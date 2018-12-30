/*2d & 3d 融合算法
点云 多平面分割、聚类
反向投影获取点晕团的2d框
和目标检测2d框 匹配
获取每个点云团的类别信息，以及3d边框信息
返回 cluster 数组信息

*/

#ifndef MERGESG_H
#define MERGESG_H


//#include <pcl/filters/voxel_grid.h> // 体素格滤波
//#include <pcl/filters/statistical_outlier_removal.h>// 统计滤波器 
//#include <pcl/filters/extract_indices.h>            // 根据点云索引提取对应的点云
#include <pcl/features/impl/integral_image_normal.hpp> // 点云积分图 法线估计
#include <pcl/features/impl/normal_3d.hpp>             // 3d点云法线估计
#include <pcl/filters/impl/extract_indices.hpp>        // 根据点云索引提取对应的点云
#include <pcl/filters/impl/voxel_grid.hpp>             // 体素格滤波
#include <pcl/kdtree/impl/kdtree_flann.hpp>            // 二叉树搜索
#include <pcl/search/impl/kdtree.hpp>
#include <pcl/segmentation/impl/extract_clusters.hpp>  // 分割器 提取点云
#include <pcl/segmentation/impl/organized_multi_plane_segmentation.hpp> // 多平面分割
#include <pcl/segmentation/plane_coefficient_comparator.h>              // 平面分割系数参数
#include <pcl/segmentation/euclidean_plane_coefficient_comparator.h>    // 欧式距离 平面分割系数
#include <pcl/segmentation/rgb_plane_coefficient_comparator.h>          // rgb颜色 平面分割系数
#include <pcl/segmentation/edge_aware_plane_comparator.h>               // 边缘 平面分割系数
#include <pcl/segmentation/euclidean_cluster_comparator.h>              // 欧式距离分割 



#include <pcl/point_types.h>
#include <pcl/common/projection_matrix.h>
#include <pcl/common/transforms.h>
#include <pcl/common/common.h>
#include <Eigen/Core>
#include <vector>

// #include <boost/make_unique.hpp>

#include <pcl/common/time.h> // pcl::getTime () 

#include "ObjectDatabase.h"// 目标数据库
#include "Detector.h"      // 2d检测器

typedef pcl::PointXYZRGB PointT;// 点类型 xyzrgba 点+颜色+透明度
typedef pcl::PointCloud<PointT> PointCloudT;// 点云类型

struct Cluster;
class ObjectDatabase;// 目标数据库

// PCL新定义点类型   3d点坐标 + 2d的像素坐标值 3d-2d点对
struct PointXYZPixel
{
  PCL_ADD_POINT4D;// 
  uint32_t pixel_x;// 像素值
  uint32_t pixel_y;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;  // NOLINT
// 注册点类型
POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZPixel,                // xyz + pixel x, y as fields
                                  (float, x, x)                 // field x
                                  (float, y, y)                 // field y
                                  (float, z, z)                 // field z
                                  (uint32_t, pixel_x, pixel_x)  // field pixel x
                                  (uint32_t, pixel_y, pixel_y)  // field pixel y
                                  )


typedef enum Comparator
{
    kPlaneCoefficientComparator,            // 平面系数 RANSAC采样
    kEuclideanPlaneCoefficientComparator,   // 欧式距离 平面分割
    kRGBPlaneCoefficientComparator,         // 颜色距离 平面分割
    kEdgeAwarePlaneComaprator               // 边缘     平面分割
} Comparator;

// 检测结果类====
typedef struct Object3d
{
    cv::Rect_<float> rect;// 边框
    Eigen::Vector3f minPt;                       // 所有点中最小的x值，y值，z值
    Eigen::Vector3f maxPt;                       // 所有点中最大的x值，y值，z值
    Eigen::Vector3f centroid;                    // 点云中心点 齐次表示 
    Eigen::Vector3f sizePt;                      // 长宽高=== 
    Eigen::Vector3f boxCenter;                   // 包围盒中心点

} Object3d;


class MergeSG
{

public:
  /** Default constructor */
  MergeSG();

  /** Default destructor */
  ~MergeSG();

  void merge(std::vector<Object>& objects, cv::Mat depth, PointCloudT::Ptr pclMap);
// 计算 点云 坐标，3d 包围盒

// 目标数据库=====
  ObjectDatabase* mpOD;// 需要定义成指针===   公开 方便可视化器获取====

private:

// 按2d框获取 点云索引，提取对应点云，点云滤波，聚类。
  void extract(std::vector<Object>& objects, 
               PointCloudT::Ptr point_ptr, 
               std::vector<Cluster>& clusters);

  // 2d物体 和 3d物体 关系 =======
  void findMaxIntersectionRelationships(std::vector<Object>& objects,   // 2d 目标检测框 
                                        std::vector<Object3d>& object3d,// 3d点云团 带2d投影框
                                        std::vector<Cluster>& clusters);// 3d点云团 带类别信息

  // 3d点云团的 2d投影边框
  bool getProjectedROI(const pcl::PointCloud<PointXYZPixel>::ConstPtr& point_cloud,// 新类型点云 x,y,z,px,py
                       cv::Rect_<float> & roi);// 3d点云团 对应像素2d边框

  // 两边框 的 匹配相似度   IOU * distance /  AvgSize===============
  double getMatch(const cv::Rect_<float> & r1, const cv::Rect_<float> & r2);
  
  // 点云分割
  void segment(const PointCloudT::ConstPtr& cloud, // 输入点云
               PointCloudT::Ptr& cloud_segment,      // 保存的点云
               std::vector<pcl::PointIndices>& cluster_indices);// 点云团索引 数组
  // 0. 实时配置分割器参数=================================
  void applyConfig();

  // 1. 估计法线=======
  void estimateNormal(const PointCloudT::ConstPtr& cloud,
                      pcl::PointCloud<pcl::Normal>::Ptr& normal_cloud);

  // 2. 分割平面===============================
  void segmentPlanes(
    const PointCloudT::ConstPtr& cloud,    // 输入点云
    const pcl::PointCloud<pcl::Normal>::Ptr& normal_cloud,// 点云法线
    std::vector<pcl::PlanarRegion<PointT>, Eigen::aligned_allocator<pcl::PlanarRegion<PointT> > >& regions,// 平面区域
    pcl::PointCloud<pcl::Label>::Ptr labels,       // 标签
    std::vector<pcl::PointIndices>& label_indices);// 索引

  // 3. 分割目标对象=============================
  void segmentObjects(
    const PointCloudT::ConstPtr& cloud,//输入点云
    std::vector<pcl::PlanarRegion<PointT>, Eigen::aligned_allocator<pcl::PlanarRegion<PointT> > >& regions,//平面区域
    pcl::PointCloud<pcl::Label>::Ptr labels, // 点 属于那个平面的 标签
    std::vector<pcl::PointIndices>& label_indices,  // 平面 标签索引
    std::vector<pcl::PointIndices>& cluster_indices);// 索引   

  // XYZRGB点+颜色 点云  拷贝到 XYZ+像素点坐标 点云
  void copyPointCloud(const PointCloudT::ConstPtr& original, 
                      const std::vector<int>& indices,
                      pcl::PointCloud<PointXYZPixel>::Ptr& dest);

private:

  // 法线估计器
  pcl::IntegralImageNormalEstimation<PointT, pcl::Normal> normal_estimation_;

  // 平面分割器
  pcl::OrganizedMultiPlaneSegmentation<PointT, pcl::Normal, pcl::Label> plane_segmentation_;
  // 平面系数 核心
  pcl::PlaneCoefficientComparator<PointT, pcl::Normal>::Ptr plane_comparator_;
  // 欧式距离平面系数  核心
  pcl::EuclideanPlaneCoefficientComparator<PointT, pcl::Normal>::Ptr euclidean_comparator_;
  // 颜色平面分割系数  核心
  pcl::RGBPlaneCoefficientComparator<PointT, pcl::Normal>::Ptr rgb_comparator_;
  // 平面边缘分割系数  核心
  pcl::EdgeAwarePlaneComparator<PointT, pcl::Normal>::Ptr edge_aware_comparator_;

// 欧式距离聚类分割
  pcl::EuclideanClusterComparator<PointT, pcl::Normal, pcl::Label>::Ptr euclidean_cluster_comparator_;
  
  Comparator mPlanComparator;// 平面分割算法
  double   mPlanNormal_angle_threshold;// 同一个平面法线角度差值阈值(度)  2.0 0.01 45.0 
  double   normal_distance_threshold;  // 法线方向的垂直距离阈值(米)      0.02  0.001  0.1 
  int mMin_plane_inliers;// 随机采样一致性，平面最少内点数量  100 200 500 1000
  int mPlane_minimum_points;// 平面点云团 点云最少数量  300 500 1000 2000

  double mNormal_max_depth_change;// 法线计算参数 深度变化阈值(米)        0.02  0.001  0.1
  double mNormal_smooth_size;     // 法线 区域面积 平滑参数大小           30.0  1.0  100.0
  
  double mEuclidean_distance_threshold;// 欧式距离聚类分割参数(米) 超过认为是 不同的点云团 0.02 0.001 0.1 
  int mObject_minimum_points;// 物体点云团 点云最少数量 50 100 200 500 

};



#endif // MERGESG_H

