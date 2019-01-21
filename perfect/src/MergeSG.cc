/* This file is part of ORB-SLAM2-SSD-Semantic.
2d & 3d 融合算法
点云 多平面分割、聚类
反向投影获取点晕团的2d框
和目标检测2d框 匹配
获取每个点云团的类别信息，以及3d边框信息
返回 cluster 数组信息


pcl/apps/src/organized_segmentation_demo.cpp.

*/

#include "MergeSG.h"

MergeSG::MergeSG():
// 平面分割
plane_comparator_(new pcl::PlaneCoefficientComparator<PointT, pcl::Normal>),
euclidean_comparator_(new pcl::EuclideanPlaneCoefficientComparator<PointT, pcl::Normal>),
rgb_comparator_(new pcl::RGBPlaneCoefficientComparator<PointT, pcl::Normal>),
edge_aware_comparator_(new pcl::EdgeAwarePlaneComparator<PointT, pcl::Normal>),
// 欧式距离聚类分割
euclidean_cluster_comparator_(new pcl::EuclideanClusterComparator<PointT, pcl::Normal, pcl::Label>)
{

  mPlanComparator = kPlaneCoefficientComparator;// 平面系数 RANSAC采样
  mPlanNormal_angle_threshold = 3.0;// 同一个平面法线角度差值阈值(度)  2.0 0.01 45.0 
  normal_distance_threshold = 0.02;  // 法线方向的垂直距离阈值(米)      0.02  0.001  0.1 
  mMin_plane_inliers = 10000;// 随机采样一致性，平面最少内点数量  1000 - 10000

  mPlane_minimum_points=10000;// 平面点云团 点云最少数量  1000 2000 3000 - 20000

  mNormal_max_depth_change = 0.02;      // 法线计算参数 深度变化阈值(米)        0.02  0.001  0.1
  mNormal_smooth_size = 20.0f;          // 法线 区域面积 平滑参数大小           30.0  1.0  100.0

  mEuclidean_distance_threshold = 0.01f;// 欧式距离聚类分割参数(米) 超过认为是 不同的点云团 0.01 0.001 0.1 

  mObject_minimum_points = 1000;// 物体点云团 点云最少数量 50 100 200 - 5000

  applyConfig();// 0. 实时配置分割器参数=====


  mpOD = new(ObjectDatabase);// 新建目标数据库
}

MergeSG::~MergeSG()
{
 //  delete plane_comparator_;
 //  delete euclidean_comparator_;
 //  delete rgb_comparator_;
 //  delete edge_aware_comparator_;
 //  delete euclidean_cluster_comparator_;
 
 delete mpOD;// 目标数据库====
}


void MergeSG::merge(std::vector<Object>& objects, cv::Mat depth, PointCloudT::Ptr pclMap)
{

     std::vector<Cluster> clusters;
     extract(objects, pclMap, clusters);// 提取3d目标=====

     for(std::vector<Cluster>::iterator it = clusters.begin();
         it != clusters.end(); ++it)
         mpOD->addObject(*it); // 加入数据库======
}



// 点云分割聚类 点云团反向投影 与 2d检测框 相似度
// 获取每个点云团的类别信息，以及3d边框信息，返回 cluster 数组信息
void MergeSG::extract(std::vector<Object>& objects, 
                      PointCloudT::Ptr point_ptr,    // 有序点云=====
                      std::vector<Cluster>& clusters)
{   
   clusters.clear();// 数据库清空======
// 1. 点云团分割，获取点云团================================= 
   std::vector<pcl::PointIndices> cluster_indices; // 点云团索引 数组=====
   PointCloudT::Ptr cloud_segment(new PointCloudT);// 保存的点云，有序点云====

   segment(point_ptr, cloud_segment, cluster_indices);

   //std::cout << "cluster  size  " << cluster_indices.size() << std::endl;

   std::vector<Object3d> object3ds;// 3d点云团

   for ( std::vector<pcl::PointIndices>::iterator indices_it = cluster_indices.begin();
         indices_it != cluster_indices.end(); ++indices_it )
   {
     try
     {
         pcl::PointCloud<PointXYZPixel>::Ptr seg(new pcl::PointCloud<PointXYZPixel>);
         copyPointCloud(cloud_segment, indices_it->indices, seg);// 由点云 indices 除以 / 取余 图像宽度 得到像素坐标
         // 求取点云范围=====
         cv::Rect_<float> obj_roi;
         bool ok = getProjectedROI(seg, obj_roi); // 获取点云反投影的 2d框====
         if(!ok) continue;

         //std::cout << "3dobject_roi: " << obj_roi.x     << " " 
         //                            << obj_roi.y     << " "
         //                            << obj_roi.width << " "
         //                            << obj_roi.height << std::endl;

         // lambda 函数
         auto cmp_x = [](PointXYZPixel const& l, PointXYZPixel const& r) { return l.x < r.x; };
         auto minmax_x = std::minmax_element(seg->begin(), seg->end(), cmp_x);// 点云x最大

         auto cmp_y = [](PointXYZPixel const& l, PointXYZPixel const& r) { return l.y < r.y; };
         auto minmax_y = std::minmax_element(seg->begin(), seg->end(), cmp_y);// 点云x最大

         auto cmp_z = [](PointXYZPixel const& l, PointXYZPixel const& r) { return l.z < r.z; };
         auto minmax_z = std::minmax_element(seg->begin(), seg->end(), cmp_z);// 点云x最大  
         
         // 边框中心点=====
         auto sum_x = [](double sum_x, PointXYZPixel const& l){return sum_x + l.x;};
	 auto sumx = std::accumulate(seg->begin(), seg->end(), 0.0, sum_x);
	 double mean_x =  sumx / seg->size(); //均值

         auto sum_y = [](double sum_y, PointXYZPixel const& l){return sum_y + l.y;};
	 auto sumy = std::accumulate(seg->begin(), seg->end(), 0.0, sum_y);
	 double mean_y =  sumy / seg->size(); //均值

         auto sum_z = [](double sum_z, PointXYZPixel const& l){return sum_z + l.z;};
	 auto sumz = std::accumulate(seg->begin(), seg->end(), 0.0, sum_z);
	 double mean_z =  sumz / seg->size(); //均值

         Object3d object3d;
         object3d.rect     = obj_roi; // 3d点云对应的 2d图像边框====
         object3d.minPt    = Eigen::Vector3f(minmax_x.first->x, minmax_y.first->y, minmax_z.first->z);
         object3d.maxPt    = Eigen::Vector3f(minmax_x.second->x,minmax_y.second->y,minmax_z.second->z);
         object3d.centroid = Eigen::Vector3f(mean_x, mean_y, mean_z); // 均值中心
         // 3d边框
         object3d.sizePt   = Eigen::Vector3f(object3d.maxPt[0]-object3d.minPt[0],
                                             object3d.maxPt[1]-object3d.minPt[1],
                                             object3d.maxPt[2]-object3d.minPt[2]);
         // 3d边框中心===
         object3d.boxCenter= Eigen::Vector3f(object3d.minPt[0]+object3d.sizePt[0]/2.0,
                                             object3d.minPt[1]+object3d.sizePt[1]/2.0,
                                             object3d.minPt[2]+object3d.sizePt[2]/2.0);

         object3ds.push_back(object3d);
     }
     catch (std::exception& e)
     {
        std::cout << e.what() << std::endl;// 错误信息
     }
   }
   
   // std::cout << "object3ds size  " << object3ds.size() << std::endl;
   // 没有 分割后的目标=== 参数===
// 2. 与2d检测框融合，生成带有标签的3d目标物体
   findMaxIntersectionRelationships(objects, object3ds, clusters);
}




// 2d物体 和 3d物体 关系 ==============================================
// 遍历 每一个2d物体
//     遍历   每一个3d物体
//        计算 2d物体边框 和3d物体投影2d边框的相似度  两边框的匹配相似度 match = IOU * distance /  AvgSize
//        记录和 该2d边框最相似的 3d物体id
void MergeSG::findMaxIntersectionRelationships(std::vector<Object>& objects,   // 2d 目标检测框 
                                               std::vector<Object3d>& object3d,// 3d点云团 带2d投影框
                                               std::vector<Cluster>& clusters) // 3d点云团 带类别信息
{
  for (std::vector<Object>::iterator obj2d_it = objects.begin();
       obj2d_it != objects.end(); ++obj2d_it)// 每一个2d物体
  {
    std::vector<Object3d>::iterator max_it = object3d.begin();// 3d物体id
    double max = 0;
    cv::Rect_<float>  rect2d = obj2d_it->rect; // 2d边框
    

    //std::cout << "2dobject_roi: " << rect2d.x     << " " 
    //                              << rect2d.y     << " "
    //                              << rect2d.width << " "
    //                              << rect2d.height << std::endl;


    
    for (std::vector<Object3d>::iterator it = max_it; 
         it != object3d.end(); ++it)// 每一个3d物体
    {
      cv::Rect_<float> rect3d = it->rect;    // 3d物体 roi 3d点云投影到2d平面后的2d边框
      double area = getMatch(rect2d, rect3d);// 两边框 的 匹配相似度   IOU * distance /  AvgSize

     // std::cout << "match: " << area << std::endl;

      if (area < max)
      {
        continue;
      }

      max = area;
      max_it = it;  // 为每一个 2d边框 寻找 一个 匹配度最高的 3d物体===========
    }

    if (max <= 0)
    {
      std::cout << "Cannot find correlated 3D object " << std::endl;
      continue;
    }
  
    // 3d对象=========

    Cluster cluster;
/*
    cluster.object.rect = obj2d_it->rect;      // 2d对象
    cluster.object.prob = obj2d_it->prob;// 2d对象
    cluster.object.object_name = obj2d_it->object_name;// 2d对象

    cluster.centroid  = max_it->centroid;// 点云中心
    cluster.minPt     = max_it->minPt;   // 最小的x值，y值，z值
    cluster.maxPt     = max_it->maxPt;   // 最大的x值，y值，z值
    cluster.sizePt    = max_it->sizePt;// 长宽高
    cluster.boxCenter = max_it->boxCenter;// 包围盒中心点
*/


    cluster.object_name = obj2d_it->object_name;// 名字
    cluster.class_id    = obj2d_it->class_id;    // 类别id
    cluster.prob        = obj2d_it->prob;// 置信度

    cluster.size        = max_it->sizePt;// 尺寸
    cluster.centroid    = max_it->centroid; // 中心点  // max_it->boxCenter; // 包围盒中心点

    clusters.push_back(cluster);

    object3d.erase(max_it);// 删除已经匹配的3d点云物体
  }
}



//  2d 像素点集 获取 对应的roi边框 min_x, min_y, max_x-min_x, max_y-min_y================
//  从 3d+2d点云团里获取 2droi边框       ======
// PointXYZPixel = 3d点 + 2d 像素点坐标======
// 计算 3d物体点云集合 投影在 相机平面上的 2d框 ROI=====
bool MergeSG::getProjectedROI(const pcl::PointCloud<PointXYZPixel>::ConstPtr& point_cloud,// 新类型点云 x,y,z,px,py
                              cv::Rect_<float> & roi)// 3d点云团 对应像素2d边框
{
// lambda 函数
  auto cmp_x = [](PointXYZPixel const& l, PointXYZPixel const& r) { return l.pixel_x < r.pixel_x; };
  
  auto minmax_x = std::minmax_element(point_cloud->begin(), point_cloud->end(), cmp_x);// 点云对应像素点坐标， pixel_x的最大最小值
  
  roi.x = minmax_x.first->pixel_x;// x_offset 框的左上角点, x坐标最小值
  auto max_x = minmax_x.second->pixel_x;// x坐标最大值
  if(roi.x >= 0 && max_x >= roi.x) 
  {
    roi.width = max_x - roi.x;// 2d框 宽度   max_x - min_x

    auto cmp_y = [](PointXYZPixel const& l, PointXYZPixel const& r) { return l.pixel_y < r.pixel_y; };
    auto minmax_y = std::minmax_element(point_cloud->begin(), point_cloud->end(), cmp_y);// 点云对应像素点坐标， pixel_y的最大最小值
    roi.y = minmax_y.first->pixel_y; // y_offset 框的左上角点, y坐标最小值
    auto max_y = minmax_y.second->pixel_y;// y坐标最大值
    if(roi.y >= 0 && max_y >= roi.y)
    { 
        roi.height = max_y - roi.y;       //  2d框 高度   max_y - min_x
        return true;
    }  
    return false;  
  }
  return false;
}

// 两边框 的 匹配相似度   IOU * distance /  AvgSize===============
double MergeSG::getMatch(const cv::Rect_<float> & r1, const cv::Rect_<float> & r2)
{
  cv::Rect2i ir1(r1), ir2(r2);
  /* calculate center of rectangle #1  边框中心点 */
  cv::Point2i c1(ir1.x + (ir1.width >> 1), ir1.y + (ir1.height >> 1));// 边框 中心点1
  /* calculate center of rectangle #2  边框中心点 */
  cv::Point2i c2(ir2.x + (ir2.width >> 1), ir2.y + (ir2.height >> 1));// 边框 中心点2

  double a1 = ir1.area(), a2 = ir2.area(), a0 = (ir1 & ir2).area();// opencv 的 矩形支持 &并集 运算符
  /* calculate the overlap rate*/
  double overlap = a0 / (a1 + a2 - a0);// IOU 交并比
  /* calculate the deviation between centers #1 and #2*/
  double deviate = sqrt(powf((c1.x - c2.x), 2) + powf((c1.y - c2.y), 2));// 边框中心点 距离 距离近相似
  /* calculate the length of diagonal for the rectangle in average size*/
  // 使用 平均尺寸  进行匹配度 加权 =====================================================
  double len_diag = sqrt(powf(((ir1.width + ir2.width) >> 1), 2) + powf(((ir1.height + ir2.height) >> 1), 2));

  /* calculate the match rate. The more overlap, the more matching. Contrary, the more deviation, the less matching*/

  return overlap * len_diag / deviate;
}


// 点云分割相关算法==============================

void MergeSG::segment(const PointCloudT::ConstPtr& cloud, // 输入点云
                      PointCloudT::Ptr& cloud_segment,    // 保存的点云
                      std::vector<pcl::PointIndices>& cluster_indices)// 点云团索引 数组
{

  std::cout << "Total original point size = " << cloud->size() << std::endl;

  pcl::copyPointCloud(*cloud, *cloud_segment);  // 保存的点云，有序点云====

  applyConfig();// 0. 实时配置分割器参数=====

  pcl::PointCloud<pcl::Normal>::Ptr normal_cloud(new pcl::PointCloud<pcl::Normal>);//法线
  estimateNormal(cloud, normal_cloud);// 1. 估计法线=======

  std::vector<pcl::PlanarRegion<PointT>, Eigen::aligned_allocator<pcl::PlanarRegion<PointT> > > regions;// 平面区域?
  pcl::PointCloud<pcl::Label>::Ptr labels(new pcl::PointCloud<pcl::Label>);// 点云标签，属于那个平面====
  std::vector<pcl::PointIndices> label_indices;// 平面点云团索引===

  segmentPlanes(cloud, normal_cloud, regions, labels, label_indices);// 2. 分割平面============

  std::cout << "find plane : " << label_indices.size() << std::endl; // 平面数量 ====
 
  segmentObjects(cloud, regions, labels, label_indices, cluster_indices);// 3. 分割目标对象====

}

// 1. 估计法线=======
void MergeSG::estimateNormal(const PointCloudT::ConstPtr& cloud,
                             pcl::PointCloud<pcl::Normal>::Ptr& normal_cloud)
{
  normal_estimation_.setInputCloud(cloud);// 输入点云
  normal_estimation_.compute(*normal_cloud);// 计算点云法线

// 设置边缘平面分割系数??
  float* distance_map = normal_estimation_.getDistanceMap();
  boost::shared_ptr<pcl::EdgeAwarePlaneComparator<PointT, pcl::Normal> > eapc =
      boost::dynamic_pointer_cast<pcl::EdgeAwarePlaneComparator<PointT, pcl::Normal> >(edge_aware_comparator_);
  eapc->setDistanceMap(distance_map);
  eapc->setDistanceThreshold(0.01f, false);

}

// 2. 分割平面===============================
void MergeSG::segmentPlanes(
    const PointCloudT::ConstPtr& cloud,    // 输入点云
    const pcl::PointCloud<pcl::Normal>::Ptr& normal_cloud,// 点云法线
    std::vector<pcl::PlanarRegion<PointT>, Eigen::aligned_allocator<pcl::PlanarRegion<PointT> > >& regions,// 平面区域
    pcl::PointCloud<pcl::Label>::Ptr labels,       // 标签
    std::vector<pcl::PointIndices>& label_indices) // 索引
{

  double mps_start = pcl::getTime ();
  std::vector<pcl::ModelCoefficients> model_coefficients;// 平面模型系数
  std::vector<pcl::PointIndices> inlier_indices;    // 平面上的点 的索引
  std::vector<pcl::PointIndices> boundary_indices;  // 其他索引

  plane_segmentation_.setInputNormals(normal_cloud);// 输入点云法线
  plane_segmentation_.setInputCloud(cloud);         // 输入点云
  // 执行平面分割====
  plane_segmentation_.segmentAndRefine(regions, 
                                       model_coefficients, 
                                       inlier_indices, 
                                       labels, 
                                       label_indices,
                                       boundary_indices);
  // mps.segment (regions); // not refinement ====
  double mps_end = pcl::getTime ();
  std::cout << "MPS+Refine took: " << double(mps_end - mps_start) << std::endl;
}


// 3. 分割目标对象=============================
void MergeSG::segmentObjects(
    const PointCloudT::ConstPtr& cloud,//输入点云
    std::vector<pcl::PlanarRegion<PointT>, Eigen::aligned_allocator<pcl::PlanarRegion<PointT> > >& regions,//平面区域
    pcl::PointCloud<pcl::Label>::Ptr labels, // 点 属于那个平面的 标签
    std::vector<pcl::PointIndices>& label_indices,  // 平面 标签索引
    std::vector<pcl::PointIndices>& cluster_indices)// 索引
{

// 更新平面对象=========================
  std::vector<bool> plane_labels;// 平面否? 数量过少，不认为是平面=====
  plane_labels.resize(label_indices.size(), false);// 默认设置为 否
  for (size_t i = 0; i < label_indices.size(); i++)
  {
    // 平面点最少数量===
    if (label_indices[i].indices.size() > mPlane_minimum_points)
    {
      plane_labels[i] = true;// 该点湍数量大于阈值，设置为 平面点与团====
    }
  }

// 欧式距离聚类分割
  euclidean_cluster_comparator_->setInputCloud(cloud);// 点云
  euclidean_cluster_comparator_->setLabels(labels);   // 点 属于那个平面的 标签
  euclidean_cluster_comparator_->setExcludeLabels(plane_labels);// 平面是否为平面的标签

  pcl::PointCloud<pcl::Label> euclidean_labels;// 聚类分割点云团
  
// 点云聚类分割器===========
  pcl::OrganizedConnectedComponentSegmentation<PointT, pcl::Label> 
                   euclidean_segmentation(euclidean_cluster_comparator_); 

  euclidean_segmentation.setInputCloud(cloud);// 输入点云
  euclidean_segmentation.segment(euclidean_labels, cluster_indices);// 点云团索引=====

// 匿名函数== 判断点云团是否足够大===
  auto func = [this](pcl::PointIndices indices) { return indices.indices.size() < this->mObject_minimum_points; };
// 删除过小的点云团
  cluster_indices.erase(std::remove_if(cluster_indices.begin(), cluster_indices.end(), func), cluster_indices.end());

  PCL_INFO ("Got %d euclidean clusters!\n", cluster_indices.size ());

}


// 0. 实时配置分割器参数=================================
void MergeSG::applyConfig()
{

// 法线估计器参数=====
  //normal_estimation_.setNormalEstimationMethod(normal_estimation_.SIMPLE_3D_GRADIENT);
  normal_estimation_.setNormalEstimationMethod(normal_estimation_.COVARIANCE_MATRIX);
  normal_estimation_.setMaxDepthChangeFactor(mNormal_max_depth_change);// 法线计算参数 深度变化阈值(米)
  normal_estimation_.setNormalSmoothingSize(mNormal_smooth_size);      // 平滑因子，法线 区域面积 平滑参数大小

// 欧式距离聚类分割器参数=====
   // 欧式距离聚类分割参数(米) 超过认为是 不同的点云团 0.02 0.001 0.1 
  euclidean_cluster_comparator_->setDistanceThreshold(mEuclidean_distance_threshold, false);


// 平面分割器参数=====
  plane_segmentation_.setMinInliers(mMin_plane_inliers);// 采样一致性算法，最少内点数量
  plane_segmentation_.setAngularThreshold(pcl::deg2rad(mPlanNormal_angle_threshold));// 平面相邻点法线角度差阈值
  plane_segmentation_.setDistanceThreshold(normal_distance_threshold);          // 法线方向的垂直距离阈值

  if (mPlanComparator == kPlaneCoefficientComparator)
  {
    plane_segmentation_.setComparator(plane_comparator_);// 平面参数
  }
  else if (mPlanComparator == kEuclideanPlaneCoefficientComparator)
  {
    plane_segmentation_.setComparator(euclidean_comparator_);// 欧式距离分割 平面
  }
  else if (mPlanComparator == kRGBPlaneCoefficientComparator)
  {
    plane_segmentation_.setComparator(rgb_comparator_);// 颜色分割平面
  }
  else if (mPlanComparator == kEdgeAwarePlaneComaprator)
  {
    plane_segmentation_.setComparator(edge_aware_comparator_);// 边缘分割平面
  }
}



// XYZRGB点+颜色 点云  拷贝到 XYZ+像素点坐标 点云
void MergeSG::copyPointCloud(const PointCloudT::ConstPtr& original, 
                                 const std::vector<int>& indices,
                                 pcl::PointCloud<PointXYZPixel>::Ptr& dest)
{
  pcl::copyPointCloud(*original, indices, *dest);// 拷贝 3d点坐标
  uint32_t width = original->width;              // 有序点云，相当于图像宽度
  for (uint32_t i = 0; i < indices.size(); i++)  // 子点云序列
  {
    dest->points[i].pixel_x = indices[i] % width;// 列坐标
    dest->points[i].pixel_y = indices[i] / width;// 行坐标
  }
}



