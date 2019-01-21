/* This file is part of ORB-SLAM2-SSD-Semantic.
 * 语义点云建图  pointcloudmapping.cc 类实现函数
 */

#include "Merge2d3d.h"

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>

// 类构造函数=======
Merge2d3d::Merge2d3d()
{
// 统计学滤波器
   mStat.setMeanK (30);	     	    // 设置在进行统计时考虑查询点临近点数  在类初始化执行
   mStat.setStddevMulThresh (1.0);   // 设置判断是否为离群点的阀值
   // 并将标准差的倍数设置为1  这意味着如果一
   // 个点的距离超出了平均距离一个标准差以上，则该点被标记为离群点，并将它移除
// 体素格下采样 ====
    mVoxel.setLeafSize( 0.01, 0.01, 0.01);
    mpOD = new(ObjectDatabase);
}

// 类 析构函数=======
Merge2d3d::~Merge2d3d()
{
    delete mpOD;
}

void Merge2d3d::merge(std::vector<Object>& objects, cv::Mat depth, PointCloud::Ptr pclMap)
{
	if(!objects.empty())
        {
          for(unsigned int i=0; i<objects.size(); i++)
          {  
             //Cluster* pcluster = new(Cluster);
             Cluster cluster;
             bool ok = mergeOne(objects[i], cluster, depth, pclMap);
             if(ok) 
                   mpOD->addObject(cluster);
          }
        }
       // std::cout<< "OD size: " << mpOD->mClusters.size() << std::endl;
}

bool Merge2d3d::mergeOne(Object& object, Cluster& cluster, cv::Mat depth_img, PointCloud::Ptr pclMap)
{
  if(object.prob >0.54)// 预测概率
  {
	float* depth = (float*)depth_img.data;              //1 通道

	cv::Rect_<float> rect = object.rect;// 边框
	int beg = (int)rect.x + ((int)rect.y-1)*depth_img.cols - 1;// 2d框起点

// 1. 计算平均深度============
	int count = 0;
	float sum = 0;
	int row_beg = (int)rect.height*0.3;
	int row_end = (int)rect.height*0.7;
	int col_beg = (int)rect.width*0.3;
	int col_end = (int)rect.width*0.7;
	for(int k=row_beg; k<row_end; k++) // 每一行
	{   
	  int start = beg + k*depth_img.cols + col_beg; // 起点
	  int end   = beg + k*depth_img.cols + col_end ;// 终点
	  for(int j = start; j<end; j++)//每一列
	  { 
	    float d = depth[j];
	    if (d < 0.5 || d > 4.0) // 这里已经转化成米为单位了======
	      continue;
	    sum += d;
	    count++;
	  }
	}
	float depth_threshold = 0.0;
	if(count>0) depth_threshold = sum / count;// 平均深度
        else 
             return false;
// 2. 根据平均深度值 获取目标点云索引=============
	pcl::PointIndices indices;
	row_beg = (int)rect.height*0.2;
	row_end = (int)rect.height*0.8;
	col_beg = (int)rect.width*0.2;
	col_end = (int)rect.width*0.8;
	for(int k=row_beg; k<row_end; k++) // 每一行
	{   
	  int start = beg + k*depth_img.cols + col_beg; // 起点
	  int end   = beg + k*depth_img.cols + col_end ;// 终点
	  for(int j = start; j<end; j++)//每一列
	  { 
	    if( abs(depth[j] - depth_threshold) < 0.2 )// 与平均深度差值为0.4m的都认为是 被检测到的物体
	      indices.indices.push_back (j); // 记录该点云索引
	  }
	}

        if(indices.indices.size() < 50)
              return false;
// 3. 使用点云索引 提取点云================
        mExtractInd.setInputCloud(pclMap);// 设置输入点云 指针
	// 设置索引==
        mExtractInd.setIndices (boost::make_shared<const pcl::PointIndices> (indices));
	PointCloud::Ptr before (new PointCloud);
	mExtractInd.filter (*before);//提取对于索引的点云
// 4. 滤波 ===============================
	// 体素栅格下采样=======
	mVoxel.setInputCloud( before );
	mVoxel.filter( *before );
	// 统计学滤波剔除 噪点==
	PointCloud::Ptr after_stat (new PointCloud);
	mStat.setInputCloud (before);//设置待滤波的点云
	mStat.filter (*after_stat); //存储内点
        if(after_stat->width * after_stat->height< 30)
              return false;
// 5. 计算点云团参数 ======================
	// 计算点云中心=========
	Eigen::Vector4f cent;
	pcl::compute3DCentroid(*after_stat, cent);
	// 计算点云 点范围======
	Eigen::Vector4f minPt, maxPt;
	pcl::getMinMax3D (*after_stat, minPt, maxPt);

	// 新建 语义目标对象====
	cluster.object_name = object.object_name;// 名字
	cluster.class_id    = object.class_id;    // 类别id
	cluster.prob        = object.prob;// 置信度
	//cluster->minPt       = Eigen::Vector3f(minPt[0], minPt[1], minPt[2]);// 最小值
	//cluster->maxPt       = Eigen::Vector3f(maxPt[0], maxPt[1], maxPt[2]);// 大
        cluster.size        = Eigen::Vector3f(maxPt[0]-minPt[0], maxPt[1]-minPt[1], maxPt[2]-minPt[2]);// 尺寸
	cluster.centroid    = Eigen::Vector3f(cent[0],  cent[1],  cent[2]); // 中心点

        return true;
   }
   return false;
}


