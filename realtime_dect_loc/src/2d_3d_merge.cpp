/*
 * 融合2的检测结果和点云，得到cluster
 */
#include "../include/2d_3d_merge.h"
//#include <pcl/io/pcd_io.h>
Merge::Merge()
{

// 统计学滤波器
   sta_.setMeanK (50);	     	    // 设置在进行统计时考虑查询点临近点数  在类初始化执行
   sta_.setStddevMulThresh (1.0);   // 设置判断是否为离群点的阀值

// 体素格滤波器 
   double resolution = 0.01; // 米为单位 要是放大1000倍  这里设置 10
   voxel_.setLeafSize( resolution, resolution, resolution);// 体素格子大小
// 
   last_ = boost::make_shared<Cloud>(); // 智能指针
// 
   tmp_  = boost::make_shared<Cloud>();
}

Merge::~Merge()
{
}

void Merge::extract(std::vector<Object>& objects,
                    Cloud::Ptr point_ptr,
                    std::vector<Cluster>& clusters)
{
    clusters.clear();
    extract_indices_.setInputCloud (point_ptr);            // 设置输入点云 指针 
    //std::cout << "point_width :" << point_ptr->width << "point_height :" << point_ptr->height << std::endl;
    // _width :640  _height :480

    for (int i=0; i<objects.size(); i++)// 每个2d检测结果
    //for (int i=0; i<1; i++)// 只遍历第一个2d检测结果
    {
         Cluster cluster;

     // 1. 从总点云中提取点云=====
	 Object object = objects[i];   // 2d检测结果
         std::cout << "object_x :" << object.rect.x << "object_y :" << object.rect.y << std::endl;
         if((object.rect.x + object.rect.width) > point_ptr->width-1)
                    object.rect.width = point_ptr->width - object.rect.x-1;

         if((object.rect.y + object.rect.height) > point_ptr->width-1)
                    object.rect.height = point_ptr->height - object.rect.y-1;
           

         int beg = (int)object.rect.x + ((int)object.rect.y-1)*point_ptr->width - 1;// 2d框起点

// 计算深度值均值，作为深度阈值===
	 int count = 0;
	 float sum = 0;
	 int row_beg = (int)object.rect.height*0.3;// 起始行
	 int row_end = (int)object.rect.height*0.7;
	 int col_beg = (int)object.rect.width*0.3;// 起始列
	 int col_end = (int)object.rect.width*0.7;
	 for(int k=row_beg; k< row_end; k++) // 每一行
	 { 
	   int start = beg + k*(int)point_ptr->width + col_beg; // 起点
	   int end   = beg + k*(int)point_ptr->width + col_end;// 终点
	   for(int j=start; j<end-1; j++)//每一列
	   { 
	     float d = point_ptr->points[j].z;// 深度值
	     if (d < 0.5 || d > 6.0) // 这里深度为 0.5 ～ 6,
		 continue;
	     sum += d;
	     count++;
	   }
	 }
	 float depth_threshold = 0.0;
	 if(count>0) depth_threshold = sum / count;// 平均深度

         //indices_.clear();// 清除原来的索引
         pcl::PointIndices indices;// 提取的点云索引
	 // int beg = (int)object.rect.x + ((int)object.rect.y-1)*point_ptr->width - 1;// 2d框起点
	 row_beg = (int)object.rect.height*0.1;// 起始行
	 row_end = (int)object.rect.height*0.9;
	 col_beg = (int)object.rect.width*0.1;// 起始列
	 col_end = (int)object.rect.width*0.9;
	 for(int k=row_beg; k< row_end; k++) // 每一行
	 {   
	   int start = beg + k*(int)point_ptr->width + col_beg;// 起点
	   int end   = beg + k*(int)point_ptr->width + col_end ;// 终点
	   for(int j=start; j<end-1; j++)//每一列
	   {
            if(abs(point_ptr->points[j].z - depth_threshold)<0.15) // -0.1 ～ 0.1 米
	       indices.indices.push_back (j);
	   }
         }
         extract_indices_.setIndices (boost::make_shared<const pcl::PointIndices> (indices));//设置索引
         // Cloud::Ptr last (new Cloud);   // 智能指针
         last_->clear(); // 清空
         extract_indices_.filter (*last_);//提取对于索引的点云  


      // 2. 点云滤波、聚类、分割平面======
         // TODO
	 // 创建滤波器，对每个点分析的临近点的个数设置为50 ，并将标准差的倍数设置为1  这意味着如果一
	 // 个点的距离超出了平均距离一个标准差以上，则该点被标记为离群点，并将它移除，存储起来
         //Cloud::Ptr tmp (new Cloud);     // 智能指针
         tmp_->clear(); // 清空
	 sta_.setInputCloud (last_);//设置待滤波的点云
	 sta_.filter (*tmp_); //存储内点
         last_->swap( *tmp_);

         // 体素栅格下采样、等
         tmp_->clear(); // 清空
         voxel_.setInputCloud( last_);
         voxel_.filter( *tmp_ );
         //last->swap( *tmp_ );


      // 3. 计算点云中心========
         Eigen::Vector4f centroid;
         pcl::compute3DCentroid(*tmp_, centroid);

      // 4. 计算点云 点范围=====
         Eigen::Vector4f minPt, maxPt;
         pcl::getMinMax3D (*tmp_, minPt, maxPt);
    
         Eigen::Vector4f dis_pt = maxPt-minPt;
         Eigen::Vector4f sum_pt = 0.5f*(minPt + maxPt);
         cluster.object = object;// 2d对象
         //cluster.c_ptr     = tmp_;// 3d点云团，不需要记录
         cluster.centroid  = centroid;// 点云中心
         cluster.minPt     = minPt;   // 最小的x值，y值，z值
         cluster.maxPt     = maxPt;   // 最大的x值，y值，z值
         cluster.sizePt    = Eigen::Vector3f(dis_pt[0], dis_pt[1], dis_pt[2]);// 长宽高
         cluster.boxCenter = Eigen::Vector3f(sum_pt[0], sum_pt[1], sum_pt[2]);// 包围盒中心点
         clusters.push_back(cluster);
    }
}
