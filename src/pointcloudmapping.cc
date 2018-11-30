/*
 * 语义点云建图  pointcloudmapping.cc 类实现函数
 */

#include "pointcloudmapping.h"
#include <KeyFrame.h>
#include <opencv2/highgui/highgui.hpp>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/common/projection_matrix.h>
#include <pcl/io/pcd_io.h>
#include "Converter.h"

#include <boost/make_shared.hpp>

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>


#include <sys/time.h>
#include <unistd.h>
// 计时
long getTimeUsec()
{
    
    struct timeval t;
    gettimeofday(&t,0);
    return (long)((long)t.tv_sec*1000*1000 + t.tv_usec);
}


// 重写 Cluster的等号操作符，方便按名字查找
bool Cluster::operator ==(const std::string &x){
    return(this->object_name == x);
} 

// 类构造函数=======
PointCloudMapping::PointCloudMapping(double resolution_)
{
    this->resolution = resolution_;// 点云体素格子 尺寸大小
    voxel.setLeafSize( resolution, resolution, resolution);
    globalMap = boost::make_shared< PointCloud >(); // 全局点云地图 共享指针

    viewerThread = make_shared<thread>( bind(&PointCloudMapping::viewer, this ) );// 可视化线程 共享指针 绑定viewer()函数 
    //map_state = 0;
    //showThread   = make_shared<thread>( bind(&PointCloudMapping::update, this ) );//  点晕更新显示 绑定update()函数 
    
// 不同颜色对应不同物体 
    // std::vector<cv::Scalar> colors;
    // colors_ptr = std::make_shared< std::vector<cv::Scalar> >();
    for (int i = 0; i < 21; i++) // 带有背景
    { // voc数据集 20类物体=======
        //colors_ptr->push_back(cv::Scalar( i*10 + 40, i*10 + 40, i*10 + 40));
       colors_.push_back(cv::Scalar( i*10 + 40, i*10 + 40, i*10 + 40));
// "background","aeroplane", "bicycle", "bird", "boat","bottle", "bus", "car", "cat", "chair",
// "cow", "diningtable", "dog", "horse","motorbike", "person", "pottedplant","sheep", "sofa", "train", "tvmonitor"
    }
    colors_[5] = cv::Scalar(255,0,255); // 瓶子 粉色    bgr
    colors_[9] = cv::Scalar(255,0,0);   // 椅子 蓝色
    colors_[15] = cv::Scalar(0,0,255);  // 人 红色
    colors_[20] = cv::Scalar(0,255,0);  // 显示器 绿色 
    
// 物体尺寸大小
    for (int i = 0; i < 21; i++)  
    { // voc数据集 20类物体=======
      obj_size_.push_back(0.6);
    }
    obj_size_[5] = 0.06;  // 瓶子  0.06 米以内认为是同一个物体 
    obj_size_[9] = 0.5;   // 椅子  
    obj_size_[15] = 0.35;  // 人   
    obj_size_[20] = 0.25;  // 显示器  

    ncnn_detector_ptr = std::make_shared<Detector>();

// 统计学滤波器
   stat.setMeanK (50);	     	    // 设置在进行统计时考虑查询点临近点数  在类初始化执行
   stat.setStddevMulThresh (1.0);   // 设置判断是否为离群点的阀值
// 并将标准差的倍数设置为1  这意味着如果一
// 个点的距离超出了平均距离一个标准差以上，则该点被标记为离群点，并将它移除
// 体素格滤波器  是那个面已经有了
 
// 点晕可视化器 ====
   pcl_viewer_prt = std::make_shared<pcl::visualization::PCLVisualizer>();
   pcl_viewer_prt->setBackgroundColor(0.0, 0.0, 0.0);// 背景为黑色
   pcl_viewer_prt->setCameraPosition(
        0, 0, 0,                                // camera位置  视角
        0, 0, 3,                                // view向量 ： 观察 米为单位
        0, -1, 0                                // up向量  y方向调换
        );
    //pcl_viewer_prt->resetCamera();
    //pcl_viewer_prt->initCameraParameters ();
   //pcl_viewer_prt->addCoordinateSystem(0.5);
   //pcl_viewer_prt->setPosition(0, 0);
    pcl_viewer_prt->setSize(1280, 640);
    //pcl_viewer_prt->setSize(1280, 960);
    //pcl::PCDWriter pcdwriter;

// 点云保存器 ====
   pcd_writer_ptr = std::make_shared<pcl::PCDWriter>();
   
   map_state_ok = 0; // 
}

// 类关闭函数，类似 类析构函数===============
void PointCloudMapping::shutdown()
{
    {
        unique_lock<mutex> lck(shutDownMutex);// 执行关闭线程
        shutDownFlag = true;
        keyFrameUpdated.notify_one();// 将等待 keyFrameUpdated 条件变量对象的其中一个线程解除阻塞
    }
    viewerThread->join();// 等待 可视化线程 结束后返回
    //showThread->join();
}

// 地图中插入一个关键帧==========================
void PointCloudMapping::insertKeyFrame(KeyFrame* kf, cv::Mat& color, cv::Mat& depth, cv::Mat& imgRGB)
{
    cout<<"receive a keyframe, id = "<<kf->mnId<<endl;
    unique_lock<mutex> lck(keyframeMutex); // 对关键帧上锁
    keyframes.push_back( kf );             // 关键帧数组 加入一个关键帧
    colorImgs.push_back( color.clone() );  // 图像数组  加入一个 图像  深拷贝
    depthImgs.push_back( depth.clone() );  // 深度数据数组 加入   深拷贝
    RGBImgs.push_back( imgRGB.clone() );   // 彩色图数组

    map_state_ok = 0;
    keyFrameUpdated.notify_one(); // 关键字帧新线程 解除一个阻塞的，来进行关键帧更新，并会 触发 点云可视化线程
    
}

// 根据 关键帧中的相机参数 和 像素图(rgb 三色) 和 深度图来计算一帧的点云( 利用当前关键帧位姿变换到 世界坐标系下)============
pcl::PointCloud< PointCloudMapping::PointT >::Ptr PointCloudMapping::generatePointCloud(KeyFrame* kf, cv::Mat& color, cv::Mat& depth)
{
    PointCloud::Ptr tmp( new PointCloud() ); // 一帧 点云 共享指针
/*
    // point cloud is null ptr    这里需要判断 指针不为 null 程度
    for ( int m=0; m<depth.rows; m+=3 )      // 行  y  (每次读取3个值(rgb三色))
    {
        for ( int n=0; n<depth.cols; n+=3 )  // 列  x
        {
            float d = depth.ptr<float>(m)[n];// 对应深度图处的 深度值
            //std::cout << d << "\t" << std::endl;
            //if (d < 500 || d>6000)        // 跳过合理范围外的深度值
// 有的相机的深度值是放大1000倍的  我的相机范围是0.5～6m  500～6000
            if (d < 0.5 || d > 6) // 这里已经转化成米为单位了======
                continue;
            PointT p;
            p.z = d;  // z坐标 
            p.x = ( n - kf->cx) * p.z / kf->fx; // (x-cx)*z/fx
            p.y = ( m - kf->cy) * p.z / kf->fy; // (y-cy)*z/fy

            p.b = color.ptr<uchar>(m)[n*3+0]; // 彩色图是RGB的顺序!!!!!!!!
            p.g = color.ptr<uchar>(m)[n*3+1];
            p.r = color.ptr<uchar>(m)[n*3+2];

            tmp->points.push_back(p);// 无序点云，未设置 点云长和宽，直接push进入点云地图
        }
    }
*/

        float* dep  = (float*)depth.data;// GrabImageRGBD() 已经除以1000转换成 CV_32F
        unsigned char* col= (unsigned char*)color.data;
        tmp->resize(color.rows * color.cols);
	tmp->width    =  color.cols;  
	tmp->height   =  color.rows;// 有序点云
        tmp->is_dense = false;// 非稠密点云，会有不好的点,可能包含inf/NaN 这样的值
        
        //for(unsigned int i = 0; i < cloud->points.size(); i++)
         //{
   #pragma omp parallel for   // =======================omp 多线程 并行处理
        for(int r=0; r<color.rows; r++ ) // y
        {
         for(int c=0; c<color.cols; c++) // x
         {
            int i = r*color.cols + c;// 总索引
            float d = dep[i];
	    tmp->points[i].x = ( c - kf->cx) * d / kf->fx; // (x-cx)*z/fx
	    tmp->points[i].y = ( r - kf->cy) * d / kf->fy; // (y-cy)*z/fy
	    tmp->points[i].z = d;
	    tmp->points[i].r = col[i*3+2];
	    tmp->points[i].g = col[i*3+1];
            tmp->points[i].b = col[i*3+0];
         }
        }
            //cloud->points[i].a = 0.5;// 半透明========
        //}

    Eigen::Isometry3d T = ORB_SLAM2::Converter::toSE3Quat( kf->GetPose() );// 当前关键帧 位姿 四元素
    PointCloud::Ptr cloud(new PointCloud);// 变换到世界坐标系的点云 
    pcl::transformPointCloud( *tmp, *cloud, T.inverse().matrix());// 当前帧下的点云 变换到 世界坐标系下
    cloud->is_dense = false;// 非稠密点云，会有不好的点 nan值等

    cout<<"generate point cloud for kf "<<kf->mnId<<", size="<<cloud->points.size()<<endl;
    return cloud;
}


void PointCloudMapping::draw_rect_with_depth_threshold(
                                    cv::Mat bgr_img, 
                                    cv::Mat depth_img, 
                                    const cv::Rect_<float>& rect, const cv::Scalar& scalar,
                                    pcl::PointIndices & indices)
{

 unsigned char* color = (unsigned char*)bgr_img.data; //3 通道
 float* depth = (float*)depth_img.data;              //1 通道
 int beg = (int)rect.x + ((int)rect.y-1)*bgr_img.cols - 1;// 2d框起点

// 1. 计算平均深度=====
 int count = 0;
 float sum = 0;
 for(int k=(int)rect.height*0.3; k<(int)rect.height*0.7; k++) // 每一行
 {   
   int start = beg + k*bgr_img.cols; // 起点
   int end   = start + (int)rect.width*0.7;// 终点
   for(int j = start + (int)rect.width*0.3 ; j<end; j++)//每一列
   { 
     float d = depth[j];
     if (d < 0.5 || d > 6) // 这里已经转化成米为单位了======
         continue;
     sum += d;
     count++;
   }
 }
 float depth_threshold = 0.0;
 if(count>0) depth_threshold = sum / count;// 平均深度

// 2. 根据深度值 设置对应 图像roi的颜色值====
 for(int k=0; k< (int)rect.height-1; k++) // 每一行
 {   
   int start = beg + k*bgr_img.cols; // 起点
   int end   = start + (int)rect.width-1;// 终点
   for(int j=start; j<end-1; j++)//每一列
   {
       if( abs(depth[j] - depth_threshold)<0.4 )// 与平均深度差值为0.4m的都认为是 被检测到的物体
       {
        indices.indices.push_back (j); // 记录该点云索引
        color[j*3+0] = scalar.val[0];  // red  
        color[j*3+1] = scalar.val[1];  // green
        color[j*3+2] = scalar.val[2];  // blue
       } 
   }
 }

}

void PointCloudMapping::sem_merge(Cluster cluster)
{
    // 1. 查看总数量,数据库为空直接加入
    int num_objects = clusters.size();
    if(num_objects==0)
    {
        clusters.push_back(cluster);
        return;
    }
    else
    {
        // 2. 数据库内已经存在物体了，查找新物体是否在数据库内已经存在
	std::vector<Cluster>::iterator iter   = clusters.begin()-1;
	std::vector<Cluster>::iterator it_end = clusters.end(); 
        std::vector< std::vector<Cluster>::iterator> likely_obj;// 同名字的物体的迭代器
	while(true) 
        {
	    iter = find(++iter, clusters.end(), cluster.object_name);// 按照名字查找
	    if (iter != it_end )// 找到一个，存放起来
                likely_obj.push_back(iter);
	    else//已经找不到了
	        break;  
	}

        // 3. 如果没找到，则直接添加 进数据库
        std::vector<Cluster>::iterator best_close;// 最近的索引
        float center_distance=100;// 对应的距离
        if(likely_obj.size()==0)
        {
            clusters.push_back(cluster);
            return;
        }
        else//找到多个和数据库里同名字的物体
        {
        // 4. 遍例每一个同名字的物体，找到中心点最近的一个
            for(int j=0; j<likely_obj.size(); j++)
            {
                std::vector<Cluster>::iterator& temp_iter = likely_obj[j];
                Cluster& temp_cluster = *temp_iter;
                Eigen::Vector3f dis_vec = cluster.centroid - temp_cluster.centroid;// 中心点连接向量
                float dist = dis_vec.norm();
                if(dist < center_distance)
                {
                    center_distance = dist; // 最短的距离
                    best_close      = temp_iter;// 对应的索引
                }
            }
        
        }

        // 5. 如果距离小于物体尺寸，则认为是同一个空间中的同一个物体，更新数据库中该物体的信息
        if(center_distance < obj_size_[cluster.class_id])
            // 这个尺度对不同的物体有不同的值，可以设置一个数组存放
        {
            //Cluster& best_cluster = *best_close;
            best_close->prob = (best_close->prob + cluster.prob)/2.0; // 综合置信度
            best_close->centroid = (best_close->centroid + cluster.centroid)/2.0; // 中心平均
            // 最小值
            float min_x = best_close->minPt[0] > cluster.minPt[0] ? cluster.minPt[0] : best_close->minPt[0];
            float min_y = best_close->minPt[1] > cluster.minPt[1] ? cluster.minPt[1] : best_close->minPt[1];
            float min_z = best_close->minPt[2] > cluster.minPt[2] ? cluster.minPt[2] : best_close->minPt[2];
            // 最大值
            float max_x = best_close->maxPt[0] > cluster.maxPt[0] ? cluster.maxPt[0] : best_close->maxPt[0];
            float max_y = best_close->maxPt[1] > cluster.maxPt[1] ? cluster.maxPt[1] : best_close->maxPt[1];
            float max_z = best_close->maxPt[2] > cluster.maxPt[2] ? cluster.maxPt[2] : best_close->maxPt[2];
            // 更新
            best_close->minPt = Eigen::Vector3f(min_x,min_y,min_z);
            best_close->maxPt = Eigen::Vector3f(max_x,max_y,max_z);
        }
        else 
        {
        // 6. 如果距离超过物体尺寸则认为是不同位置的同一种物体，直接放入数据库
            clusters.push_back(cluster);
        }
    }
    return; 
}

void PointCloudMapping::add_cube(void)
{
    pcl_viewer_prt->removeAllShapes();// 去除 之前 已经显示的形状
    for(int i=0; i<clusters.size(); i++)
    {
        Cluster& cluster  = clusters[i];
        std::string& name = cluster.object_name;  // 物体类别名
        int&   class_id   = cluster.class_id;     // 类别id
        float& prob = cluster.prob;               // 置信度
	Eigen::Vector3f& minPt = cluster.minPt;   // 所有点中最小的x值，y值，z值
	Eigen::Vector3f& maxPt = cluster.maxPt;   // 所有点中最大的x值，y值，z值
	Eigen::Vector3f& centr = cluster.centroid;// 点云中心点
        Eigen::Vector3f boxCe = (maxPt + minPt)*0.5f; // 盒子中心
        Eigen::Vector3f boxSi = maxPt - minPt;        // 盒子尺寸
          
	fprintf(stderr, "3d %s %.5f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f\n", 
		        name.c_str(), prob, centr[0], centr[1], centr[2], 
                        maxPt[0], maxPt[1], maxPt[2], minPt[0], minPt[1], minPt[2]);
               // 打印名字、置信度、中心点坐标
	const Eigen::Quaternionf quat(Eigen::Quaternionf::Identity());// 姿态 四元素
	std::string name_new = name + boost::chrono::to_string(i);    // 包围框的名字
	pcl_viewer_prt->addCube(boxCe, quat, 
                                boxSi[0], boxSi[1], boxSi[2], name_new.c_str()); // 添加盒子
	pcl_viewer_prt->setShapeRenderingProperties(
                        pcl::visualization::PCL_VISUALIZER_REPRESENTATION, 
	                pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, 
			name_new.c_str());
	pcl_viewer_prt->setShapeRenderingProperties(
                        pcl::visualization::PCL_VISUALIZER_COLOR,
                        colors_[class_id].val[2]/255.0, 
                        colors_[class_id].val[1]/255.0, colors_[class_id].val[0]/255.0,// 颜色
			name_new.c_str());//       
    }
} 



// 可视化所有保存的 关键帧形成的 点云===========================
void PointCloudMapping::viewer()
{
    //pcl::PCDWriter pcdwriter;
    //pcl::visualization::CloudViewer viewer("viewer"); // pcl 点云可视化器
    //while(!pcl_viewer_prt->wasStopped ()) // 会出现段错误
    int count= 0;
    while(1)
    {
        {
            unique_lock<mutex> lck_shutdown( shutDownMutex ); // 关闭锁 
            if (shutDownFlag)
            {
                break;
            }
        }
        {
            // 更新点云 
            std::cout<< count << std::endl;
            if(count >= 1000){
              pcl_viewer_prt->spinOnce(100);// 也会出现段错误 pcl_viewer_prt 未来得及初始化
              count = 1000;
            }
            count++;

            unique_lock<mutex> lck_keyframeUpdated( keyFrameUpdateMutex ); // 关键帧更新锁
            keyFrameUpdated.wait( lck_keyframeUpdated );// 阻塞 关键帧更新锁
            // 需要等待 insertKeyFrame() 函数中完成 添加 关键帧 后，执行后面的!!!!!!!!!
        }

        // keyframe is updated
        size_t N=0;
        {
            unique_lock<mutex> lck( keyframeMutex );// 关键帧锁
            N = keyframes.size();                   // 当前 保存的 关键帧数量
            std::cout << "KeyframeSize: " << N << std::endl;
        }

        for ( size_t i=lastKeyframeSize; i<N ; i++ )// 从上一次已经可视化的关键帧开始 继续向地图中添加点云
        {
            long time = getTimeUsec();// 开始计时
            std::vector<Object> objects;
            ncnn_detector_ptr->Run(RGBImgs[i], objects); // 在彩色图上执行目标检测获取结果

            if(objects.size()>0)
                std::cout<< "detect first obj: " << objects[0].object_name << std::endl;

            std::vector<pcl::PointIndices> vec_indices; // 每个物体对应的点云索引=======
            std::vector<std::string> clusters_name;     // 点云团名字
            std::vector<float> clusters_prob;           // 点云团置信度
            std::vector<int>   clusters_class_id;       // 类别id

            for (int t = 0; t < objects.size() ; t++) 
            { // 为每一个目标 上色
	        //cv::putText(img, detector.Names(box.m_class), box.tl(), cv::FONT_HERSHEY_SIMPLEX, 1.0, colors[box.m_class], 2);
	        //cv::rectangle(img, box, colors[box.m_class], 2);
              const Object& obj = objects[t];
              if(obj.prob >0.54)// 预测准确度在 0.55以上 才认为是正确的
               {
                //const Object& obj = objects[t];
	        //cv::rectangle(colorImgs[i], obj.rect, colors_[obj.class_id], -1, 4);
                //cv::imwrite("result.jpg", colorImgs[i]);
                //cv::rectangle(RGBImgs[i], obj.rect, colors_[obj.class_id], -1, 4);// 为目标物体框内的区域然上颜色
                pcl::PointIndices indices;
                // 根据深度阈值，为 roi 区域上色
                draw_rect_with_depth_threshold(RGBImgs[i], depthImgs[i], obj.rect, colors_[obj.class_id], indices);
                vec_indices.push_back(indices);           // 点云团索引
                clusters_name.push_back(obj.object_name); // 名字
                clusters_prob.push_back(obj.prob);        // 置信度
                clusters_class_id.push_back(obj.class_id);// 类别id  
               }
            }
            
            if((i%2==0)&&(objects.size()==0)) continue; // 跳过不太重要的帧
       
            //cv::imwrite("result.jpg", colorImgs[i]);

            //PointCloud::Ptr p = generatePointCloud( keyframes[i], colorImgs[i], depthImgs[i] );// 生成新一帧的点云
             PointCloud::Ptr p = generatePointCloud( keyframes[i], RGBImgs[i], depthImgs[i] );// 生成新一帧的点云  
            // 处理每一帧 获取语义分割点云的信息===============================
            extract_indices.setInputCloud(p);// 设置输入点云 指针 
 //  #pragma omp parallel for   // =======================omp 多线程 并行处理
            for (int n = 0; n < vec_indices.size() ; n++)// 每个点云团
            {
               pcl::PointIndices & indices = vec_indices[n];       // 每个点云团的索引
               std::string&   cluster_name = clusters_name[n];    // 名字
               float          cluster_prob = clusters_prob[n];    // 置信度
               int            class_id     = clusters_class_id[n];// 类别id 
               extract_indices.setIndices (boost::make_shared<const pcl::PointIndices> (indices));
               // 设置索引=============
               PointCloud::Ptr before (new PointCloud);
               extract_indices.filter (*before);//提取对于索引的点云
               // 统计学滤波剔除 噪点==
               PointCloud::Ptr after_stat (new PointCloud);
               stat.setInputCloud (before);//设置待滤波的点云
	       stat.filter (*after_stat); //存储内点
               // 体素栅格下采样=======
               PointCloud::Ptr after_voxel (new PointCloud);
               voxel.setInputCloud( after_stat );
               voxel.filter( *after_voxel );
               // 计算点云中心=========
               Eigen::Vector4f cent;
               pcl::compute3DCentroid(*after_voxel, cent);
               // 计算点云 点范围======
               Eigen::Vector4f minPt, maxPt;
               pcl::getMinMax3D (*after_voxel, minPt, maxPt);
               
               // 新建 语义目标对象====
               Cluster cluster;
               cluster.object_name = cluster_name;// 名字
               cluster.class_id    = class_id;    // 类别id
               cluster.prob        = cluster_prob;// 置信度
               cluster.minPt       = Eigen::Vector3f(minPt[0], minPt[1], minPt[2]);// 最小值
               cluster.maxPt       = Eigen::Vector3f(maxPt[0], maxPt[1], maxPt[2]);// 大
               cluster.centroid    = Eigen::Vector3f(cent[0],  cent[1],  cent[2]); // 中心
               
               // 融合进总的 clusters
               sem_merge(cluster);

            }
           // 有序点云转换成无序点云
           std::vector<int> temp;
           PointCloud::Ptr out_pt(new PointCloud());
           pcl::removeNaNFromPointCloud(*p, *out_pt, temp);
           *globalMap += *out_pt;                   // 加入到 总的 点云地图中  
           time = getTimeUsec() - time;// 结束计时
           printf("KeyFrame %ld Semtic Mapping time: %ld ms\n", i, time/1000); // 显示检测时间
        }

        PointCloud::Ptr tmp(new PointCloud());      // 体素格滤波后的点云
        voxel.setInputCloud( globalMap );           // 体素格滤波器 输入原点云
        voxel.filter( *tmp );                       // 滤波后的点云
        globalMap->swap( *tmp );                    // 全局点云地图 替换为 体素格滤波后的点云
        map_state_ok = 1;
        /*
        viewer.showCloud( globalMap );              // 显示 点云
        cout << "show global map, size=" << globalMap->points.size() << endl;
        */

        lastKeyframeSize = N;                       // 迭代更新上次已经更新到的 关键帧id

// 显示点云 ==========  
        add_cube(); 
        pcl::visualization::PointCloudColorHandlerRGBField<PointT> rgb(globalMap);
        pcl_viewer_prt->removePointCloud("SemMap");// 去除原来的点云====
        pcl_viewer_prt->addPointCloud<PointT> (globalMap, rgb, "SemMap");
        pcl_viewer_prt->setPointCloudRenderingProperties (
                       pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "SemMap");

        while(map_state_ok)
        {
          //obj_pt = cluster_large.c_ptr;
          pcl_viewer_prt->spinOnce (100);// 
          // pcl_viewer_prt->spin(); // 太快上面的检测就做不到了
          boost::this_thread::sleep (boost::posix_time::microseconds (100000));// 
         //usleep(3000);
        }
        cout << "global map, size=" << globalMap->points.size() << endl;
        //map_state = 1;
        if(globalMap->points.size()>0)
            pcd_writer_ptr->write<PointT>("global_color.pcd", *globalMap);
    }

    //pcd_writer_ptr->write<PointT>("global_color.pcd", *globalMap);
}


// 点云更新显示
void PointCloudMapping::update()
{
std::cout<< "update"<< std::endl; 
add_cube(); 
pcl::visualization::PointCloudColorHandlerRGBField<PointT> rgb(globalMap);
pcl_viewer_prt->removePointCloud("SemMap");// 去除原来的点云====
pcl_viewer_prt->addPointCloud<PointT> (globalMap, rgb, "SemMap");
pcl_viewer_prt->setPointCloudRenderingProperties (
               pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "SemMap");	       
//obj_pt = cluster_large.c_ptr;
pcl_viewer_prt->spinOnce (100);// 
boost::this_thread::sleep (boost::posix_time::microseconds (100000));   //随时间
}



