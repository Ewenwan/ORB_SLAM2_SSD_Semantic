/*
 * 点云建图  pointcloudmapping.cc 类实现函数
 */

#include "pointcloudmapping.h"
#include <KeyFrame.h>
#include <opencv2/highgui/highgui.hpp>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/common/projection_matrix.h>
#include "Converter.h"

#include <boost/make_shared.hpp>
// 类构造函数=======
PointCloudMapping::PointCloudMapping(double resolution_)
{
    this->resolution = resolution_;// 点云体素格子 尺寸大小
    voxel.setLeafSize( resolution, resolution, resolution);
    globalMap = boost::make_shared< PointCloud >(); // 全局点云地图 共享指针

    viewerThread = make_shared<thread>( bind(&PointCloudMapping::viewer, this ) );// 可视化线程 共享指针 绑定viewer()函数 
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
}

// 地图中插入一个关键帧==========================
void PointCloudMapping::insertKeyFrame(KeyFrame* kf, cv::Mat& color, cv::Mat& depth)
{
    cout<<"receive a keyframe, id = "<<kf->mnId<<endl;
    unique_lock<mutex> lck(keyframeMutex); // 对关键帧上锁
    keyframes.push_back( kf );             // 关键帧数组 加入一个关键帧
    colorImgs.push_back( color.clone() );  // 图像数组  加入一个 图像  深拷贝
    depthImgs.push_back( depth.clone() );  // 深度数据数组 加入   深拷贝

    keyFrameUpdated.notify_one();          // 关键字更新线程 解除一个阻塞的，来进行关键帧更新，并会 触发 点云可视化线程
}

// 根据 关键帧中的相机参数 和 像素图(rgb 三色) 和 深度图来计算一帧的点云( 利用当前关键帧位姿变换到 世界坐标系下)============
pcl::PointCloud< PointCloudMapping::PointT >::Ptr PointCloudMapping::generatePointCloud(KeyFrame* kf, cv::Mat& color, cv::Mat& depth)
{
    PointCloud::Ptr tmp( new PointCloud() ); // 一帧 点云 共享指针
    // point cloud is null ptr    这里需要判断 指针不为 null 
    for ( int m=0; m<depth.rows; m+=3 )      // 行  y  (每次读取3个值(rgb三色))
    {
        for ( int n=0; n<depth.cols; n+=3 )  // 列  x
        {
            float d = depth.ptr<float>(m)[n];// 对应深度图处的 深度值
            if (d < 0.01 || d>10)            // 跳过合理范围外的深度值
                continue;
            PointT p;
            p.z = d;  // z坐标 
            p.x = ( n - kf->cx) * p.z / kf->fx; // (x-cx)*z/fx
            p.y = ( m - kf->cy) * p.z / kf->fy; // (y-cy)*z/fy

            p.b = color.ptr<uchar>(m)[n*3];
            p.g = color.ptr<uchar>(m)[n*3+1];
            p.r = color.ptr<uchar>(m)[n*3+2];

            tmp->points.push_back(p);// 无序点云，未设置 点云长和宽，直接push进入点云地图
        }
    }

    Eigen::Isometry3d T = ORB_SLAM2::Converter::toSE3Quat( kf->GetPose() );// 当前关键帧 位姿 四元素
    PointCloud::Ptr cloud(new PointCloud);// 变换到世界坐标系的点云 
    pcl::transformPointCloud( *tmp, *cloud, T.inverse().matrix());// 当前帧下的点云 变换到 世界坐标系下
    cloud->is_dense = false;// 非稠密点云，会有不好的点 nan值等

    cout<<"generate point cloud for kf "<<kf->mnId<<", size="<<cloud->points.size()<<endl;
    return cloud;
}

// 可视化所有保存的 关键帧形成的 点云===========================
void PointCloudMapping::viewer()
{
    pcl::visualization::CloudViewer viewer("viewer"); // pcl 点云可视化器
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
            unique_lock<mutex> lck_keyframeUpdated( keyFrameUpdateMutex ); // 关键帧更新锁
            keyFrameUpdated.wait( lck_keyframeUpdated );// 阻塞 关键帧更新锁
            // 需要等待 insertKeyFrame() 函数中完成 添加 关键帧 后，执行后面的!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        }

        // keyframe is updated
        size_t N=0;
        {
            unique_lock<mutex> lck( keyframeMutex );// 关键帧锁
            N = keyframes.size();                   // 当前 保存的 关键帧数量
        }

        for ( size_t i=lastKeyframeSize; i<N ; i++ )// 从上一次已经可视化的关键帧开始 继续向地图中添加点云
        {
            PointCloud::Ptr p = generatePointCloud( keyframes[i], colorImgs[i], depthImgs[i] );// 生成新一帧的点云
            *globalMap += *p;                       // 加入到 总的 点云地图中
        }
        PointCloud::Ptr tmp(new PointCloud());      // 体素格滤波后的点云
        voxel.setInputCloud( globalMap );           // 体素格滤波器 输入原点云
        voxel.filter( *tmp );                       // 滤波后的点云
        globalMap->swap( *tmp );                    // 全局点云地图 替换为 体素格滤波后的点云
        viewer.showCloud( globalMap );              // 显示 点云
        cout << "show global map, size=" << globalMap->points.size() << endl;
        lastKeyframeSize = N;                       // 迭代更新上次已经更新到的 关键帧id
    }
}

