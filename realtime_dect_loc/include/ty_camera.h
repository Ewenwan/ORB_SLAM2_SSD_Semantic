/*
 * 图漾相机类
 * 直接从图漾传感器获取 校准的rgb和点云
 */

#ifndef TY_CAMERA_H
#define TY_CAMERA_H

#include "./ty/common/common.hpp"
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <pcl/point_types.h> // 点
#include <pcl/point_cloud.h> // 点云
// 用户数据
typedef struct CallbackData {
    int             index;    // index
    TY_DEV_HANDLE   hDevice;  // 设备
    TY_CAMERA_DISTORTION color_dist;// 畸变数据
    TY_CAMERA_INTRINSIC color_intri;// 内参数
    int             rgbd_cam_enable;
} CallbackData; // 图漾 用户数据

class TYcamera
{
public:
	TYcamera();  // 构造函数
	~TYcamera(); // 析构函数
	// 捕获点云和图像=======
	void run(pcl::PointCloud<pcl::PointXYZRGB>::Ptr c_ptr, cv::Mat& color);

private:
	// 回调函数
	//void eventCallback(TY_EVENT_INFO *event_info, void *userdata);

	// 处理帧=================================================
	void handleFrame(TY_FRAME_DATA* frame, void* cbData, cv::Mat& color, pcl::PointCloud<pcl::PointXYZRGB>::Ptr c_ptr);

	// xyz点 和 彩色图像 合成 xyzrgba点云======
	void genPointCloudXYZRGBFromVec3f(cv::Mat& data_mat, cv::Mat& color_mat, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);

	CallbackData* cb_data_;// 用户数据
	//static char * buffer;
	//static char buffer[1024*1024*20];// 缓存区
	char* frameBuffer[2];   // 帧数据大小 两帧 队列  指针数组
	//cv::Mat* colorImg_ptr_;  // 彩图指针
        //pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ptr_; // rgba点云指针  boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> >
        //pcl::PointCloud<pcl::PointXYZRGB>* cloud_ptr_; // rgba点云指针
        
	TY_FRAME_DATA* frame_;  // 相机捕获的数据指针

};


#endif   // TY_CAMERA_H
