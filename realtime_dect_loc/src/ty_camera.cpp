/*
 * 图漾相机类
 * 直接从图漾传感器获取 校准的rgb和点云
 */
#include "../include/ty_camera.h"

// 回调函数=========================
void eventCallback(TY_EVENT_INFO *event_info, void *userdata)
{
    if (event_info->eventId == TY_EVENT_DEVICE_OFFLINE) 
    {
        LOGD("=== Event Callback: Device Offline!");
// Note: 
 //     Please set TY_BOOL_KEEP_ALIVE_ONOFF feature to false if you need to debug with breakpoint!
    }
    else if (event_info->eventId == TY_EVENT_LICENSE_ERROR) 
    {
        LOGD("=== Event Callback: License Error!");
    }
}

// 构造函数======================
TYcamera::TYcamera()
{
// 1. 初始化API===
    static char buffer[1024*1024*20];
    //LOGD("=== Init lib");
    ASSERT_OK( TYInitLib() );
    TY_VERSION_INFO* pVer = (TY_VERSION_INFO*)buffer; 
    ASSERT_OK( TYLibVersion(pVer) );
    // LOGD("     - lib version: %d.%d.%d", pVer->major, pVer->minor, pVer->patch); 
    static int  n;
    ASSERT_OK( TYGetDeviceNumber(&n) );// 设备 数量
    TY_DEVICE_BASE_INFO* pBaseInfo = (TY_DEVICE_BASE_INFO*)buffer;
    ASSERT_OK( TYGetDeviceList(pBaseInfo, 100, &n) );// 设备列表
    if(n == 0)
    {
         LOGD("=== No device got");
         return;
    }
    const char* ID = pBaseInfo[0].id;
    TY_DEV_HANDLE hDevice;// 设备 头
    ASSERT_OK( TYOpenDevice( ID, &hDevice) );// 打开第一个设备

// 配置组件 
    int32_t allComps;
    ASSERT_OK( TYGetComponentIDs(hDevice, &allComps) );
    if(!(allComps & TY_COMPONENT_RGB_CAM)){
        LOGE("=== Has no RGB camera, cant do registration");
        return;
    } 
    int32_t componentIDs = TY_COMPONENT_POINT3D_CAM | TY_COMPONENT_RGB_CAM;
    ASSERT_OK( TYEnableComponents(hDevice, componentIDs) );// 使能点云 和 RGB 组件
    

// 2. 配置参数
    TY_FEATURE_INFO info;
    TY_STATUS ty_status = TYGetFeatureInfo(hDevice, TY_COMPONENT_DEPTH_CAM, TY_ENUM_IMAGE_MODE, &info);
    if ((info.accessMode & TY_ACCESS_WRITABLE) && (ty_status == TY_STATUS_OK)) 
    { 
      // 设置分辨率 深度图======
      int err = TYSetEnum(hDevice,            TY_COMPONENT_DEPTH_CAM, 
                          TY_ENUM_IMAGE_MODE, TY_IMAGE_MODE_640x480);
        ASSERT(err == TY_STATUS_OK || err == TY_STATUS_NOT_PERMITTED);   
    } 
    
    ty_status = TYGetFeatureInfo(hDevice, TY_COMPONENT_RGB_CAM, TY_ENUM_IMAGE_MODE, &info);
    if ((info.accessMode & TY_ACCESS_WRITABLE) && (ty_status == TY_STATUS_OK)) 
    { 
      // 设置分辨率 彩色图======
      int err = TYSetEnum(hDevice,            TY_COMPONENT_RGB_CAM, 
                          TY_ENUM_IMAGE_MODE, TY_IMAGE_MODE_640x480);
        ASSERT(err == TY_STATUS_OK || err == TY_STATUS_NOT_PERMITTED);   
    } 

// 3. 配置帧缓冲区
    int32_t frameSize;
    //frameSize = 1280 * 960 * (3 + 2 + 2);// 彩色图 默认为 1280*960
    // 若配置为 640*480 则: frameSize = 640 * 480 * (3 + 2 + 2)
    ASSERT_OK( TYGetFrameBufferSize(hDevice, &frameSize) );
    frameBuffer[0] = new char[frameSize];
    frameBuffer[1] = new char[frameSize];
    ASSERT_OK( TYEnqueueBuffer(hDevice, frameBuffer[0], frameSize) );
    ASSERT_OK( TYEnqueueBuffer(hDevice, frameBuffer[1], frameSize) );


    cb_data_ = new(CallbackData);
    cb_data_->index = 0;
    cb_data_->hDevice = hDevice;// 设备头

    // 注册 事件回调函数 
    ASSERT_OK(TYRegisterEventCallback(hDevice, eventCallback, NULL));

// 6. 关闭触发模式=====
    //LOGD("=== Disable trigger mode");
    ASSERT_OK( TYSetBool(hDevice, TY_COMPONENT_DEVICE, TY_BOOL_TRIGGER_MODE, false) );

// 7. 启动采集
    LOGD("=== Start capture");
    ASSERT_OK( TYStartCapture(hDevice) );

// 从API获取相机参数 ======
    TY_CAMERA_DISTORTION color_dist;// 相机 畸变参数
    TY_CAMERA_INTRINSIC color_intri;// 相机 内参数
    TY_STATUS ret = TYGetStruct(hDevice, TY_COMPONENT_RGB_CAM, TY_STRUCT_CAM_DISTORTION, &color_dist, sizeof(color_dist));
    ret |= TYGetStruct(hDevice, TY_COMPONENT_RGB_CAM, TY_STRUCT_CAM_INTRINSIC, &color_intri, sizeof(color_intri));
    if (ret == TY_STATUS_OK)
    {
       cb_data_->color_intri = color_intri;// 相机 内参数
       cb_data_->color_dist= color_dist;   // 相机 畸变参数
    }
    else
    { //reading data from device failed .set some default values....
       memset(cb_data_->color_dist.data, 0, 12 * sizeof(float));
       // 畸变参数 k1,k2,p1,p2,k3,k4,k5,k6,s1,s2,s3,s4
       memset(cb_data_->color_intri.data, 0, 9 * sizeof(float));// 内参数
       cb_data_->color_intri.data[0] = 1000.f;// fx
       cb_data_->color_intri.data[4] = 1000.f;// fy
       cb_data_->color_intri.data[2] = 600.f;// cx
       cb_data_->color_intri.data[5] = 450.f;// cy
    }

    LOGD("=== cam init ok ===");
    
    frame_        = new(TY_FRAME_DATA);
    //colorImg_ptr_ = new(cv::Mat);
    //cloud_ptr_    = new(pcl::PointCloud<pcl::PointXYZRGB>());
    //cloud_ptr_ (new pcl::PointCloud<pcl::PointXYZRGB>);
    cb_data_->rgbd_cam_enable = 1;// 设置使能了
}

// ===析构函数======
 TYcamera::~TYcamera()
{

	//delete cloud_ptr;// 共享指针自动管理

	if(cb_data_->rgbd_cam_enable)
	{
	    ASSERT_OK( TYStopCapture(cb_data_->hDevice) );// 停止采集
	    ASSERT_OK( TYCloseDevice(cb_data_->hDevice) );// 关闭设备
	    ASSERT_OK( TYDeinitLib() );         // 关闭API
	    
	    delete cb_data_;
	    //delete []buffer; // 删除数组
	    delete frame_; 

	    delete frameBuffer[0];
	    delete frameBuffer[1];
	    //delete colorImg_ptr_; 
            //delete cloud_ptr_;
	}
}

void TYcamera::run(pcl::PointCloud<pcl::PointXYZRGB>::Ptr c_ptr, cv::Mat& color)
//void  ty_rgbd_run(sensor_msgs::PointCloud2Ptr points)
{
        int err = TYFetchFrame(cb_data_->hDevice, frame_, -1);// 捕获 一帧数据
        if( err != TY_STATUS_OK ) 
        {
            LOGE("Fetch frame error %d: %s", err, TYErrorString(err));
            // cb_data_->rgbd_cam_enable = 0;
            return;
        } 
        else 
        {

          TYcamera::handleFrame(frame_, cb_data_, color, c_ptr);// 传入用户数据
          //LOGD("=== get xyzrgb point cloud, WIDTH: %d, Hight: %d ", c_ptr->width, c_ptr->height);
          //LOGD("=== get picture, WIDTH: %d, Hight: %d ", color.rows, color.cols);
          ASSERT_OK( TYEnqueueBuffer(cb_data_->hDevice, frame_->userBuffer, frame_->bufferSize) );
         // 使能队列
          
        }
}

// 处理帧=================================================
void TYcamera::handleFrame(TY_FRAME_DATA* frame, void* cbData, cv::Mat& color, pcl::PointCloud<pcl::PointXYZRGB>::Ptr c_ptr)
{

    CallbackData* pData = (CallbackData*)(cbData);

    //cv::Mat depth, irl, irr, color, point3D;
    //parseFrame(*frame, &depth, &irl, &irr, &color, &point3D);
    cv::Mat point3D;
    // cv::Mat color = Img;
    parseFrame(*frame, 0, 0, 0, &color, &point3D);// 这个3d点和 rgb可能没有配准，需要使用配准后的深度图产生配准的点云
    if(!point3D.empty() && !color.empty())
    {      
        cv::Mat undistort_result(color.size(), CV_8UC3);
        //undistort_result = cv::Mat(color.size(), CV_8UC3);
        TY_IMAGE_DATA dst;        // 目标图像
        dst.width = color.cols;   // 宽度 列数
        dst.height = color.rows;  // 高度 行数
        dst.size = undistort_result.size().area() * 3;// 3通道 
        dst.buffer = undistort_result.data;
        dst.pixelFormat = TY_PIXEL_FORMAT_RGB; // RGB 格式
        TY_IMAGE_DATA src;        // 源图像=================
        src.width = color.cols;
        src.height = color.rows;
        src.size = color.size().area() * 3;
        src.pixelFormat = TY_PIXEL_FORMAT_RGB;
        src.buffer = color.data; 
        ASSERT_OK(TYUndistortImage(&pData->color_intri, &pData->color_dist, NULL, &src, &dst));
        color = undistort_result.clone();// 畸变矫正后的图像= 拷贝数据====  这里可以优化=======

        TYcamera::genPointCloudXYZRGBFromVec3f(point3D, undistort_result, c_ptr);

    }
    ASSERT_OK( TYEnqueueBuffer(pData->hDevice, frame->userBuffer, frame->bufferSize) );
    
}

// xyz点 和 彩色图像 合成 xyzrgba点云======
void TYcamera::genPointCloudXYZRGBFromVec3f(cv::Mat& data_mat, cv::Mat& color_mat,
                                             pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
        float* data  = (float*)data_mat.data;
        unsigned char* color = (unsigned char*)color_mat.data;
        cloud->resize(data_mat.rows * data_mat.cols); // 640×480 = 307200 个点
	cloud->width    =  data_mat.cols;// 列数，图像宽度=== 
	cloud->height   =  data_mat.rows;// 有序点云
        cloud->is_dense = false;// 非稠密点云，会有不好的点,可能包含inf/NaN 这样的值
                                // 结构点云，有序点云 organized and not dense point clouds 
        
        for(unsigned int i = 0; i < cloud->points.size(); i++)
         {
            //cloud->push_back(pcl::PointXYZRGB(data[i*3+0],  data[i*3+1],  data[i*3+2], 
            //                                  color[i*3+0], color[i*3+1], color[i*3+2] ));
           if(data[i*3+2]<400) // 深度之很小，假的
           {
               cloud->points[i].x = cloud->points[i].y = cloud->points[i].z = std::numeric_limits<float>::quiet_NaN();
           }
           else
           {
            // 传感器过来放大了1000倍
	    cloud->points[i].x = data[i*3+0]/1000;
	    cloud->points[i].y = data[i*3+1]/1000;
	    cloud->points[i].z = data[i*3+2]/1000;
	    cloud->points[i].r = color[i*3+2];
	    cloud->points[i].g = color[i*3+1];
            cloud->points[i].b = color[i*3+0];
           }
            //cloud->points[i].a = 0.5;// 半透明========
        }
}

