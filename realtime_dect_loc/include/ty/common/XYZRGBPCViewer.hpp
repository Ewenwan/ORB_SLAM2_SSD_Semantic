#ifndef PERCIPIO_SAMPLE_COMMON_RGBPOINT_CLOUD_VIEWER_HPP_
#define PERCIPIO_SAMPLE_COMMON_RGBPOINT_CLOUD_VIEWER_HPP_

#include <opencv2/opencv.hpp>
#include <string>

#include <pcl/point_types.h> // 点
#include <pcl/point_cloud.h> // 点云

class XYZRGBPCViewerImpl;

class XYZRGBPCViewer {
public:
    XYZRGBPCViewer();
    ~XYZRGBPCViewer();

    void show(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ptr, const std::string &windowName);
    bool isStopped(const std::string &windowName);

private:
    XYZRGBPCViewerImpl* impl;// 具体实现类
};

#endif
