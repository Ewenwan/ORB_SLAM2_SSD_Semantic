/**
* This file is part of ORB-SLAM2.
* 地图的可视化
*/

#ifndef MAPDRAWER_H
#define MAPDRAWER_H

#include"Map.h"
#include"MapPoint.h"
#include"KeyFrame.h"
#include"Converter.h"
#include<pangolin/pangolin.h>

#include<mutex>

//===================new======

// octomap
#include <octomap/ColorOcTree.h>
#include <octomap/Pointcloud.h>
#include <octomap/octomap.h>
// pcl
#include <pcl/io/pcd_io.h>// 读写
#include <pcl/common/transforms.h>// 点云坐标变换
#include <pcl/point_types.h>      // 点类型
#include <pcl/filters/voxel_grid.h>// 体素格滤波
#include <pcl/filters/passthrough.h>//  直通滤波
#include <pcl/sample_consensus/method_types.h>// 采样一致性，采样方法
#include <pcl/sample_consensus/model_types.h>// 模型
#include <pcl/segmentation/sac_segmentation.h>// 采样一致性分割
#include <pcl/filters/extract_indices.h>// 提取点晕索引


// =====================
#include "Detector.h"// 2d目标检测结果
#include "Merge2d3d.h"// 融合2d和点云信息 到3d目标数据库
#include "MergeSG.h"// 融合2d和点云信息 到3d目标数据库

class Merge2d3d;

class MergeSG;

namespace ORB_SLAM2
{

class MapDrawer
{
public:
    MapDrawer(Map* pMap, const string &strSettingPath);

    ~MapDrawer();

    Map* mpMap;

// 显示点======普通点黑色===参考地图点红色===颜色可修改====
    void DrawMapPoints();
// 显示关键帧================蓝色====关键帧连线偏绿色
    void DrawKeyFrames(const bool bDrawKF, const bool bDrawGraph);
// 显示当前帧 相机位姿========绿色==
    void DrawCurrentCamera(pangolin::OpenGlMatrix &Twc);
// 设置当前帧 相机姿==========
    void SetCurrentCameraPose(const cv::Mat &Tcw);
    void SetReferenceKeyFrame(KeyFrame *pKF);// 没看到
// 获取当前相机位姿，返回 OpenGlMatrix 类型=====
    void GetCurrentOpenGLCameraMatrix(pangolin::OpenGlMatrix &M);
// 获取相机位姿====返回 cv::mat 类型======new=======
bool GetCurrentCameraPos(cv::Mat &Rcw, cv::Mat  &Ow);
void DrawGrid();// 通过画线 显示格子
// 胖果林显示 pcl点云点===

// 显示当前帧 点云===
void DrawObs(void);

// 显示稠密 octomap图====
void DrawOctoMap();

// 显示数据库 中的物体，3d框
void DrawObject();

// 保存 octomap地图====
void SaveOctoMap(const char*filename);

void RegisterObs(pcl::PointCloud<pcl::PointXYZRGB> mvObs);


// 融合2d和点云信息 到3d目标数据库 
// Merge2d3d* mpMerge2d3d;  // 需要定义为指针否则 编译器报错
   MergeSG* mpMerge2d3d;

protected:
     // 生成当前帧的点云，简单滤波 并 分离地面 和 非地面
     void GeneratePointCloud(KeyFrame* kf, 
                             pcl::PointCloud<pcl::PointXYZRGB>& ground,                  
                             pcl::PointCloud<pcl::PointXYZRGB>& nonground);

     // 生成当前帧的点云，简单滤波 并 分离地面 和 非地面
     void GeneratePointCloud(KeyFrame* kf, 
                             pcl::PointCloud<pcl::PointXYZRGB>& ground,                  
                             pcl::PointCloud<pcl::PointXYZRGB>& nonground,
                             std::vector<Object>& objects);

     // 总octomao地图中插入，新生成的点云===
     void InsertScan(octomap::point3d sensorOrigin, 
                     pcl::PointCloud<pcl::PointXYZRGB>& ground, 
                     pcl::PointCloud<pcl::PointXYZRGB>& nonground);
      // 散斑??
     bool isSpeckleNode(const octomap::OcTreeKey &nKey);
     
     // 更新 octomap====
     void UpdateOctomap(vector<KeyFrame*> vKFs);
     
     // 有高度生成颜色==
     void heightMapColor(double h, double& r, double &g, double& b);
     
     // 载入octomap====
     void LoadOctoMap();

private:

// 显示 大小尺寸参数
    float mKeyFrameSize;
    float mKeyFrameLineWidth;
    float mGraphLineWidth;
    float mPointSize;
    float mCameraSize;
    float mCameraLineWidth;

    cv::Mat mCameraPose;

    std::mutex mMutexCamera;

private:

   // int last_obj_size;

    pcl::PointCloud<pcl::PointXYZRGB> observation; // 当前帧点云===

    uint16_t  lastKeyframeSize =0;// 上一次关键帧数量大小
    
    octomap::ColorOcTree *m_octree;
    octomap::KeyRay m_keyRay; // temp storage for casting
    octomap::OcTreeKey m_updateBBXMin;
    octomap::OcTreeKey m_updateBBXMax;

    double m_maxRange;
    bool m_useHeightMap;

    double m_colorFactor;
    double m_res;// octomap 图精度
    unsigned m_treeDepth;
    unsigned m_maxTreeDepth;
    bool bIsLocalization;


    octomap::OcTreeKey m_paddedMinKey, m_paddedMaxKey;
    inline static void updateMinKey(const octomap::OcTreeKey&in, 
                                    octomap::OcTreeKey& min)
    {
        for(unsigned int i=0; i<3; i++)
            min[i] = std::min(in[i], min[i]);
    }
    inline static void updateMaxKey(const octomap::OcTreeKey&in, 
                                    octomap::OcTreeKey& max)
    {
        for(unsigned int i=0; i<3; i++)
            max[i] = std::max(in[i], max[i]);
    }

};

} //namespace ORB_SLAM

#endif // MAPDRAWER_H
