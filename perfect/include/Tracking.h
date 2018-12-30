/**
* This file is part of ORB-SLAM2.
* 前端跟踪线程实现================
* 
*/


#ifndef TRACKING_H
#define TRACKING_H

#include<opencv2/core/core.hpp>// opencv
#include<opencv2/features2d/features2d.hpp>// orb特征检测

#include"Viewer.h"           // 可视化类
#include"FrameDrawer.h"      // 显示帧类 
#include"Map.h"              // 地图类
#include"LocalMapping.h"     // 局部建图类
#include"LoopClosing.h"      // 闭环检测类
#include"Frame.h"            // 帧类
#include"ORBVocabulary.h"    // ORB字典类 
#include"KeyFrameDatabase.h" // 关键帧数据库类
#include"ORBextractor.h"     // orb特征提取类
#include"Initializer.h"      // 初始化类
#include"MapDrawer.h"        // 显示地图类
#include"System.h"           // 系统类头文件

#include <mutex>

// for pointcloud mapping and viewing
#include "pointcloudmapping.h"// 新添加  点云建图类========

#include "Geometry.h" //  几何学 几何模型修正 深度数据关键点区域增长更新===
#include "Flow.h"     //  光流计算===

#include "RunDetect.h" // 目标检测运行线程====
class RunDetect;

class PointCloudMapping;// 声明 点云可视化类
// 没有放在 namespace ORB_SLAM2 内 因为 PointCloudMapping类没有声明在 ORB_SLAM2 明明空间内

namespace ORB_SLAM2
{

class Viewer;        // 可视化类
class FrameDrawer;   // 显示帧类 
class Map;           // 地图类
class LocalMapping;  // 局部建图类
class LoopClosing;   // 闭环检测类
class System;        // 系统类头文件


class Tracking
{  

public:

// 跟踪类初始化=================
    Tracking(System* pSys, ORBVocabulary* pVoc, 
             FrameDrawer* pFrameDrawer, 
             MapDrawer* pMapDrawer, Map* pMap, 
             //shared_ptr<PointCloudMapping> pPointCloud,// 新添加=====
             KeyFrameDatabase* pKFDB, const string &strSettingPath, const int sensor);
    // 带有点云 建图====
    Tracking(System* pSys, ORBVocabulary* pVoc, 
             FrameDrawer* pFrameDrawer, 
             MapDrawer* pMapDrawer, Map* pMap, 
             shared_ptr<PointCloudMapping> pPointCloud,// 新添加=====
             KeyFrameDatabase* pKFDB, const string &strSettingPath, const int sensor);
    // 带有 目标检测 线程===
    Tracking(System* pSys, ORBVocabulary* pVoc, 
             FrameDrawer* pFrameDrawer, 
             MapDrawer* pMapDrawer, Map* pMap, 
             KeyFrameDatabase* pKFDB, const string &strSettingPath, const int sensor,
             shared_ptr<RunDetect> pRunDetect);

// 双目/RGBD/单目相机 跟踪函数===========
    cv::Mat GrabImageStereo(const cv::Mat &imRectLeft,  // 左图
                            const cv::Mat &imRectRight, // 右图
                            const double &timestamp);   // 时间戳

    cv::Mat GrabImageRGBD(const cv::Mat &imRGB,         // RGB图
                          const cv::Mat &imD,           // 深度图
                          const double &timestamp);     // 时间戳

    cv::Mat GrabImageMonocular(const cv::Mat &im,       // RGB图
                               const double &timestamp);// 时间戳

// 设置 其他类指针============共内部使用============
    void SetLocalMapper(LocalMapping* pLocalMapper);// 设置 局部建图类 对象指针
    void SetLoopClosing(LoopClosing* pLoopClosing); // 设置 闭环检测类 对象指针
    void SetViewer(Viewer* pViewer);                // 设置 可视化类   对象指针

    // Load new settings
    // The focal lenght should be similar or scale prediction will fail when projecting points
    // TODO: Modify MapPoint::PredictScale to take into account focal lenght
    void ChangeCalibration(const string &strSettingPath);

    // Use this function if you have deactivated local mapping and you only want to localize the camera.
    void InformOnlyTracking(const bool &flag);// 仅仅定位模式，不建图


public:

    // Tracking states   跟踪任务状态 枚举变量======
    enum eTrackingState
    {
        SYSTEM_NOT_READY=-1,// 系统未准备好
        NO_IMAGES_YET=0,    // 没有图像传入
        NOT_INITIALIZED=1,  // 未初始化
        OK=2,               // 跟踪状态正常
        LOST=3              // 跟踪丢失，触发重定位
    };

    eTrackingState mState;  // 当前跟踪状态
    eTrackingState mLastProcessedState;// 上次跟踪状态

    // Input sensor
    int mSensor;// 传感器类型

    // Current Frame
    Frame mCurrentFrame;// 当前帧 
    cv::Mat mImGray;    // 灰度图
    cv::Mat mimMask;    // 动态点mask
    float   mflowThreshold;// 光流判断动静点阈值====
//=========================================================================
    cv::Mat mImDepth; // adding mImDepth member to realize pointcloud view
                      // 添加一个深度图 作为 类的 成员变量，用来形成点云
    
    cv::Mat mImRGB;   // 彩色图====

// 单目相机初始化相关 变量========================
    // Initialization Variables (Monocular)
    std::vector<int> mvIniLastMatches; // 匹配点
    std::vector<int> mvIniMatches;
    std::vector<cv::Point2f> mvbPrevMatched;
    std::vector<cv::Point3f> mvIniP3D;// 生成的3D点
    Frame mInitialFrame;

// 链表变量=============================
    // Lists used to recover the full camera trajectory at the end of the execution.
    // Basically we store the reference keyframe for each frame and its relative transformation
    list<cv::Mat> mlRelativeFramePoses;// 关键帧位姿
    list<KeyFrame*> mlpReferences;     // 关键帧 引用
    list<double> mlFrameTimes;         // 关键帧 时间戳
    list<bool> mlbLost;                // 关键帧 状态 丢是否

    // True if local mapping is deactivated and we are performing only localization
    bool mbOnlyTracking;// 仅定位模式   定位+建图

    void Reset();       // 重置=====

protected:
    // Main tracking function. It is independent of the input sensor.
    void Track();// 跟踪接口，根据相机类型，跳转到不同相机的 跟踪函数


    // Berta: LightTrack=============================new=======================
void LightTrack();

bool TrackHomo(cv::Mat& homo);// 两帧之前得到单应变换矩阵====

    void StereoInitialization();     // 双目/深度 相机 地图初始化
    void MonocularInitialization();  // 单目地图初始化 初始化获得 初始位姿
    void CreateInitialMapMonocular();// 最小化重投影 优化位姿 

    void CheckReplacedInLastFrame(); // 上一帧地图点 是否有替换点 有替换点的则进行更新
// Local Mapping 线程可能会将关键帧中某些 MapPoints 进行替换，
// 由于 tracking 中需要用到 mLastFrame，这里检查并更新上一帧中被替换的 MapPoints

    void UpdateLastFrame(); // 丰富上一帧的地图点（临时）=======
          // 这些MapPoints在TrackWithMotionModel的UpdateLastFrame函数里生成（仅双目和rgbd）
          // 生成点时 会记录 在 临时点集中 mlpTemporalPoints  链表结构 
          // 这里生成的仅仅是为了提高双目或rgbd摄像头的帧间跟踪效果，用完以后就扔了，没有添加到地图中

    bool TrackReferenceKeyFrame();   // 跟踪关键帧模式  当前帧和 最近一个关键帧 做匹配
    bool TrackWithMotionModel();     // 跟踪上一帧 运动跟踪模式   当前帧和上一帧 做匹配



    // Berta: LightTrackWithMotionModel=================new============
    bool LightTrackWithMotionModel(bool &bVO);

    bool Relocalization();           // 重定位  当前帧 和 所有关键帧 做匹配

    bool Relocalization(bool save_change);// 加入 是否 记录 重定位记录

    void UpdateLocalMap();           // 跟踪局部地图前 更新 局部地图：更新局部地图点、局部关键帧
    void UpdateLocalPoints();             // 更新 局部地图 点
    void UpdateLocalKeyFrames();          // 更新局部关键帧

    void SearchLocalPoints();             // 跟踪局部地图前 搜索局部地图点 
                                          // 局部地图点 搜寻和当前帧 关键点描述子 的匹配 
                                          // 有匹配的加入到 当前帧 特征点对应的地图点中
    bool TrackLocalMap();            // 跟踪局部地图

    bool NeedNewKeyFrame();          // 跟踪局部地图之后 判断是否需要新建关键帧
    void CreateNewKeyFrame();        // 创建关键帧

    // In case of performing only localization, this flag is true when there are no matches to
    // points in the map. Still tracking will continue if there are enough matches with temporal points.
    // In that case we are doing visual odometry. The system will try to do relocalization to recover
    // "zero-drift" localization to the map.
    bool mbVO;
	// mbVO 是 mbOnlyTracking 为true时的才有的一个变量
	// mbVO 为0表示此帧匹配了很多的MapPoints，跟踪很正常，
	// mbVO 为1表明此帧匹配了很少的MapPoints，少于10个，要跪的节奏 

    //Other Thread Pointers
    LocalMapping* mpLocalMapper;// 局部建图 
    LoopClosing* mpLoopClosing; // 回环检测 

    //ORB
    ORBextractor* mpORBextractorLeft, *mpORBextractorRight; // 左右图像 ORB特征提取器
    ORBextractor* mpIniORBextractor;// 单目第一帧提取器 关键点数量 增加1倍

    //BoW
    ORBVocabulary* mpORBVocabulary; // ORB特征字典
    KeyFrameDatabase* mpKeyFrameDB; // 关键帧数据库

    // Initalization (only for monocular)
    Initializer* mpInitializer;     // 单目初始化器，主要是为了生成 初始的地图点

    //Local Map  局部地图
    KeyFrame* mpReferenceKF;// 参考关键帧

    std::vector<KeyFrame*> mvpLocalKeyFrames;// 局部 关键帧
    std::vector<MapPoint*> mvpLocalMapPoints;// 局部地图点
    
    // System
    System* mpSystem;      // 系统对象
    
    //Drawers 可视化======================================
    Viewer* mpViewer;// 可视化器======
    FrameDrawer* mpFrameDrawer;// 显示帧
    MapDrawer* mpMapDrawer;    // 显示地图

    //Map  地图========================
    Map* mpMap;

    //Calibration matrix  相机 校正 矩阵
    cv::Mat mK;        // 内参数
    cv::Mat mDistCoef; // 畸变参数
    float mbf;// 基线 × 视差

    //New KeyFrame rules (according to fps)
    int mMinFrames; // 新建关键帧 标志参数
    int mMaxFrames;

    // Threshold close/far points
    // Points seen as close by the stereo/RGBD sensor are considered reliable
    // and inserted from just one frame. Far points requiere a match in two keyframes.
    float mThDepth;// 双目/rgbd 最大深度值 阈值

    // For RGB-D inputs only. For some datasets (e.g. TUM) the depthmap values are scaled.
    float mDepthMapFactor;// 深度图 值放大因子

    //Current matches in frame
    int mnMatchesInliers; // 当前帧 匹配点数量 

    //Last Frame, KeyFrame and Relocalisation Info
    KeyFrame* mpLastKeyFrame; // 上一个关键帧
    Frame mLastFrame;         // 上一帧
    unsigned int mnLastKeyFrameId;  // 上一个关键帧 id
    unsigned int mnLastRelocFrameId;// 上一个重定位时的帧id

    //Motion Model  相机运动速度======
    cv::Mat mVelocity;// mVelocity = mCurrentFrame.mTcw*LastTwc;//运动速度 为前后两针的 变换矩阵

    //Color order (true RGB, false BGR, ignored if grayscale)
    bool mbRGB;

    list<MapPoint*> mlpTemporalPoints;// 每一帧产生的临时点集合  链表结构


 // for point cloud viewing =======新建==========================
    shared_ptr<PointCloudMapping> mpPointCloudMapping; // 点云地图类 共享指针 类成员变量

    shared_ptr< RunDetect > mpRunDetect;         // 目标检测运行线程

    DynaSLAM::Geometry mGeometry; // ===========================new==========
    FlowSLAM::Flow     mFlow;

};

} //namespace ORB_SLAM

#endif // TRACKING_H
