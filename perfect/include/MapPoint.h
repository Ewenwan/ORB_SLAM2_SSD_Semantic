/**
* This file is part of ORB-SLAM2.
* 地图点  普通帧上的地图点
*               关键帧上的地图点
* 创建 地图点    观测帧集合   最优的描述子
* 坏点检测 被观测次数 小于2  删除地图点
* 地图点距离 参考帧相机中心的 相对 坐标
* 地图点 相对参考帧 相机中心  在 图像金字塔上 每层 各个尺度空间的距离
* 
*/

#ifndef MAPPOINT_H
#define MAPPOINT_H

#include"KeyFrame.h"
#include"Frame.h"
#include"Map.h"

#include<opencv2/core/core.hpp>
#include<mutex>

namespace ORB_SLAM2
{

class KeyFrame;// 关键帧
class Map;// 地图
class Frame;//普通帧


class MapPoint
{
public:
  
  // 构造函数（地图点3D坐标及其参考帧）

    // 载入地图时 创建地图点=============
    MapPoint(const cv::Mat &Pos, int FirstKFid, int FirstFrame, Map* pMap);

	// 创建关键帧地图点   世界坐标点     所属关键帧    所属地图
	// 参考帧是关键帧，该地图点将与许多帧关键帧对应，建立关键帧之间的共视关系
    MapPoint(const cv::Mat &Pos, KeyFrame* pRefKF, Map* pMap);
    
   // 参考帧是普通帧，该地图点只与当前普通帧的特征点对应 
   // 创建普通帧地图点    世界坐标点       所属地图   所属普通 帧      帧id
    MapPoint(const cv::Mat &Pos,  Map* pMap, Frame* pFrame, const int &idxF);

    
    void SetWorldPos(const cv::Mat &Pos);//设置点
    cv::Mat GetWorldPos();// 获取点
    cv::Mat GetNormal();// 点 相对于 相机光心坐标 
    KeyFrame* GetReferenceKeyFrame();// 获取参考关键帧

// ====================================new=================================
    void SetReferenceKeyFrame(KeyFrame* pRefKF);//======设置 地图点参考帧==


    std::map<KeyFrame*,size_t> GetObservations();// 观测到该地图点的帧集 + 观测到的次数
    int Observations();

    void AddObservation(KeyFrame* pKF,size_t idx);// 该地图点 添加一个观测帧记录
    void EraseObservation(KeyFrame* pKF);// 删除观测帧记录

    int GetIndexInKeyFrame(KeyFrame* pKF);
    bool IsInKeyFrame(KeyFrame* pKF);

    void SetBadFlag();
    bool isBad();

    void Replace(MapPoint* pMP);    
    MapPoint* GetReplaced();

    void IncreaseVisible(int n=1);
    void IncreaseFound(int n=1);
    
    //GetFoundRatio低表示该地图点在很多关键帧的视野范围内，但是没有匹配上很多特征点。
    float GetFoundRatio();
    inline int GetFound(){
        return mnFound;
    }

    void ComputeDistinctiveDescriptors();
// 1 在观测到该地图点的多个特征点中（对应多个关键帧），
     // 挑选出区分度最高的描述子，作为这个地图点的描述子；
    cv::Mat GetDescriptor();// 地图点对应的描述子
    
// 2. 计算地图点平均观测方向和深度 （在所有观测帧上）
    void UpdateNormalAndDepth();

    float GetMinDistanceInvariance();
    float GetMaxDistanceInvariance();
    int PredictScale(const float &currentDist, KeyFrame*pKF);
    int PredictScale(const float &currentDist, Frame*pF);

//=====new 直接 传递  pKF->mfLogScaleFactor 参数 =======
    int PredictScale(const float &currentDist, const float &logScaleFactor);

public:
    long unsigned int mnId;
    static long unsigned int nNextId;
    long int mnFirstKFid;
    long int mnFirstFrame;
    int nObs;//    观测到该地图点的相机数  关键帧数量

    // Variables used by the tracking
    // 跟踪 到的 参数 
    float mTrackProjX;//匹配点 x偏移
    float mTrackProjY;//匹配点 y偏移
    float mTrackProjXR;
    bool mbTrackInView;
    int mnTrackScaleLevel;
    float mTrackViewCos;
    long unsigned int mnTrackReferenceForFrame;
    long unsigned int mnLastFrameSeen;

    // Variables used by local mapping
    long unsigned int mnBALocalForKF;
    long unsigned int mnFuseCandidateForKF;// 地图点 和 哪一帧的 地图点有融合 的标记

    // Variables used by loop closing
    long unsigned int mnLoopPointForKF;// 闭环检测标志
    long unsigned int mnCorrectedByKF;
    long unsigned int mnCorrectedReference;    
    cv::Mat mPosGBA;
    long unsigned int mnBAGlobalForKF;


    static std::mutex mGlobalMutex;

protected:    

     // Position in absolute coordinates
     cv::Mat mWorldPos;

     // Keyframes observing the point and associated index in keyframe
     std::map<KeyFrame*,size_t> mObservations;//  能够观测到该点的 观测关键帧 

     // Mean viewing direction
     cv::Mat mNormalVector;

     // Best descriptor to fast matching 地图点 特征描述子
     cv::Mat mDescriptor;

     // Reference KeyFrame
     KeyFrame* mpRefKF;// 参考关键帧

     // Tracking counters
     int mnVisible;
     int mnFound;

     // Bad flag (we do not currently erase MapPoint from memory)
     // 坏点标志  被观测到的次数  小于2
     bool mbBad;
     MapPoint* mpReplaced;//坏点的 代替 点?

     // Scale invariance distances
     float mfMinDistance;
     float mfMaxDistance;

     Map* mpMap;

     std::mutex mMutexPos;
     std::mutex mMutexFeatures;
};

} //namespace ORB_SLAM

#endif // MAPPOINT_H
