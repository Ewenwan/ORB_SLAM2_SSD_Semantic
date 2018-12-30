/**
* This file is part of ORB-SLAM2.
* 
* 该类负责特征点与特征点之间，地图点与特征点之间通过投影关系、词袋模型或者Sim3位姿匹配。
* 用来辅助完成单目初始化，三角化恢复新的地图点，tracking，relocalization以及loop closing，
* 因此比较重要。
* 
各类之间的匹配   局部匹配  全局匹配等 

何时用投影匹配，何时用DBow2进行匹配？
在Relocalization和LoopClosing中进行匹配的是在很多帧关键帧集合中匹配，
属于Place Recognition，因此需要用DBow，

而 投影匹配 适用于两帧之间，
或者投影范围内（局部地图，前一个关键帧对应地图点）的MapPoints与当前帧之间。
*/


#ifndef ORBMATCHER_H
#define ORBMATCHER_H

#include<vector>
#include<opencv2/core/core.hpp>
#include<opencv2/features2d/features2d.hpp>

#include"MapPoint.h"
#include"KeyFrame.h"
#include"Frame.h"


namespace ORB_SLAM2
{

class ORBmatcher
{    
public:

    ORBmatcher(float nnratio=0.6, bool checkOri=true);

    // Computes the Hamming distance between two ORB descriptors
    // 计算两个 特征点 的 ORB二进制 描述子之间的 汉明字符串匹配距离
    static int DescriptorDistance(const cv::Mat &a, const cv::Mat &b);

    // Search matches between Frame keypoints and projected MapPoints. Returns number of matches
    // Used to track the local map (Tracking)
    // a. 当前帧 的 关键点的描述子 和 投影的 地图点的 描述子之间匹配===========
    // 当前帧 和 局部地图 之间的匹配
    // 在TrackReferenceKeyFrame和TrackWithMotionModel中，仅仅是两帧之间跟踪，会跟丢地图点，
    // 这里通过跟踪局部地图，在当前帧中恢复出一些当前帧的地图点。
    // 其中的阈值th一般根据单目还是双目，或者最近有没有进行过重定位来确定，
    // 代表在投影点的这个平面阈值范围内寻找匹配特征点。
    // 匹配点不仅需要满足对极几何，初始位姿的约束；还需要满足描述子之间距离较小
    // 投影匹配
    int SearchByProjection(Frame &F, const std::vector<MapPoint*> &vpMapPoints, const float th=3);

    // Project MapPoints tracked in last frame into the current frame and search matches.
    // Used to track from previous frame (Tracking)
    // b. 当前帧 和 上一帧 的匹配========================================
    // 匹配上一帧的地图点，即前后两帧匹配，用于计算运动速度  TrackWithMotionModel 
     // 两帧 之间的匹配 容易跟丢
    // 投影匹配
    int SearchByProjection(Frame &CurrentFrame, const Frame &LastFrame, const float th, const bool bMono);

// 上一帧投影下来搜索匹配点，返回 2d-2d匹配点对============================================
    int SearchByProjection(Frame &CurrentFrame, const Frame &LastFrame, 
                           const float th, const bool bMono,
                           vector<cv::Point2f>& points_last,
                           vector<cv::Point2f>& points_current);

    // Project MapPoints seen in KeyFrame into the Frame and search matches.
    // Used in relocalisation (Tracking)
    // c. 在当前帧中匹配所有关键帧中的地图点，用于重定位 Relocalization。==============
    // DBow2 匹配
    int SearchByProjection(Frame &CurrentFrame, KeyFrame* pKF, const std::set<MapPoint*> &sAlreadyFound, const float th, const int ORBdist);

    // Project MapPoints using a Similarity Transformation and search matches.
    // Used in loop detection (Loop Closing)
    // 在当前关键帧中匹配所有关键帧中的地图点，需要计算sim3，用于Loop Closing。
    // DBow2 匹配    
     int SearchByProjection(KeyFrame* pKF, cv::Mat Scw, const std::vector<MapPoint*> &vpPoints, std::vector<MapPoint*> &vpMatched, int th);

    // Search matches between MapPoints in a KeyFrame and ORB in a Frame.
    // Brute force constrained to ORB that belong to the same vocabulary node (at a certain level)
    // Used in Relocalisation and Loop Detection
     // 重定位 和 回环检测 关键帧和帧 /关键帧 之间的匹配
     // 两帧 之间的匹配 容易跟丢
     // 这里形参表示的匹配的主被动关系和SearchByProjection是反的
     
     // a. 在当前帧中匹配关键帧中的地图点，
     // 用于TrackReferenceKeyFrame(和上一个关键帧BOW匹配) 和 Relocalization（和所有关键帧BOW匹配）
    // DBow2 匹配     
    int SearchByBoW(KeyFrame *pKF, Frame &F, std::vector<MapPoint*> &vpMapPointMatches);
    
    // b. 在当前关键帧中匹配所有关键帧中的地图点，用于Loop Closing。
    int SearchByBoW(KeyFrame *pKF1, KeyFrame* pKF2, std::vector<MapPoint*> &vpMatches12);

    // Matching for the Map Initialization (only used in the monocular case)
    // 单目初始化时 的匹配
    int SearchForInitialization(Frame &F1, Frame &F2, std::vector<cv::Point2f> &vbPrevMatched, std::vector<int> &vnMatches12, int windowSize=10);

    // Matching to triangulate new MapPoints. Check Epipolar Constraint.
    // 以及利用三角化，在两个关键帧之间恢复出一些地图点
    int SearchForTriangulation(KeyFrame *pKF1, KeyFrame* pKF2, cv::Mat F12,
                               std::vector<pair<size_t, size_t> > &vMatchedPairs, const bool bOnlyStereo);

    // Search matches between MapPoints seen in KF1 and KF2 transforming by a Sim3 [s12*R12|t12]
    // In the stereo and RGB-D case, s12=1
    
    int SearchBySim3(KeyFrame* pKF1, KeyFrame* pKF2, std::vector<MapPoint *> &vpMatches12, const float &s12, const cv::Mat &R12, const cv::Mat &t12, const float th);

    
  // 4. 两个重载的Fuse函数，用于地图点的融合
    // Project MapPoints into KeyFrame and search for duplicated MapPoints.
    // 地图点能匹配上当前关键帧的地图点，也就是地图点重合了，选择观测数多的地图点替换；
    // 地图点能匹配上当前帧的特征点，但是该特征点还没有生成地图点，则生成新的地图点）。
    int Fuse(KeyFrame* pKF, const vector<MapPoint *> &vpMapPoints, const float th=3.0);

    // Project MapPoints into KeyFrame using a given Sim3 and search for duplicated MapPoints.
    // 　重载的函数是为了减小尺度漂移的影响，需要知道当前关键帧的sim3位姿。
    int Fuse(KeyFrame* pKF, cv::Mat Scw, const std::vector<MapPoint*> &vpPoints, float th, vector<MapPoint *> &vpReplacePoint);

public:

    static const int TH_LOW;
    static const int TH_HIGH;
    static const int HISTO_LENGTH;


protected:

    bool CheckDistEpipolarLine(const cv::KeyPoint &kp1, const cv::KeyPoint &kp2, const cv::Mat &F12, const KeyFrame *pKF);

    float RadiusByViewingCos(const float &viewCos);

// 统计方向直方图最高的三个bin保留，其他范围内的匹配点剔除。
// 另外，若最高的比第二高的高10倍以上，则只保留最高的bin中的匹配点。
// 若最高的比第 三高的高10倍以上，则 保留最高的和第二高bin中的匹配点。
    void ComputeThreeMaxima(std::vector<int>* histo, const int L, int &ind1, int &ind2, int &ind3);

    float mfNNratio;
    bool mbCheckOrientation;
};

}// namespace ORB_SLAM

#endif // ORBMATCHER_H
