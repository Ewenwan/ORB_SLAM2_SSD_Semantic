/**
* This file is part of ORB-SLAM2.
*
*  后端建图和优化线程
* LocalMapping作用是将Tracking中送来的关键帧放在mlNewKeyFrame列表中；
* 处理新关键帧，地图点检查剔除，生成新地图点，Local BA，关键帧剔除。
* 主要工作在于维护局部地图，也就是SLAM中的Mapping。
* 
* Tracking线程 只是判断当前帧是否需要加入关键帧，并没有真的加入地图，
* 因为Tracking线程的主要功能是局部定位，
* 
* 而处理地图中的关键帧，地图点，包括如何加入，
* 如何删除的工作是在LocalMapping线程完成的
* 
*/

#ifndef LOCALMAPPING_H
#define LOCALMAPPING_H

#include "KeyFrame.h"
#include "Map.h"
#include "LoopClosing.h"
#include "Tracking.h"
#include "KeyFrameDatabase.h"

#include <mutex>


namespace ORB_SLAM2
{

class Tracking;
class LoopClosing;
class Map;

class LocalMapping
{
public:
    LocalMapping(Map* pMap, const float bMonocular);

    void SetLoopCloser(LoopClosing* pLoopCloser);

    void SetTracker(Tracking* pTracker);

    // Main function
    void Run();

    void InsertKeyFrame(KeyFrame* pKF);// 图中插入关键帧====

    // Thread Synch
    void RequestStop();
    void RequestReset();
    bool Stop();
    void Release();
    bool isStopped();
    bool stopRequested();
    bool AcceptKeyFrames();
    void SetAcceptKeyFrames(bool flag);
    bool SetNotStop(bool flag);

    void InterruptBA();

    void RequestFinish();
    bool isFinished();

    int KeyframesInQueue(){
        unique_lock<std::mutex> lock(mMutexNewKFs);
        return mlNewKeyFrames.size();
    }

protected:

    bool CheckNewKeyFrames();
    
 // 处理新关键帧：ProcessNewKeyFrame()   
/*a. 根据词典 计算当前关键帧Bow，便于后面三角化恢复新地图点；
b. 将TrackLocalMap中跟踪局部地图匹配上的地图点绑定到当前关键帧
    （在Tracking线程中只是通过匹配进行局部地图跟踪，优化当前关键帧姿态），
    也就是在graph 中加入当前关键帧作为node，并更新edge。
    
    而CreateNewMapPoint()中则通过当前关键帧，在局部地图中添加与新的地图点；

c. 更新加入当前关键帧之后关键帧之间的连接关系，包括更新Covisibility图和Essential图
 （最小生成树spanning tree，共视关系好的边subset of edges from covisibility graph 
   with high covisibility (θ=100)， 闭环边）。
*/
    void ProcessNewKeyFrame();
    
 // 而CreateNewMapPoint()中则通过当前关键帧，在局部地图中添加新的地图点；   
    void CreateNewMapPoints();
    
// 对于 ProcessNewKeyFrame 和 CreateNewMapPoints 中最近添加的MapPoints进行检查剔除
    void MapPointCulling();
    
    void SearchInNeighbors();

    void KeyFrameCulling();// 关键帧融合=======

    cv::Mat ComputeF12(KeyFrame* &pKF1, KeyFrame* &pKF2);

    cv::Mat SkewSymmetricMatrix(const cv::Mat &v);

    bool mbMonocular;

    void ResetIfRequested();
    bool mbResetRequested;
    std::mutex mMutexReset;

    bool CheckFinish();
    void SetFinish();
    bool mbFinishRequested;
    bool mbFinished;
    std::mutex mMutexFinish;

    Map* mpMap;

    LoopClosing* mpLoopCloser;
    Tracking* mpTracker;

    std::list<KeyFrame*> mlNewKeyFrames;// tracker 加入的新关键帧 链表

    KeyFrame* mpCurrentKeyFrame;

    std::list<MapPoint*> mlpRecentAddedMapPoints;// 最近添加的地图点 链表

    std::mutex mMutexNewKFs;

    bool mbAbortBA;

    bool mbStopped;
    bool mbStopRequested;
    bool mbNotStop;
    std::mutex mMutexStop;

    bool mbAcceptKeyFrames;
    std::mutex mMutexAccept;
};

} //namespace ORB_SLAM

#endif // LOCALMAPPING_H
