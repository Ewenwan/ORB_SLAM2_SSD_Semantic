/**
* This file is part of ORB-SLAM2.
* 关键帧显示
*/

#ifndef FRAMEDRAWER_H
#define FRAMEDRAWER_H

#include "Tracking.h"
#include "MapPoint.h"
#include "Map.h"

#include<opencv2/core/core.hpp>
#include<opencv2/features2d/features2d.hpp>

#include<mutex>
namespace ORB_SLAM2
{

class Tracking;
class Viewer;
class MapDrawer;

class FrameDrawer
{
public:
    FrameDrawer(Map* pMap);// 类构造函数
    FrameDrawer(Map* pMap, MapDrawer* pMapDrawer, const string &strSettingPath);

    // Update info from the last processed frame.
    void Update(Tracking *pTracker);// 从 track 对象更新数据到 本类内

    // Draw last processed frame.
    cv::Mat DrawFrame();// 显示关键帧
    void generatePC(void); // 生成当前帧的点云====

protected:

    void DrawTextInfo(cv::Mat &im, int nState, cv::Mat &imText);//显示 文本信息

    void FillImage(cv::Mat &im, const cv::Mat &mask, cv::Scalar color);

    // Info of the frame to be drawn
    cv::Mat mIm;// 灰度图像
    cv::Mat mDynMask;// 动态点mask
    cv::Mat mImDep;// 深度图
    cv::Mat mImRGB;// 深度图
    cv::Mat mK;// 相机内参数
    cv::Mat mTcw;// 相机位姿

    int N;
    vector<cv::KeyPoint> mvCurrentKeys;// 当前帧关键点
    vector<bool> mvbMap, mvbVO;           // 是否有对应的地图点
    bool mbOnlyTracking;                              //  模式
    int mnTracked, mnTrackedVO;
    vector<cv::KeyPoint> mvIniKeys;// 初始化参考帧关键点
    vector<int> mvIniMatches;// 匹配点
    int mState;//tracker状态

    Map* mpMap;// 地图
    MapDrawer* mpMapDrawer;// 地图显示绘制===

    std::mutex mMutex;// 访问 tracker 类 的数据线程锁
};

} //namespace ORB_SLAM

#endif // FRAMEDRAWER_H
