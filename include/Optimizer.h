/**
* This file is part of ORB-SLAM2.
* 全局/局部 优化 使用G2O图优化
*/

#ifndef OPTIMIZER_H
#define OPTIMIZER_H

#include "Map.h"
#include "MapPoint.h"
#include "KeyFrame.h"
#include "LoopClosing.h"
#include "Frame.h"

#include "Thirdparty/g2o/g2o/types/types_seven_dof_expmap.h"
// 定义好的顶点类型  7维度 优化变量  例如 相机 位姿 + 深度信息


namespace ORB_SLAM2
{

class LoopClosing;

class Optimizer
{
public:
  // BA 最小化重投影误差 优化
    void static BundleAdjustment(const std::vector<KeyFrame*> &vpKF, const std::vector<MapPoint*> &vpMP,
                                 int nIterations = 5, bool *pbStopFlag=NULL, const unsigned long nLoopKF=0,
                                 const bool bRobust = true);
    // 全局优化  地图  迭代次数
    void static GlobalBundleAdjustemnt(Map* pMap, int nIterations=5, bool *pbStopFlag=NULL,
                                       const unsigned long nLoopKF=0, const bool bRobust = true);
    // 局部优化
    void static LocalBundleAdjustment(KeyFrame* pKF, bool *pbStopFlag, Map *pMap);
    // 位姿 优化
    int static PoseOptimization(Frame* pFrame);

    // 优化图 3d位姿 + 3d平移  双目/深度 优化6维
    // 单目 在优化 3d位姿 + 3d平移上在 优化一维 深度信息
    // if bFixScale is true, 6DoF optimization (stereo,rgbd), 7DoF otherwise (mono)
    // 闭环检测到后的优化
    void static OptimizeEssentialGraph(Map* pMap, KeyFrame* pLoopKF, KeyFrame* pCurKF,
                                       const LoopClosing::KeyFrameAndPose &NonCorrectedSim3,
                                       const LoopClosing::KeyFrameAndPose &CorrectedSim3,
                                       const map<KeyFrame *, set<KeyFrame *> > &LoopConnections,
                                       const bool &bFixScale);

    // if bFixScale is true, optimize SE3 (stereo,rgbd), Sim3 otherwise (mono)
    static int OptimizeSim3(KeyFrame* pKF1, KeyFrame* pKF2, std::vector<MapPoint *> &vpMatches1,
                            g2o::Sim3 &g2oS12, const float th2, const bool bFixScale);
};

} //namespace ORB_SLAM

#endif // OPTIMIZER_H
