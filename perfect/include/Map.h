/**
* This file is part of ORB-SLAM2.
* 地图 管理 关键帧 地图点
*  添加/删除  关键帧/地图点
* 
*/

#ifndef MAP_H
#define MAP_H

#include "MapPoint.h"
#include "KeyFrame.h"
#include <set>

#include <mutex>


#include "Converter.h"
#include "ORBextractor.h"
#include "Frame.h"

// 地图
namespace ORB_SLAM2
{

class MapPoint;
class KeyFrame;

// 载入地图时使用====
class ORBextractor;
class Converter;
class Frame;

class Map
{
public:
    Map();

    void AddKeyFrame(KeyFrame* pKF);// 添加关键帧
    void AddMapPoint(MapPoint* pMP);// 添加地图点
    void EraseMapPoint(MapPoint* pMP);// 删除地图点
    void EraseKeyFrame(KeyFrame* pKF);//删除关键帧
    void SetReferenceMapPoints(const std::vector<MapPoint*> &vpMPs);// 参考地图点
    void InformNewBigChange();// 地图数据 大变化
    int GetLastBigChangeIdx();// 上次地图数据大变化 id

    std::vector<KeyFrame*> GetAllKeyFrames();// 得到所有关键帧
    std::vector<MapPoint*> GetAllMapPoints();// 得到所有地图点
    std::vector<MapPoint*> GetReferenceMapPoints();

    long unsigned int MapPointsInMap();// 地图中地图点的数量
    long unsigned  KeyFramesInMap();// 关键帧数量
    long unsigned int GetMaxKFid();

    void clear();

    vector<KeyFrame*> mvpKeyFrameOrigins;

    std::mutex mMutexMapUpdate;

    // This avoid that two points are created simultaneously in separate threads (id conflict)
    std::mutex mMutexPointCreation;


	bool Save(const string &filename);// 保存地图===
	bool Load(const string &filename, ORBVocabulary &voc);// 载入地图====

protected://类内访问变量

// 地图读写实现函数====
	void _WriteMapPoint(ofstream &f, MapPoint* mp);// 写地图点=====
	void _WriteKeyFrame(ofstream &f, KeyFrame* kf,  
                            map<MapPoint*, unsigned long int>& idx_of_mp);// 写关键帧====

	MapPoint* _ReadMapPoint(ifstream &f);// 读地图点
	KeyFrame* _ReadKeyFrame(ifstream &f, 
                                ORBVocabulary &voc, 
                                std::vector<MapPoint*> amp, 
                                ORBextractor* ex);// 读取关键帧====
	


    std::set<MapPoint*> mspMapPoints;//所有地图点
    std::set<KeyFrame*> mspKeyFrames;//所有关键帧

    std::vector<MapPoint*> mvpReferenceMapPoints;//所有参考地图点 上一帧

    long unsigned int mnMaxKFid;

    // Index related to a big change in the map (loop closure, global BA)
    int mnBigChangeIdx;

    std::mutex mMutexMap;// 线程 互斥锁
    // 其中std::mutex就是lock、unlock。std::lock_guard与std::mutex配合使用，把锁放到lock_guard中时，
    // mutex自动上锁，lock_guard析构时，同时把mutex解锁。mutex又称互斥量。

    Converter convert;
};

} //namespace ORB_SLAM

#endif // MAP_H
