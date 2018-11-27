/**
* This file is part of ORB-SLAM2.
* 地图 管理 关键帧 地图点
*  添加/删除  关键帧/地图点
* 
* 
*      (1)、std::mutex：该类表示普通的互斥锁, 不能递归使用。

	(2)、std::timed_mutex：该类表示定时互斥锁，不能递归使用。
	        std::time_mutex比std::mutex多了两个成员函数：

	A、try_lock_for()：函数参数表示一个时间范围，在这一段时间范围之内线程如果没有获得锁则保持阻塞；
	      如果在此期间其他线程释放了锁，则该线程可获得该互斥锁；
	      如果超时(指定时间范围内没有获得锁)，则函数调用返回false。

	B、try_lock_until()：函数参数表示一个时刻，在这一时刻之前线程如果没有获得锁则保持阻塞；
	      如果在此时刻前其他线程释放了锁，则该线程可获得该互斥锁；如果超过指定时刻没有获得锁，
	      则函数调用返回false。

	(3)、std::recursive_mutex：该类表示递归互斥锁。递归互斥锁可以被同一个线程多次加锁，
		以获得对互斥锁对象的多层所有权。例如，同一个线程多个函数访问临界区时都可以各自加锁，
		执行后各自解锁。std::recursive_mutex释放互斥量时需要调用与该锁层次深度相同次数的unlock()，
		即lock()次数和unlock()次数相同。可见，线程申请递归互斥锁时，
		如果该递归互斥锁已经被当前调用线程锁住，则不会产生死锁。
		此外，std::recursive_mutex的功能与 std::mutex大致相同。

	(4)、std::recursive_timed_mutex：带定时的递归互斥锁。

	互斥类的最重要成员函数是lock()和unlock()。在进入临界区时，执行lock()加锁操作，
	如果这时已经被其它线程锁住，则当前线程在此排队等待。退出临界区时，执行unlock()解锁操作。
	
	
* 
* 
*/

#ifndef MAP_H
#define MAP_H

#include "MapPoint.h"
#include "KeyFrame.h"
#include <set>

#include <mutex>



namespace ORB_SLAM2
{

class MapPoint;
class KeyFrame;

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

protected://类内访问变量
    std::set<MapPoint*> mspMapPoints;//所有地图点
    std::set<KeyFrame*> mspKeyFrames;//所有关键帧

    std::vector<MapPoint*> mvpReferenceMapPoints;//所有参考地图点

    long unsigned int mnMaxKFid;

    // Index related to a big change in the map (loop closure, global BA)
    int mnBigChangeIdx;

    std::mutex mMutexMap;// 线程 互斥锁
    // 其中std::mutex就是lock、unlock。std::lock_guard与std::mutex配合使用，把锁放到lock_guard中时，
    // mutex自动上锁，lock_guard析构时，同时把mutex解锁。mutex又称互斥量。
    
};

} //namespace ORB_SLAM

#endif // MAP_H
