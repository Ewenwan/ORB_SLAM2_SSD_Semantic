/**
* This file is part of ORB-SLAM2.
* 地图 管理 关键帧 地图点
*  添加/删除  关键帧/地图点
*/

#include "Map.h"

#include<mutex>

#include <sys/stat.h>// 文件状态===

namespace ORB_SLAM2
{
// 初始 最大关键帧id = 0  最大变化id = 0
Map::Map():mnMaxKFid(0),mnBigChangeIdx(0)
{
}

void Map::AddKeyFrame(KeyFrame *pKF)
{
    unique_lock<mutex> lock(mMutexMap);
    mspKeyFrames.insert(pKF);// set 集合 插入关键帧
    if(pKF->mnId > mnMaxKFid)// 关键帧 id
        mnMaxKFid=pKF->mnId;// 最大关键帧 的id
}

void Map::AddMapPoint(MapPoint *pMP)
{
    unique_lock<mutex> lock(mMutexMap);
    mspMapPoints.insert(pMP);// set 集合 插入 地图点
}

void Map::EraseMapPoint(MapPoint *pMP)
{
    unique_lock<mutex> lock(mMutexMap);
    
    // wyw 添加
  //      auto  it = mspMapPoints.find(pMP);// 获取指针
  //      delete *it;//删除
    
    mspMapPoints.erase(pMP);// 删除了指针  内容清除？

    // TODO: This only erase the pointer.
    // Delete the MapPoint

}

void Map::EraseKeyFrame(KeyFrame *pKF)
{
    unique_lock<mutex> lock(mMutexMap);
    
        // wyw 添加
   //     auto  it = mspKeyFrames.find(pKF);// 获取指针
   //     delete *it;//删除
        
    mspKeyFrames.erase(pKF);// 删除了指针  内容清除？

    // TODO: This only erase the pointer.
    // Delete the MapPoint
}

void Map::SetReferenceMapPoints(const vector<MapPoint *> &vpMPs)
{
    unique_lock<mutex> lock(mMutexMap);
    mvpReferenceMapPoints = vpMPs;
}

void Map::InformNewBigChange()
{
    unique_lock<mutex> lock(mMutexMap);
    mnBigChangeIdx++;// id++
}

int Map::GetLastBigChangeIdx()
{
    unique_lock<mutex> lock(mMutexMap);
    return mnBigChangeIdx;
}

// 返回所有关键帧
vector<KeyFrame*> Map::GetAllKeyFrames()
{
    unique_lock<mutex> lock(mMutexMap);
    return vector<KeyFrame*>(mspKeyFrames.begin(),mspKeyFrames.end());
}

// 返回所有地图点
vector<MapPoint*> Map::GetAllMapPoints()
{
    unique_lock<mutex> lock(mMutexMap);
    return vector<MapPoint*>(mspMapPoints.begin(),mspMapPoints.end());
}

// 地图中地图点的数量
long unsigned int Map::MapPointsInMap()
{
    unique_lock<mutex> lock(mMutexMap);
    return mspMapPoints.size();
}
// 地图中 关键帧数量
long unsigned int Map::KeyFramesInMap()
{
    unique_lock<mutex> lock(mMutexMap);
    return mspKeyFrames.size();
}
// 地图中 参考地图点
vector<MapPoint*> Map::GetReferenceMapPoints()
{
    unique_lock<mutex> lock(mMutexMap);
    return mvpReferenceMapPoints;
}
// 最大 id的 关键帧 的 id
long unsigned int Map::GetMaxKFid()
{
    unique_lock<mutex> lock(mMutexMap);
    return mnMaxKFid;
}

void Map::clear()
{
    for(set<MapPoint*>::iterator sit=mspMapPoints.begin(), send=mspMapPoints.end(); sit!=send; sit++)
        delete *sit;// 删除地图点内容

    for(set<KeyFrame*>::iterator sit=mspKeyFrames.begin(), send=mspKeyFrames.end(); sit!=send; sit++)
        delete *sit;// 删除 关键帧

    mspMapPoints.clear();//清楚全部
    mspKeyFrames.clear();
    mnMaxKFid = 0;
    mvpReferenceMapPoints.clear();
    mvpKeyFrameOrigins.clear();
}


// 地图保存与读取==============

//
// Binary version
//
// TODO: frameid vs keyframeid  载入关键帧======
//
KeyFrame* Map::_ReadKeyFrame(ifstream &f, 
                             ORBVocabulary &voc, 
                             std::vector<MapPoint*> amp, 
                             ORBextractor* orb_ext) 
{
  Frame fr;
  fr.mpORBvocabulary = &voc;
  f.read((char*)&fr.mnId, sizeof(fr.mnId));              // 关键帧 ID
  //cerr << " reading keyfrane id " << fr.mnId << endl;
  f.read((char*)&fr.mTimeStamp, sizeof(fr.mTimeStamp));  // 时间戳 timestamp
  cv::Mat Tcw(4,4,CV_32F);                               // 位置 position
  f.read((char*)&Tcw.at<float>(0, 3), sizeof(float));
  f.read((char*)&Tcw.at<float>(1, 3), sizeof(float));
  f.read((char*)&Tcw.at<float>(2, 3), sizeof(float));
  Tcw.at<float>(3,3) = 1.;
  cv::Mat Qcw(1,4, CV_32F);                             // 四元素方位 orientation
  f.read((char*)&Qcw.at<float>(0, 0), sizeof(float));
  f.read((char*)&Qcw.at<float>(0, 1), sizeof(float));
  f.read((char*)&Qcw.at<float>(0, 2), sizeof(float));
  f.read((char*)&Qcw.at<float>(0, 3), sizeof(float));
  convert.RmatOfQuat(Tcw, Qcw);// 四元素转 变换矩阵======
  fr.SetPose(Tcw);
  f.read((char*)&fr.N, sizeof(fr.N));                    // 关键点数量 nb keypoints
  fr.mvKeys.reserve(fr.N);
  fr.mDescriptors.create(fr.N, 32, CV_8UC1);             // 关键点描述子===
  fr.mvpMapPoints = vector<MapPoint*>(fr.N,static_cast<MapPoint*>(NULL));// 关键点对应的地图点====
  for (int i=0; i<fr.N; i++) {
        // 关键点======
	cv::KeyPoint kp;
	f.read((char*)&kp.pt.x,     sizeof(kp.pt.x));
	f.read((char*)&kp.pt.y,     sizeof(kp.pt.y));
	f.read((char*)&kp.size,     sizeof(kp.size));
	f.read((char*)&kp.angle,    sizeof(kp.angle));
	f.read((char*)&kp.response, sizeof(kp.response));
	f.read((char*)&kp.octave,   sizeof(kp.octave));
	fr.mvKeys.push_back(kp);
        // 描述子====
	for (int j=0; j<32; j++)
	  f.read((char*)&fr.mDescriptors.at<unsigned char>(i, j), sizeof(char));
        // 地图点====         
	unsigned long int mpidx;
	f.read((char*)&mpidx,   sizeof(mpidx));
	if (mpidx == ULONG_MAX)	fr.mvpMapPoints[i] = NULL;
	else fr.mvpMapPoints[i] = amp[mpidx];
  }
  // mono only for now  单目==========
  fr.mvuRight = vector<float>(fr.N,-1);// 关键点 右目匹配点横坐标
  fr.mvDepth = vector<float>(fr.N,-1); // 关键点 深度值
  fr.mpORBextractorLeft = orb_ext;     // 关键点提取器
  fr.InitializeScaleLevels();// 金字塔尺度
  fr.UndistortKeyPoints();   // 矫正 需要矫正参数====
  fr.AssignFeaturesToGrid(); // 分配特征点到 图像网格
  fr.ComputeBoW();// 计算 词典表示向量 

  KeyFrame* kf = new KeyFrame(fr, this, NULL);// 创建关键帧====
  kf->mnId = fr.mnId; // bleeee why? do I store that?
  for (int i=0; i<fr.N; i++) 
  {
  	if (fr.mvpMapPoints[i])
        {
  	  fr.mvpMapPoints[i]->AddObservation(kf, i);// 观测关系
	  if (!fr.mvpMapPoints[i]->GetReferenceKeyFrame()) 
                  fr.mvpMapPoints[i]->SetReferenceKeyFrame(kf);// 参考关键帧===
	}
  }

  return kf;
}

// 载入地图点==============================
MapPoint* Map::_ReadMapPoint(ifstream &f)
 {
  long unsigned int id; 
  f.read((char*)&id, sizeof(id));              // 地图点 ID
  cv::Mat wp(3,1, CV_32F);
  f.read((char*)&wp.at<float>(0), sizeof(float));
  f.read((char*)&wp.at<float>(1), sizeof(float));
  f.read((char*)&wp.at<float>(2), sizeof(float));
  long int mnFirstKFid=0, mnFirstFrame=0;
  MapPoint* mp = new MapPoint(wp, mnFirstKFid, mnFirstFrame, this);
  mp->mnId = id;
  return mp;
}

// 载入地图=====================================================
bool Map::Load(const string &filename, ORBVocabulary &voc) 
{
// 需要相机参数==============
 // if (!Camera::initialized) {
//	cerr << "Map: camera is not initialized. Cowardly refusing to load anything" << endl;
//	return false;
 // }
  
  int nFeatures = 2000;
  float scaleFactor = 1.2; // 金字塔尺度====
  int nLevels = 8, fIniThFAST = 20, fMinThFAST = 7;
  ORB_SLAM2::ORBextractor orb_ext = ORB_SLAM2::ORBextractor(nFeatures, scaleFactor, nLevels, fIniThFAST, fMinThFAST);

  cerr << "Map: reading from " << filename << endl;
  ifstream f;
  f.open(filename.c_str());

// 读取地图点============================
  long unsigned int nb_mappoints, max_id=0;
  f.read((char*)&nb_mappoints, sizeof(nb_mappoints));  // 地图点数量====            
  cerr << "reading " << nb_mappoints << " mappoints" << endl; 
  for (unsigned int i=0; i<nb_mappoints; i++) 
  {
	ORB_SLAM2::MapPoint* mp = _ReadMapPoint(f);
	if (mp->mnId>=max_id) max_id=mp->mnId;
	AddMapPoint(mp);// 记录地图点====
  }
  ORB_SLAM2::MapPoint::nNextId = max_id+1; // that is probably wrong if last mappoint is not here :(

  std::vector<MapPoint*> amp = GetAllMapPoints();// 所有的地图点====

// 读取关键帧==================================
  long unsigned int nb_keyframes;
  f.read((char*)&nb_keyframes, sizeof(nb_keyframes));
  cerr << "reading " << nb_keyframes << " keyframe" << endl; 
  vector<KeyFrame*> kf_by_order;
  for (unsigned int i=0; i<nb_keyframes; i++) 
  {
	KeyFrame* kf = _ReadKeyFrame(f, voc, amp, &orb_ext); 
	AddKeyFrame(kf);
	kf_by_order.push_back(kf);
  }
  
// 载入最小生成树  Spanning tree =======================
  map<unsigned long int, KeyFrame*> kf_by_id;// id:关键帧 
  for(auto kf: mspKeyFrames)
	kf_by_id[kf->mnId] = kf;

  for(auto kf: kf_by_order) 
  {
	unsigned long int parent_id;
	f.read((char*)&parent_id, sizeof(parent_id));        // 父亲关键帧id parent id
	if (parent_id != ULONG_MAX)
	  kf->ChangeParent(kf_by_id[parent_id]);
	unsigned long int nb_con;                            // 共视关键帧 number connected keyframe
	f.read((char*)&nb_con, sizeof(nb_con));  
	for (unsigned long int i=0; i<nb_con; i++) 
        {
	  unsigned long int id; int weight;
	  f.read((char*)&id, sizeof(id));                   // 相连的关键帧 connected keyframe
	  f.read((char*)&weight, sizeof(weight));           // 共视权重(共同看到的地图点数量决定)connection weight
	  kf->AddConnection(kf_by_id[id], weight);          // 链接关系===
	}
  }
  // MapPoints descriptors
  for(auto mp: amp) {
	mp->ComputeDistinctiveDescriptors(); // 为地图点 选取最好的 描述子 
	mp->UpdateNormalAndDepth();          // 更新 地图点的 平均观测参数
  }

#if 0
  for(auto mp: mspMapPoints)
	if (!(mp->mnId%100))
	  cerr << "mp " << mp->mnId << " " << mp->Observations() << " " << mp->isBad() << endl;
#endif

#if 0
  for(auto kf: kf_by_order) {
	cerr << "loaded keyframe id " << kf->mnId << " ts " << kf->mTimeStamp << " frameid " << kf->mnFrameId << " TrackReferenceForFrame " << kf->mnTrackReferenceForFrame << endl;
	cerr << " parent " << kf->GetParent() << endl;
	cerr << "children: ";
	for(auto ch: kf->GetChilds())
	  cerr << " " << ch;
	cerr <<endl;
  }
#endif
  return true;
}



// 写地图点=====================================
void Map::_WriteMapPoint(ofstream &f, MapPoint* mp) 
{
  f.write((char*)&mp->mnId, sizeof(mp->mnId));               // id: long unsigned int
  cv::Mat wp = mp->GetWorldPos();// 位置 x,y,z 
  f.write((char*)&wp.at<float>(0), sizeof(float));           // pos x: float
  f.write((char*)&wp.at<float>(1), sizeof(float));           // pos y: float
  f.write((char*)&wp.at<float>(2), sizeof(float));           // pos z: float
}

// 写关键帧
void Map::_WriteKeyFrame(ofstream &f, KeyFrame* kf, map<MapPoint*, unsigned long int>& idx_of_mp) 
{
  f.write((char*)&kf->mnId, sizeof(kf->mnId));                 // 关键帧 id: long unsigned int
  f.write((char*)&kf->mTimeStamp, sizeof(kf->mTimeStamp));     // 时间戳 ts: TimeStamp, double

#if 0
  cerr << "writting keyframe id " << kf->mnId << " ts " << kf->mTimeStamp << " frameid " << kf->mnFrameId << " TrackReferenceForFrame " << kf->mnTrackReferenceForFrame << endl;
  cerr << " parent " << kf->GetParent() << endl;
  cerr << "children: ";
  for(auto ch: kf->GetChilds())
	cerr << " " << ch->mnId;
  cerr <<endl;
  cerr << kf->mnId << " connected: (" << kf->GetConnectedKeyFrames().size() << ") ";
  for (auto ckf: kf->GetConnectedKeyFrames())
	cerr << ckf->mnId << "," << kf->GetWeight(ckf) << " ";
  cerr << endl;
#endif
  


  cv::Mat Tcw = kf->GetPose();// 关键帧位姿===
  f.write((char*)&Tcw.at<float>(0,3), sizeof(float));          // px: float
  f.write((char*)&Tcw.at<float>(1,3), sizeof(float));          // py: float
  f.write((char*)&Tcw.at<float>(2,3), sizeof(float));          // pz: float
  vector<float> Qcw = Converter::toQuaternion(Tcw.rowRange(0,3).colRange(0,3));
  f.write((char*)&Qcw[0], sizeof(float));                      // qx: float
  f.write((char*)&Qcw[1], sizeof(float));                      // qy: float
  f.write((char*)&Qcw[2], sizeof(float));                      // qz: float
  f.write((char*)&Qcw[3], sizeof(float));                      // qw: float

  f.write((char*)&kf->N, sizeof(kf->N));                       // 特征点数量 nb_features: int
  for (int i=0; i<kf->N; i++) 
  {
        // 特征点==============
	cv::KeyPoint kp = kf->mvKeys[i];
	f.write((char*)&kp.pt.x,     sizeof(kp.pt.x));               // float
	f.write((char*)&kp.pt.y,     sizeof(kp.pt.y));               // float
	f.write((char*)&kp.size,     sizeof(kp.size));               // float
	f.write((char*)&kp.angle,    sizeof(kp.angle));              // float
	f.write((char*)&kp.response, sizeof(kp.response));           // float
	f.write((char*)&kp.octave,   sizeof(kp.octave));             // int
        // 描述子
	for (int j=0; j<32; j++) 
	  f.write((char*)&kf->mDescriptors.at<unsigned char>(i,j), sizeof(char));

	unsigned long int mpidx; 
        MapPoint* mp = kf->GetMapPoint(i); // 地图点
	if (mp == NULL) mpidx = ULONG_MAX;
	else mpidx = idx_of_mp[mp]; // 所在所有地图点 map的 id
	f.write((char*)&mpidx,   sizeof(mpidx));  // long int
  }

}

// 保存 二进制 地图===================
bool Map::Save(const string &filename) 
{
  cerr << "Map: Saving to " << filename << endl;
  ofstream f;
  f.open(filename.c_str(), ios_base::out|ios::binary);
  
  cerr << "  writing " << mspMapPoints.size() << " mappoints" << endl;
  unsigned long int nbMapPoints = mspMapPoints.size();// 地图点数量
  f.write((char*)&nbMapPoints, sizeof(nbMapPoints));  // 写地图点总数
// 写入 每一个地图点=====
  for(auto mp: mspMapPoints)
	_WriteMapPoint(f, mp);

  map<MapPoint*, unsigned long int> idx_of_mp; // 地图点:id 映射====
  unsigned long int i = 0;
  for(auto mp: mspMapPoints) 
  {
	idx_of_mp[mp] = i; // 映射===
	i += 1;
  }

// 写每一个关键帧 =======  
  cerr << "  writing " << mspKeyFrames.size() << " keyframes" << endl;
  unsigned long int nbKeyFrames = mspKeyFrames.size();
  f.write((char*)&nbKeyFrames, sizeof(nbKeyFrames)); // 写关键帧数量
  for(auto kf: mspKeyFrames)
	_WriteKeyFrame(f, kf, idx_of_mp);// 写入一个关键帧=====

// 保存 父关键帧id 共视关系 和 权重，最小生成树======
  // store tree and graph
  for(auto kf: mspKeyFrames) {
	KeyFrame* parent = kf->GetParent();// 父关键帧id
	unsigned long int parent_id = ULONG_MAX; 
	if (parent) parent_id = parent->mnId;
	f.write((char*)&parent_id, sizeof(parent_id));

	unsigned long int nb_con = kf->GetConnectedKeyFrames().size();// 相关连的 关键帧
	f.write((char*)&nb_con, sizeof(nb_con));
	for (auto ckf: kf->GetConnectedKeyFrames()) 
        {
	  int weight = kf->GetWeight(ckf); // 相关关系权重(共同看到的地图点数量)
	  f.write((char*)&ckf->mnId, sizeof(ckf->mnId));
	  f.write((char*)&weight, sizeof(weight));
	}
  }

  f.close();
  cerr << "Map: finished saving" << endl;
  struct stat st;
  stat(filename.c_str(), &st);// 获取文件大小
  cerr << "Map: saved " << st.st_size << " bytes" << endl;

#if 0
  for(auto mp: mspMapPoints)
	if (!(mp->mnId%100))
	  cerr << "mp " << mp->mnId << " " << mp->Observations() << " " << mp->isBad() << endl;
#endif

  return true;


}


} //namespace ORB_SLAM
