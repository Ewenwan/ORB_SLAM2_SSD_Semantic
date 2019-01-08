/**
* This file is part of ORB-SLAM2.
* 关键帧
* KeyFrame类利用Frame类来构造。
* 对于什么样的Frame可以认为是关键帧以及何时需要加入关键帧，
* 是实现在tracking模块中的。
* 普通帧里面精选出来的具有代表性的帧
* 
* 由于KeyFrame中一部分数据会被多个线程访问修改，因此需要在这些成员中加线程锁，
* 保证同一时间只有一个线程有访问权。涉及线程安全的有：
【1】关键帧位姿的设置（lock(mMutexPose)）；
【2】关键帧间连接关系的设置（lock(mMutexConnections)）；
【3】关键帧对应地图点的操作（lock(mMutexFeatures)），包括通过地图点计算相连关键帧之间的权重。
* 

KeyFrame中维护了一个map，保存了与当前帧共视的KeyFrame*与权重（共视MapPonits数量）。
对关键帧之间关系是用加权有向图来完成的，那么理解其spanning tree生成树的原理就很有必要了。

KeyFrame中比较难理解的是SetBagFlag()函数，真实删除当前关键帧之前，
需要处理好父亲和儿子关键帧关系，不然会造成整个关键帧维护的图断裂，
或者混乱，不能够为后端提供较好的初值。

理解起来就是父亲挂了，儿子需要找新的父亲，在候选父亲里找，
当前帧的父亲（mpParent）肯定在候选父亲中的；


*/

#include "KeyFrame.h"
#include "Converter.h"
#include "ORBmatcher.h"
#include<mutex>

namespace ORB_SLAM2
{

    long unsigned int KeyFrame::nNextId=0;
    //Copy Constructor  默认初始化 主要是 直接赋值 写入到类内 变量
    // 普通帧的 量全部复制过来
    KeyFrame::KeyFrame(Frame &F, Map *pMap, KeyFrameDatabase *pKFDB):
	mnFrameId(F.mnId),  mTimeStamp(F.mTimeStamp), mnGridCols(FRAME_GRID_COLS), mnGridRows(FRAME_GRID_ROWS),
	mfGridElementWidthInv(F.mfGridElementWidthInv), mfGridElementHeightInv(F.mfGridElementHeightInv),
	mnTrackReferenceForFrame(0), mnFuseTargetForKF(0), mnBALocalForKF(0), mnBAFixedForKF(0),
	mnLoopQuery(0), mnLoopWords(0), mnRelocQuery(0), mnRelocWords(0), mnBAGlobalForKF(0),
	fx(F.fx), fy(F.fy), cx(F.cx), cy(F.cy), invfx(F.invfx), invfy(F.invfy),
	mbf(F.mbf), mb(F.mb), mThDepth(F.mThDepth), N(F.N), mvKeys(F.mvKeys), mvKeysUn(F.mvKeysUn),
	mvuRight(F.mvuRight), mvDepth(F.mvDepth), mDescriptors(F.mDescriptors.clone()),
	mBowVec(F.mBowVec), mFeatVec(F.mFeatVec), mnScaleLevels(F.mnScaleLevels), mfScaleFactor(F.mfScaleFactor),
	mfLogScaleFactor(F.mfLogScaleFactor), mvScaleFactors(F.mvScaleFactors), mvLevelSigma2(F.mvLevelSigma2),
	mvInvLevelSigma2(F.mvInvLevelSigma2), mnMinX(F.mnMinX), mnMinY(F.mnMinY), mnMaxX(F.mnMaxX),
	mnMaxY(F.mnMaxY), mK(F.mK), mvpMapPoints(F.mvpMapPoints), mpKeyFrameDB(pKFDB),
	mpORBvocabulary(F.mpORBvocabulary), mbFirstConnection(true), mpParent(NULL), mbNotErase(false),
	mbToBeErased(false), mbBad(false), mHalfBaseline(F.mb/2), mpMap(pMap)
    {
	mnId=nNextId++;// 关键帧 id =  上一关键帧 id +1
        
        // 置位 普通帧是 关键帧的 flag
        F.mIsKeyFrame = true;//
        
	mGrid.resize(mnGridCols);// 关键帧的格点 是 vertor 的 vector 的vector
	for(int i=0; i<mnGridCols;i++)// 每一列 64列
	{
	    mGrid[i].resize(mnGridRows);//变成48行大小
	    for(int j=0; j<mnGridRows; j++)//每一行
		mGrid[i][j] = F.mGrid[i][j];// 普通 帧 的 格点 关键点数量 统计 赋值 到 关键帧
	}

	SetPose(F.mTcw);    
    }
    
// 传递 深度图和彩色图
    KeyFrame::KeyFrame(Frame &F, Map *pMap, KeyFrameDatabase *pKFDB, cv::Mat rgb, cv::Mat depth):
	mnFrameId(F.mnId),  mTimeStamp(F.mTimeStamp), mnGridCols(FRAME_GRID_COLS), mnGridRows(FRAME_GRID_ROWS),
	mfGridElementWidthInv(F.mfGridElementWidthInv), mfGridElementHeightInv(F.mfGridElementHeightInv),
	mnTrackReferenceForFrame(0), mnFuseTargetForKF(0), mnBALocalForKF(0), mnBAFixedForKF(0),
	mnLoopQuery(0), mnLoopWords(0), mnRelocQuery(0), mnRelocWords(0), mnBAGlobalForKF(0),
	fx(F.fx), fy(F.fy), cx(F.cx), cy(F.cy), invfx(F.invfx), invfy(F.invfy),
	mbf(F.mbf), mb(F.mb), mThDepth(F.mThDepth), N(F.N), mvKeys(F.mvKeys), mvKeysUn(F.mvKeysUn),
	mvuRight(F.mvuRight), mvDepth(F.mvDepth), mDescriptors(F.mDescriptors.clone()),
	mBowVec(F.mBowVec), mFeatVec(F.mFeatVec), mnScaleLevels(F.mnScaleLevels), mfScaleFactor(F.mfScaleFactor),
	mfLogScaleFactor(F.mfLogScaleFactor), mvScaleFactors(F.mvScaleFactors), mvLevelSigma2(F.mvLevelSigma2),
	mvInvLevelSigma2(F.mvInvLevelSigma2), mnMinX(F.mnMinX), mnMinY(F.mnMinY), mnMaxX(F.mnMaxX),
	mnMaxY(F.mnMaxY), mK(F.mK), mvpMapPoints(F.mvpMapPoints), mpKeyFrameDB(pKFDB),
	mpORBvocabulary(F.mpORBvocabulary), mbFirstConnection(true), mpParent(NULL), mbNotErase(false),
	mbToBeErased(false), mbBad(false), mHalfBaseline(F.mb/2), mpMap(pMap)
    {
// 复制到类内变量===========
        rgb.copyTo(mImRGB);
        depth.copyTo(mImDep);

	mnId=nNextId++;// 关键帧 id =  上一关键帧 id +1
        
        // 置位 普通帧是 关键帧的 flag
        F.mIsKeyFrame = true;//
        
	mGrid.resize(mnGridCols);// 关键帧的格点 是 vertor 的 vector 的vector
	for(int i=0; i<mnGridCols;i++)// 每一列 64列
	{
	    mGrid[i].resize(mnGridRows);//变成48行大小
	    for(int j=0; j<mnGridRows; j++)//每一行
		mGrid[i][j] = F.mGrid[i][j];// 普通 帧 的 格点 关键点数量 统计 赋值 到 关键帧
	}

	SetPose(F.mTcw);    
    }

// 用单词(ORB单词词典) 线性表示 一帧所有描述子  相当于  一个句子 用几个单词 来表示
// 词典 N个M维的单词
// 一帧的描述子  n个M维的描述子
// 生成一个 N*1的向量 记录一帧的描述子 使用词典单词的情况
// 4. 将当前帧的描述子矩阵（可以转换成向量），转换成词袋模型向量
// （DBoW2::BowVector mBowVec； DBoW2::FeatureVector mFeatVec；）：
    void KeyFrame::ComputeBoW()
    {
	if(mBowVec.empty() || mFeatVec.empty())
	{
	  // mat 类型描述子 转换成 容器 mat类型
	    vector<cv::Mat> vCurrentDesc = Converter::toDescriptorVector(mDescriptors);
	    // Feature vector associate features with nodes in the 4th level (from leaves up)
	    // We assume the vocabulary tree has 6 levels, change the 4 otherwise
	    mpORBvocabulary->transform(vCurrentDesc,mBowVec,mFeatVec,4);// 计算 描述子向量 用词典线性表示的向量
	}
    }

    /*
     * 设置KeyFrame中成员变量
     * mTcw，世界坐标系 变换到   相机坐标系
     * mTwc，相机坐标系 变换到  世界坐标系
     * Ow（左目相机中心坐标）= Rcw.t() * (-tcw )  
     *                        Ow其实是世界参考系（第一帧）原点（相机光心）
     *                        在当前帧参考系（相机坐标系）中的坐标，等价于twc
     * 和Cw（双目相机baseline中点坐标），
     * 相机坐标Z朝北，X朝东，Y朝地。
     * 
     */
    void KeyFrame::SetPose(const cv::Mat &Tcw_)
    {
	unique_lock<mutex> lock(mMutexPose);
	Tcw_.copyTo(Tcw);// 拷贝到 类内变量  
	cv::Mat Rcw = Tcw.rowRange(0,3).colRange(0,3);// 世界 到 相机  旋转矩阵
	cv::Mat tcw = Tcw.rowRange(0,3).col(3);   // 世界 到 相机 平移向量
	cv::Mat Rwc = Rcw.t(); // 相机 到 世界  旋转矩阵  正交矩阵 转置 = 逆
	Ow = -Rwc*tcw;// 相机中心点在世界坐标系坐标  相机00点--->mRwc------>mtwc--------
	// 以上为 一个相机 的相机中心点坐标

// cw直接求逆计算量比较大，
//一般矩阵求逆在实现时都会用等价的矩阵表达式去表示，这里Ow就对应Tcw-1中的平移向量-RTt.	

	// 以下为 双目 相机 中心  基线 中心 点坐标 x轴方向上 平移 基线一半的距离
	Twc = cv::Mat::eye(4,4,Tcw.type());
	Rwc.copyTo(Twc.rowRange(0,3).colRange(0,3));
	Ow.copyTo(Twc.rowRange(0,3).col(3));
	cv::Mat center = (cv::Mat_<float>(4,1) << mHalfBaseline, 0 , 0, 1);// 基线 中心 点坐标 x轴方向上 平移 基线一半的距离
	Cw = Twc*center;// 关键帧 双目相机 基线 中心坐标 用于显示
    }

    // // 世界 到 相机  位姿
    cv::Mat KeyFrame::GetPose()
    {
	unique_lock<mutex> lock(mMutexPose);
	return Tcw.clone();
    }
    // 相机 到 世界  
    cv::Mat KeyFrame::GetPoseInverse()
    {
	unique_lock<mutex> lock(mMutexPose);
	return Twc.clone();
    }

    // 单个相机 光心 点坐标
    cv::Mat KeyFrame::GetCameraCenter()
    {
	unique_lock<mutex> lock(mMutexPose);
	return Ow.clone();
    }

    // 双目相机 基线中心点  坐标
    cv::Mat KeyFrame::GetStereoCenter()
    {
	unique_lock<mutex> lock(mMutexPose);
	return Cw.clone();
    }

    // 旋转向量
    cv::Mat KeyFrame::GetRotation()
    {
	unique_lock<mutex> lock(mMutexPose);
	return Tcw.rowRange(0,3).colRange(0,3).clone();
    }
    // 平移向量
    cv::Mat KeyFrame::GetTranslation()
    {
	unique_lock<mutex> lock(mMutexPose);
	return Tcw.rowRange(0,3).col(3).clone();
    }

    // 为关键帧之间添加连接，通过关键帧之间的weight连接，weight指的是两个关键帧之间共同观测到的地图点
    void KeyFrame::AddConnection(KeyFrame *pKF, const int &weight)
    {
	{
	    unique_lock<mutex> lock(mMutexConnections);
	    if(!mConnectedKeyFrameWeights.count(pKF))// 关键帧权重
		mConnectedKeyFrameWeights[pKF]=weight;
	    else if(mConnectedKeyFrameWeights[pKF]!=weight)
		mConnectedKeyFrameWeights[pKF]=weight;
	    else
		return;
	}
	// 每一个关键帧都会维护一个自己的map，其中记录了与其他关键帧之间的weight。
	// 每次为当前关键帧添加新的连接关键帧后，都需要根据weight对map结构重新排序
	UpdateBestCovisibles();
    }

    // 根据关键帧 对应的权重 进行排序
    // 返回 有序 关键帧   有序权重
    //  mConnectedKeyFrameWeights   map类型 
    // 由于map结构没有sort函数，需要将元素取出放入一个pair组成的vector中
    // 排序后 更新 mvpOrderedConnectedKeyFrames  mvOrderedWeights
    void KeyFrame::UpdateBestCovisibles()
    {
	unique_lock<mutex> lock(mMutexConnections);
	vector<pair<int,KeyFrame*> > vPairs;
	// pair是将2个数据组合成一个数据，当需要这样的需求时就可以使用pair，如stl中的map就是将key和value放在一起来保存。
	vPairs.reserve(mConnectedKeyFrameWeights.size());
	
	// mit的类型为 map<KeyFrame*,int>::iterator   auto 可以根据右式子自动推断
	for(auto mit=mConnectedKeyFrameWeights.begin(), mend=mConnectedKeyFrameWeights.end(); mit!=mend; mit++)
	  vPairs.push_back(make_pair(mit->second,mit->first));// 权重   关键帧  对

	sort(vPairs.begin(),vPairs.end());// 对关键帧 的 权重 进行 排序 
	list<KeyFrame*> lKFs;// 链表结构 的表 存储关键帧
	list<int> lWs;// 存储 对于权重 共视点数量
	for(size_t i=0, iend=vPairs.size(); i<iend;i++)
	{
	    lKFs.push_front(vPairs[i].second);//关键帧
	    lWs.push_front(vPairs[i].first);// 权重
	}
	
	mvpOrderedConnectedKeyFrames = vector<KeyFrame*>(lKFs.begin(),lKFs.end());//有序 关键帧 
	mvOrderedWeights = vector<int>(lWs.begin(), lWs.end());    // 有序权重
    }

    //  返回所有连接的关键帧  得到关键帧序列  
    // set是关联容器 未排序(set)
    set<KeyFrame*> KeyFrame::GetConnectedKeyFrames()
    {
	unique_lock<mutex> lock(mMutexConnections);
	set<KeyFrame*> s;
      // mit的类型为 map<KeyFrame*,int>::iterator   auto 可以根据右式子自动推断
	for(auto mit=mConnectedKeyFrameWeights.begin(); mit!=mConnectedKeyFrameWeights.end(); mit++)
	    s.insert(mit->first);// 插入关键帧
	return s;
    }

    // 返回所有连接的关键帧  返回全部序列 关键帧 容器
    //  vector是顺序容器  排序(vector)
    vector<KeyFrame*> KeyFrame::GetVectorCovisibleKeyFrames()
    {
	unique_lock<mutex> lock(mMutexConnections);
	return mvpOrderedConnectedKeyFrames;
    }

    // 返回前N个最优 关键帧
    vector<KeyFrame*> KeyFrame::GetBestCovisibilityKeyFrames(const int &N)
    {
	unique_lock<mutex> lock(mMutexConnections);
	if((int)mvpOrderedConnectedKeyFrames.size()<N)
	    return mvpOrderedConnectedKeyFrames;
	else
	    return vector<KeyFrame*>(mvpOrderedConnectedKeyFrames.begin(),mvpOrderedConnectedKeyFrames.begin()+N);
    }

    // 根据权重w  二分查找 有序序列 中的某个对象
    // 返回权重大于 w的关键帧
    vector<KeyFrame*> KeyFrame::GetCovisiblesByWeight(const int &w)
    {
	unique_lock<mutex> lock(mMutexConnections);

	if(mvpOrderedConnectedKeyFrames.empty())
	    return vector<KeyFrame*>();
      // 二分查找   返回 权重 w对于的元素  第一个数起始 2：尾数 3：查找的值  4：比较函数
	vector<int>::iterator it = upper_bound(mvOrderedWeights.begin(),mvOrderedWeights.end(),w,KeyFrame::weightComp);
	if(it==mvOrderedWeights.end())// 找到最后了 还没找到
	    return vector<KeyFrame*>();
	else// 找到了
	{
	    int n = it-mvOrderedWeights.begin();//  减去迭代器开始   得到差值
	    return vector<KeyFrame*>(mvpOrderedConnectedKeyFrames.begin(), mvpOrderedConnectedKeyFrames.begin()+n);
	}
    }

    // 返回关键帧对于的权重
    int KeyFrame::GetWeight(KeyFrame *pKF)
    {
	unique_lock<mutex> lock(mMutexConnections);
	if(mConnectedKeyFrameWeights.count(pKF))
	    return mConnectedKeyFrameWeights[pKF];
	else
	    return 0;
    }

    // 添加地图点
    // 当前帧对应的地图点的指针均存放在mvpMapPoints（mvp代表：member、vector、pointer）向量中
    void KeyFrame::AddMapPoint(MapPoint *pMP, const size_t &idx)
    {
	unique_lock<mutex> lock(mMutexFeatures);
	mvpMapPoints[idx]=pMP;
    }

    // 按id删除地图点 
    void KeyFrame::EraseMapPointMatch(const size_t &idx)
    {
	unique_lock<mutex> lock(mMutexFeatures);
	mvpMapPoints[idx]=static_cast<MapPoint*>(NULL);
    }
    // 按点 删除地图点 
    void KeyFrame::EraseMapPointMatch(MapPoint* pMP)
    {
	int idx = pMP->GetIndexInKeyFrame(this);//得到id
	if(idx>=0)
	    mvpMapPoints[idx]=static_cast<MapPoint*>(NULL);
    }

    // 替换地图点
    void KeyFrame::ReplaceMapPointMatch(const size_t &idx, MapPoint* pMP)
    {
	mvpMapPoints[idx]=pMP;
    }

    // 得到所有地图点  未排序(set)
    set<MapPoint*> KeyFrame::GetMapPoints()
    {
	unique_lock<mutex> lock(mMutexFeatures);
	set<MapPoint*> s;// set 集合
	for(size_t i=0, iend=mvpMapPoints.size(); i<iend; i++)
	{
	    if(!mvpMapPoints[i])
		continue;
	    MapPoint* pMP = mvpMapPoints[i];
	    if(!pMP->isBad())
		s.insert(pMP);
	}
	return s;// 返回所有地图点 set集合
    }

    // 跟踪 地图点 关键点能够被 观测到 的 个数
    // 返回高质量MapPoints（被至少minObs个关键帧观察到）的数量，
    // 其中会判断MapPoint的Observations()属性，对比给出的阈值
    int KeyFrame::TrackedMapPoints(const int &minObs)
    {
	unique_lock<mutex> lock(mMutexFeatures);

	int nPoints=0;
	const bool bCheckObs = minObs>0;
	for(int i=0; i<N; i++)// 每一个特征点
	{
	    MapPoint* pMP = mvpMapPoints[i];//关键点对应的 地图点
	    if(pMP)
	    {
		if(!pMP->isBad())
		{
		    if(bCheckObs)
		    {
			if(mvpMapPoints[i]->Observations() >= minObs)// 可以被观测到的次数较大
			    nPoints++;//可以被观测到
		    }
		    else
			nPoints++;
		}
	    }
	}
	return nPoints;
    }
    
    // 得到所有地图点  排序(vector)
    vector<MapPoint*> KeyFrame::GetMapPointMatches()
    {
	unique_lock<mutex> lock(mMutexFeatures);
	return mvpMapPoints;
    }

    MapPoint* KeyFrame::GetMapPoint(const size_t &idx)
    {
	unique_lock<mutex> lock(mMutexFeatures);
	return mvpMapPoints[idx];
    }

   // 建立关键帧之间的连接关系
    void KeyFrame::UpdateConnections()
    {
	map<KeyFrame*,int> KFcounter;// 帧 观测到 地图点的次数

	vector<MapPoint*> vpMP;//所有地图点

	{
	    unique_lock<mutex> lockMPs(mMutexFeatures);
	    vpMP = mvpMapPoints;
	}

	//For all map points in keyframe check in which other keyframes are they seen
	//Increase counter for those keyframes
	// vector<MapPoint*>::iterator vit 
	for(auto vit = vpMP.begin(), vend=vpMP.end(); vit!=vend; vit++)// 每一个 关键点
	{
	    MapPoint* pMP = *vit;// *vit 迭代器里的内容 为 地图点指针 MapPoint*

	    if(!pMP)
		continue;

	    if(pMP->isBad())// 指针 解应用 + 访问 成员函数  ->
		continue;
	    
	    //  map<KeyFrame*,size_t> observations observations 
	  auto  observations = pMP->GetObservations();// 关键点 所在的 关键帧
	  // map<KeyFrame*,size_t>::iterator mit 
    // 地图点的 观测帧
	    for(auto  mit = observations.begin(), mend=observations.end(); mit!=mend; mit++)
	    {
		if(mit->first->mnId == mnId)
		    continue;
		KFcounter[mit->first]++;// 该帧 观测到 关键点一次   该帧 观测到 地图点的次数
	    }
	}

	// This should not happen
	if(KFcounter.empty())
	    return;

	//If the counter is greater than threshold add connection
	//In case no keyframe counter is over threshold add the one with maximum counter
	int nmax=0;
	KeyFrame* pKFmax=NULL;
	int th = 15;// 被观测次数 阈值

	vector<pair<int,KeyFrame*> > vPairs;// 容器 键值对  保留大于观测次阈值的 关键帧 和其观测次数
	vPairs.reserve(KFcounter.size());
	// map<KeyFrame*,int>::iterator mit
	for(auto  mit = KFcounter.begin(), mend=KFcounter.end(); mit!=mend; mit++)
	{
	    if(mit->second > nmax)
	    {
		nmax = mit->second;//  观测到地图点 次数最大 的 关键帧 的观测次数
		pKFmax=mit->first;//  观测到地图点 次数最大 的 关键帧 
	    }
	    if(mit->second >= th)// 大于阈值
	    {
		vPairs.push_back(make_pair(mit->second , mit->first));//保留大于观测次阈值的 关键帧 和其观测次数 
		(mit->first)->AddConnection(this, mit->second);// 
	    }
	}

	if(vPairs.empty())
	{
	    vPairs.push_back(make_pair(nmax,pKFmax));
	    pKFmax->AddConnection(this,nmax);
	}

        sort(vPairs.begin(),vPairs.end());// 超过 15次 观测到地图点 的 帧 再排序
	list<KeyFrame*> lKFs;// 关键帧
	list<int> lWs;// 被观测次数
	for(size_t i=0; i<vPairs.size();i++)
	{
	    lKFs.push_front(vPairs[i].second);// 有序
	    lWs.push_front(vPairs[i].first);
	}

	{
	    unique_lock<mutex> lockCon(mMutexConnections);

	    // mspConnectedKeyFrames = spConnectedKeyFrames;
	    mConnectedKeyFrameWeights = KFcounter;
	    mvpOrderedConnectedKeyFrames = vector<KeyFrame*>(lKFs.begin(),lKFs.end());// 超过 15词被观测到 的 关键帧
	    mvOrderedWeights = vector<int>(lWs.begin(), lWs.end());//最好的 关键帧序列

	    if(mbFirstConnection && mnId!=0)
	    {
		mpParent = mvpOrderedConnectedKeyFrames.front();
		mpParent->AddChild(this);
		mbFirstConnection = false;
	    }

	}
    }

    // 关键帧树 操作

    // 添加孩子
    // set 红黑二叉树
    void KeyFrame::AddChild(KeyFrame *pKF)
    {
	unique_lock<mutex> lockCon(mMutexConnections);
	mspChildrens.insert(pKF);// 插入
    }

    // 删除孩子
    void KeyFrame::EraseChild(KeyFrame *pKF)
    {
	unique_lock<mutex> lockCon(mMutexConnections);
	mspChildrens.erase(pKF);
    }

    void KeyFrame::ChangeParent(KeyFrame *pKF)
    {
	unique_lock<mutex> lockCon(mMutexConnections);
	mpParent = pKF;
	pKF->AddChild(this);
    }

    set<KeyFrame*> KeyFrame::GetChilds()
    {
	unique_lock<mutex> lockCon(mMutexConnections);
	return mspChildrens;
    }

    KeyFrame* KeyFrame::GetParent()
    {
	unique_lock<mutex> lockCon(mMutexConnections);
	return mpParent;
    }

    bool KeyFrame::hasChild(KeyFrame *pKF)
    {
	unique_lock<mutex> lockCon(mMutexConnections);
	return mspChildrens.count(pKF);
    }

    void KeyFrame::AddLoopEdge(KeyFrame *pKF)
    {
	unique_lock<mutex> lockCon(mMutexConnections);
	mbNotErase = true;
	mspLoopEdges.insert(pKF);
    }

    set<KeyFrame*> KeyFrame::GetLoopEdges()//
    {
	unique_lock<mutex> lockCon(mMutexConnections);
	return mspLoopEdges;
    }

    void KeyFrame::SetNotErase()
    {
	unique_lock<mutex> lock(mMutexConnections);
	mbNotErase = true;
    }

    // 首先设置为坏帧，如果该帧不是回环帧，则可以真的删掉；如果该帧是回环帧，怎么都删不掉的
    void KeyFrame::SetErase()
    {
	{
	    unique_lock<mutex> lock(mMutexConnections);
	    if(mspLoopEdges.empty())
	    {
		mbNotErase = false;
	    }
	}

	if(mbToBeErased)
	{
	    SetBadFlag();
	}
    }
/*KeyFrame中比较难理解的是SetBagFlag()函数，真实删除当前关键帧之前，需要处理好父亲和儿子关键帧关系，
 * 
 * 不然会造成整个关键帧维护的图断裂，或者混乱，不能够为后端提供较好的初值。
     理解起来就是父亲挂了，儿子需要找新的父亲，在候选父亲里找，
     当前帧的父亲（mpParent）肯定在候选父亲中的；
     当前帧的父亲帧删除 当前帧
     地图删除 当前帧
     关键帧数据库 删除 当前帧
 */
    void KeyFrame::SetBadFlag()
    {   
	{
	    unique_lock<mutex> lock(mMutexConnections);
	    if(mnId==0)
		return;
	    else if(mbNotErase)
	    {
		mbToBeErased = true;
		return;
	    }
	}
    //【1】 删除原有 连接关系
       // map<KeyFrame*,int>::iterator
	for(auto mit = mConnectedKeyFrameWeights.begin(), mend=mConnectedKeyFrameWeights.end(); mit != mend; mit++)
	    mit->first->EraseConnection(this);

	for(size_t i=0; i<mvpMapPoints.size(); i++)
	    if(mvpMapPoints[i])
		mvpMapPoints[i]->EraseObservation(this);
	   {
	    unique_lock<mutex> lock(mMutexConnections);
	    unique_lock<mutex> lock1(mMutexFeatures);

	    mConnectedKeyFrameWeights.clear();
	    mvpOrderedConnectedKeyFrames.clear();

	    // Update Spanning Tree
   // 【2】首先将当前帧的父亲，放入候选父亲中    
	    set<KeyFrame*> sParentCandidates;
	    sParentCandidates.insert(mpParent);

	    // Assign at each iteration one children with a parent (the pair with highest covisibility weight)
	    // Include that children as new parent candidate for the rest
	    while(!mspChildrens.empty())
	    {
		bool bContinue = false;

		int max = -1;
		KeyFrame* pC;
		KeyFrame* pP;
		
	// 【3】 遍历当前帧的所有儿子	  每个儿子需要在 父亲帧的父亲帧中 找到一个  新父亲帧
                  // set<KeyFrame*>::iterator
		for(auto sit=mspChildrens.begin(), send=mspChildrens.end(); sit != send; sit++)
		{
		    KeyFrame* pKF = *sit;// 当前帧的 儿子帧
		    if(pKF->isBad())
			continue;
		    
           // 【4】然后遍历儿子A的每个共视帧
		    // Check if a parent candidate is connected to the keyframe
		    vector<KeyFrame*> vpConnected = pKF->GetVectorCovisibleKeyFrames();//儿子帧A的所有共视帧
		    for(size_t i=0, iend=vpConnected.size(); i<iend; i++)
		    {   // set<KeyFrame*>::iterator
	        // 【5】查看儿子帧的每一个共视帧 是不是 候选父亲帧 中的一个   
			for( auto spcit=sParentCandidates.begin(), spcend=sParentCandidates.end(); spcit!=spcend; spcit++)
			{
		 // 【6】 儿子帧A的 共视帧  中 有 有候选父亲 帧  
			// 则将 儿子帧A的父亲更新为该 候选父亲  应为当前帧需要删除 儿子需要向上找 新父亲
			    if(vpConnected[i]->mnId == (*spcit)->mnId)
			    {
				int w = pKF->GetWeight(vpConnected[i]);
				if(w>max)
				{
				    pC = pKF;//  共视帧
				    pP = vpConnected[i];// 父亲帧
				    max = w;
				    bContinue = true;
				}
			    }
			}
		    }
		}

		if(bContinue)
		{
		    pC->ChangeParent(pP);
		    sParentCandidates.insert(pC);// 并且将A放入候选父亲中（因为这时候A已经将整个图联系起来了）
		    mspChildrens.erase(pC);
		}
		else
		    break;
	    }

	    // If a children has no covisibility links with any parent candidate, assign to the original parent of this KF
     // 如果遍历一圈下来，发现有的儿子还没有找到新父亲，例如儿子B的共视帧不是候选父亲里的任何一个。
     // 这种情况出现在，B和当前帧的父亲不存在共视关系（速度太快，旋转太急，匹配跟丢）。
     // 并且B与当前帧的儿子之间也没有共视关系：当前帧不是一个好的关键帧，本来就没有多少儿子；
     // 或者B本身是个例外，恩，反正B是个孤家寡人。。。
	    if(!mspChildrens.empty())
	      // set<KeyFrame*>::iterator
		for(auto  sit=mspChildrens.begin(); sit != mspChildrens.end(); sit++)
		{
		    (*sit)->ChangeParent(mpParent);// 直接将儿子帧B的父亲 设置为当前帧的父亲，交给爷爷去管
		}

	    mpParent->EraseChild(this);// 父亲删除当前帧
	    mTcp = Tcw*mpParent->GetPoseInverse();
	    mbBad = true;
	}
	mpMap->EraseKeyFrame(this);//地图删除当前帧
	mpKeyFrameDB->erase(this);// 关键帧数据库 删除当前帧
    }

    bool KeyFrame::isBad()
    {
	unique_lock<mutex> lock(mMutexConnections);
	return mbBad;
    }

    void KeyFrame::EraseConnection(KeyFrame* pKF)
    {
	bool bUpdate = false;
	{
	    unique_lock<mutex> lock(mMutexConnections);
	    if(mConnectedKeyFrameWeights.count(pKF))
	    {
		mConnectedKeyFrameWeights.erase(pKF);
		bUpdate=true;
	    }
	}

	if(bUpdate)
	    UpdateBestCovisibles();
    }

    vector<size_t> KeyFrame::GetFeaturesInArea(const float &x, const float &y, const float &r) const
    {
	vector<size_t> vIndices;
	vIndices.reserve(N);

	const int nMinCellX = max(0,(int)floor((x-mnMinX-r)*mfGridElementWidthInv));
	if(nMinCellX>=mnGridCols)
	    return vIndices;

	const int nMaxCellX = min((int)mnGridCols-1,(int)ceil((x-mnMinX+r)*mfGridElementWidthInv));
	if(nMaxCellX<0)
	    return vIndices;

	const int nMinCellY = max(0,(int)floor((y-mnMinY-r)*mfGridElementHeightInv));
	if(nMinCellY>=mnGridRows)
	    return vIndices;

	const int nMaxCellY = min((int)mnGridRows-1,(int)ceil((y-mnMinY+r)*mfGridElementHeightInv));
	if(nMaxCellY<0)
	    return vIndices;

	for(int ix = nMinCellX; ix<=nMaxCellX; ix++)
	{
	    for(int iy = nMinCellY; iy<=nMaxCellY; iy++)
	    {
		const vector<size_t> vCell = mGrid[ix][iy];
		for(size_t j=0, jend=vCell.size(); j<jend; j++)
		{
		    const cv::KeyPoint &kpUn = mvKeysUn[vCell[j]];
		    const float distx = kpUn.pt.x-x;
		    const float disty = kpUn.pt.y-y;

		    if(fabs(distx)<r && fabs(disty)<r)
			vIndices.push_back(vCell[j]);
		}
	    }
	}

	return vIndices;
    }

    bool KeyFrame::IsInImage(const float &x, const float &y) const
    {
	return (x>=mnMinX && x<mnMaxX && y>=mnMinY && y<mnMaxY);
    }

    cv::Mat KeyFrame::UnprojectStereo(int i)
    {
	const float z = mvDepth[i];
	if(z>0)
	{
	    const float u = mvKeys[i].pt.x;
	    const float v = mvKeys[i].pt.y;
	    const float x = (u-cx)*z*invfx;
	    const float y = (v-cy)*z*invfy;
	    cv::Mat x3Dc = (cv::Mat_<float>(3,1) << x, y, z);

	    unique_lock<mutex> lock(mMutexPose);
	    return Twc.rowRange(0,3).colRange(0,3)*x3Dc+Twc.rowRange(0,3).col(3);
	}
	else
	    return cv::Mat();
    }

    //  单目 环境 深度中值
    float KeyFrame::ComputeSceneMedianDepth(const int q)
    {
	vector<MapPoint*> vpMapPoints;
	cv::Mat Tcw_;
	{
	    unique_lock<mutex> lock(mMutexFeatures);
	    unique_lock<mutex> lock2(mMutexPose);
	    vpMapPoints = mvpMapPoints;
	    Tcw_ = Tcw.clone();
	}

	vector<float> vDepths;
	vDepths.reserve(N);
	cv::Mat Rcw2 = Tcw_.row(2).colRange(0,3);// 第三行 乘以 坐标 是z轴的坐标值
	Rcw2 = Rcw2.t();
	float zcw = Tcw_.at<float>(2,3);// z轴 平移量 深度平移
	for(int i=0; i<N; i++)
	{
	    if(mvpMapPoints[i])
	    {
		MapPoint* pMP = mvpMapPoints[i];
		cv::Mat x3Dw = pMP->GetWorldPos();
		float z = Rcw2.dot(x3Dw) + zcw;// 第三行 乘以 坐标 是z轴的坐标值 再加上 z轴的平移量
		vDepths.push_back(z);//得到深度
	    }
	}

	sort(vDepths.begin(),vDepths.end());// 排序

	return vDepths[(vDepths.size()-1)/q];//深度中值
    }

} //namespace ORB_SLAM
