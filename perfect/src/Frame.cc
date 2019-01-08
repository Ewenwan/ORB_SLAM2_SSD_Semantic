/**
* This file is part of ORB-SLAM2.
*  普通帧 每一幅 图像都会生成 一个帧
* 
************双目相机帧*************
* 左右图
* orb特征提取
* 计算匹配点对
* 由视差计算对应关键点的深度距离
* 
* 双目 立体匹配
* 1】为左目每个特征点建立带状区域搜索表，限定搜索区域，（前已进行极线校正）
* 2】在限定区域内 通过描述子进行 特征点匹配，得到每个特征点最佳匹配点（scaleduR0）
* 3】通过SAD滑窗得到匹配修正量 bestincR
* 4】(bestincR, dist)  (bestincR - 1, dist)  (bestincR +1, dist) 三点拟合出抛物线，得到亚像素修正量 deltaR
* 5】最终匹配点位置 为 : scaleduR0 + bestincR  + deltaR
* 
* 视差 Disparity 和深度Depth
* z = bf /d      b 双目相机基线长度  f为焦距  d为视差(同一点在两相机像素平面 水平方向像素单位差值)
* 
* 为特征点分配网格
* 
******深度相机 帧******************
* 深度值 由未校正特征点 坐标对于应深度图 中的 值确定
* 匹配点坐标横坐标值 为 特征点坐标 横坐标值 - 视差 = 特征点坐标 横坐标值  - bf / 深度
* 为特征点分配网格
* 
********单目相机帧****************
* 深度值容器初始化
* 匹配点坐标 容器初始化
* 为特征点分配网格
* 
* 
* *************图像金字塔********************************
* 0层是原始图。对0层高斯核卷积后，降采样（删除所有偶数行和偶数列）
* 即可得到高斯金字塔第1层；插入0用高斯卷积恢复成原始大小，与0层相减，得到0层拉普拉斯金字塔，
* 对应的是0层高斯金字塔在滤波降采样过程中丢失的信息，在其上可以提取特征。
* 然后不断降采样，获得不同层的高斯金字塔与拉普拉斯金字塔，
* 提取出的特征对应的尺度与金字塔的层数是正相关的。层数越高，对应的尺度越大，尺度不确定性越高。
* 通过对图像的这种处理，我们可以提取出尺寸不变的的特征。
* 但是在特征匹配时，需要考虑到提取特征点时对应的层数（尺度）。
* 
* 
*/

#include "Frame.h"
#include "Converter.h"
#include "ORBmatcher.h"
#include <thread>

namespace ORB_SLAM2
{

      long unsigned int Frame::nNextId=0;
      bool Frame::mbInitialComputations=true;
      float Frame::cx, Frame::cy, Frame::fx, Frame::fy, Frame::invfx, Frame::invfy;
      float Frame::mnMinX, Frame::mnMinY, Frame::mnMaxX, Frame::mnMaxY;
      float Frame::mfGridElementWidthInv, Frame::mfGridElementHeightInv;

	// 一些常用代码片段 封装 成函数=============
      void Frame::InitializeScaleLevels() 
      {  // 金字塔尺度
	  mnScaleLevels = mpORBextractorLeft->GetLevels();
	  mfScaleFactor = mpORBextractorLeft->GetScaleFactor();
	  mfLogScaleFactor = log(mfScaleFactor);
	  mvScaleFactors = mpORBextractorLeft->GetScaleFactors();
	  mvInvScaleFactors = mpORBextractorLeft->GetInverseScaleFactors();
	  mvLevelSigma2 = mpORBextractorLeft->GetScaleSigmaSquares();
	  mvInvLevelSigma2 = mpORBextractorLeft->GetInverseScaleSigmaSquares();
      }

// 相机参数可以单独 提出来 做一个类
	void Frame::ComputeImageBounds()
	{
          int Camera_width=480, Camera_height=640;
	  // if(Camera::DistCoef.at<float>(0)!=0.0)
          if(mDistCoef.at<float>(0)!=0.0)
	    {
		cv::Mat mat(4,2,CV_32F);
		mat.at<float>(0,0)=0.0; mat.at<float>(0,1)=0.0;
		mat.at<float>(1,0)=Camera_width; mat.at<float>(1,1)=0.0;
		mat.at<float>(2,0)=0.0; mat.at<float>(2,1)=Camera_height;
		mat.at<float>(3,0)=Camera_width; mat.at<float>(3,1)=Camera_height;

		// Undistort corners
		mat=mat.reshape(2);
		//cv::undistortPoints(mat, mat, Camera::K, Camera::DistCoef, cv::Mat(), Camera::K);
                cv::undistortPoints(mat, mat, mK, mDistCoef, cv::Mat(), mK);
		mat=mat.reshape(1);

			Frame::mnMinX = min(mat.at<float>(0,0),mat.at<float>(2,0));
			Frame::mnMaxX = max(mat.at<float>(1,0),mat.at<float>(3,0));
			Frame::mnMinY = min(mat.at<float>(0,1),mat.at<float>(1,1));
			Frame::mnMaxY = max(mat.at<float>(2,1),mat.at<float>(3,1));

	    }
	    else
	    {
		  Frame::mnMinX = 0.0f;
		  Frame::mnMaxX = Camera_width;
		  Frame::mnMinY = 0.0f;
		  Frame::mnMaxY = Camera_height;
	    }
	}

      void Frame::InitializeClass(cv::Mat K) 
      {
	  if(Frame::mbInitialComputations)
	    {
		  cerr << "in Frame::InitializeClass" << endl;
		  ComputeImageBounds();
	Frame::mfGridElementWidthInv=static_cast<float>(FRAME_GRID_COLS)/
                                                       (Frame::mnMaxX-Frame::mnMinX);
        Frame::mfGridElementHeightInv=static_cast<float>(FRAME_GRID_ROWS)/
                                                       (Frame::mnMaxY-Frame::mnMinY);
	      fx = K.at<float>(0,0);
	      fy = K.at<float>(1,1);
	      cx = K.at<float>(0,2);
	      cy = K.at<float>(1,2);
	      invfx = 1.0f/fx;
	      invfy = 1.0f/fy;

	Frame::mbInitialComputations=false;
	    }
      }

//////////////////////////////////////////////////




      Frame::Frame()
      {}

      /**
      * @brief Copy constructor    默认初始化 主要是 直接赋值 写入到类内 变量
      *
      *  拷贝构造函数, mLastFrame = Frame(mCurrentFrame)
      */
      Frame::Frame(const Frame &frame)
	  :mpORBvocabulary(frame.mpORBvocabulary), mpORBextractorLeft(frame.mpORBextractorLeft), mpORBextractorRight(frame.mpORBextractorRight),
	  mTimeStamp(frame.mTimeStamp), mK(frame.mK.clone()), mDistCoef(frame.mDistCoef.clone()),
	  mbf(frame.mbf), mb(frame.mb), mThDepth(frame.mThDepth), N(frame.N), mvKeys(frame.mvKeys),
	  mvKeysRight(frame.mvKeysRight), mvKeysUn(frame.mvKeysUn),  mvuRight(frame.mvuRight),
	  mvDepth(frame.mvDepth), mBowVec(frame.mBowVec), mFeatVec(frame.mFeatVec),
	  mDescriptors(frame.mDescriptors.clone()), mDescriptorsRight(frame.mDescriptorsRight.clone()),
	  mvpMapPoints(frame.mvpMapPoints), mvbOutlier(frame.mvbOutlier), mnId(frame.mnId),
	  mpReferenceKF(frame.mpReferenceKF), mnScaleLevels(frame.mnScaleLevels),
	  mfScaleFactor(frame.mfScaleFactor), mfLogScaleFactor(frame.mfLogScaleFactor),
	  mvScaleFactors(frame.mvScaleFactors), mvInvScaleFactors(frame.mvInvScaleFactors),
	  mvLevelSigma2(frame.mvLevelSigma2), mvInvLevelSigma2(frame.mvInvLevelSigma2),
          mImDynm_mask(frame.mImDynm_mask),// 动静mask cv::Mat=============
          mIsKeyFrame(frame.mIsKeyFrame),  // 是否是关键帧 bool===============
          mImDepth(frame.mImDepth)         // 深度图
      {
	  for(int i=0;i<FRAME_GRID_COLS;i++)//64列
	      for(int j=0; j<FRAME_GRID_ROWS; j++)//48行
		  mGrid[i][j]=frame.mGrid[i][j];//存vector的数组 格子特征点数 赋值 

	  if(!frame.mTcw.empty())
	      SetPose(frame.mTcw);
      }
      
      /**
      * @brief  双目 立体匹配  初始化 构造帧
      * @param imLeft       		 左图像
      * @param imRight    		 右图像
      * @param timeStamp		 时间戳
      * @param extractorLeft          左图像 orb 特征提取器
      * @param extractorRight        右图像 orb 特征提取器
      * @param voc                           orb 字典
      * @param K                               相机内参数
      * @param distCoef                   畸变校正参数
      * @param bf                             双目相机基线 × 焦距
      * @param thDepth                  
      */
      Frame::Frame(const cv::Mat &imLeft, const cv::Mat &imRight, const double &timeStamp, ORBextractor* extractorLeft, ORBextractor* extractorRight, ORBVocabulary* voc, cv::Mat &K, cv::Mat &distCoef, const float &bf, const float &thDepth)
	  :mpORBvocabulary(voc),mpORBextractorLeft(extractorLeft),mpORBextractorRight(extractorRight), mTimeStamp(timeStamp), mK(K.clone()),mDistCoef(distCoef.clone()), mbf(bf), mThDepth(thDepth),
	  mpReferenceKF(static_cast<KeyFrame*>(NULL))
      {
	  // Frame ID
	  mnId=nNextId++;

	  // Scale Level Info
	  // 特征点匹配 图像金字塔参数
	  mnScaleLevels = mpORBextractorLeft->GetLevels();// 特征提取 图像金字塔 层数
	  mfScaleFactor = mpORBextractorLeft->GetScaleFactor();//尺度
	  mfLogScaleFactor = log(mfScaleFactor);
	  mvScaleFactors = mpORBextractorLeft->GetScaleFactors();
	  mvInvScaleFactors = mpORBextractorLeft->GetInverseScaleFactors();
	  mvLevelSigma2 = mpORBextractorLeft->GetScaleSigmaSquares();
	  mvInvLevelSigma2 = mpORBextractorLeft->GetInverseScaleSigmaSquares();

	  // ORB extraction
	  // 左右相机图像 ORB特征提取   未校正的图像  得到 关键点位置后 直接对 关键点坐标进行校正
	  thread threadLeft(&Frame::ExtractORB,this,0,imLeft);// 左相机     提取 orb特征点 和描述子  线程 关键点 mvKeys   描述子mDescriptors
	  thread threadRight(&Frame::ExtractORB,this,1,imRight);// 又相机 提取 orb特征点 和描述子                       mvKeysRight     mDescriptorsRight
	  threadLeft.join();//加入到线程
	  threadRight.join();


      // 向量mvKeys中存放N个提取出的左图关键点，mDescriptor中存放提取出的左图描述子，
	  // 右图的放在 mvKeysRight 和 mDescriptorsRight 中；
	  N = mvKeys.size();//左图关键点数量 
	  if(mvKeys.empty())
	      return;

	  // Undistort特征点，这里没有对双目进行校正，因为要求输入的图像已经进行极线校正
	  UndistortKeyPoints();
      // 双目匹配 计算匹配点对  根据视差计算深度信息  d= mbf/d
	   // 计算双目间的匹配, 匹配成功的特征点会计算其深度
           // 深度存放在 mvDepth 中  与右图 匹配点横坐标差 放在 mvuRight  
	  ComputeStereoMatches();

      // 初始化地图点及其外点；
           //为每一个关键点 构造一个空的地图点
	  mvpMapPoints = vector<MapPoint*>(N,static_cast<MapPoint*>(NULL));    
	  mvbOutlier = vector<bool>(N,false);
           // 对应的地图点是否是外点 地图点按照 [R t]投影到 本帧图上 是否在 图像范围内


	  // This is done only for the first Frame (or after a change in the calibration)
	  if(mbInitialComputations)//第一帧 进行计算
	  {
	    // 对于未校正的图像 计算校正后 图像的 尺寸 mnMinX  , mnMaxX   mnMinY,  mnMaxY
	    ComputeImageBounds(imLeft);
	    // 640*480 图像 分成 64*48 个网格
	    // 计算每个 像素 占据的 网格量   像素差×该量 得到 网格下标
	      mfGridElementWidthInv=static_cast<float>(FRAME_GRID_COLS)/(mnMaxX-mnMinX);// 每个像素占用的网格数
	      mfGridElementHeightInv=static_cast<float>(FRAME_GRID_ROWS)/(mnMaxY-mnMinY);

	      fx = K.at<float>(0,0);
	      fy = K.at<float>(1,1);
	      cx = K.at<float>(0,2);
	      cy = K.at<float>(1,2);
	      invfx = 1.0f/fx;
	      invfy = 1.0f/fy;

	      mbInitialComputations=false;
	  }

	  mb = mbf/fx;// 双目相机基线长度
	
	// 按照 特征点 的像素坐标  分配到各个网格内
	// 每个网格记录了 特征点的 序列下标
	// 最后将关键点分布到64*48分割而成的网格中（目的是加速匹配以及均匀化关键点分布）
	  AssignFeaturesToGrid();
      }
      
      /**
      * @brief  深度相机 初始化  帧结构 灰度图 深度图
      * @param mGray		 灰度图
      * @param imDepth         深度图
      */
      Frame::Frame(const cv::Mat &imGray, const cv::Mat &imDepth, 
                   const double &timeStamp, ORBextractor* extractor,ORBVocabulary* voc, 
                   cv::Mat &K, cv::Mat &distCoef, const float &bf, const float &thDepth)
	  :mpORBvocabulary(voc),mpORBextractorLeft(extractor),mpORBextractorRight(static_cast<ORBextractor*>(NULL)),
	  mTimeStamp(timeStamp), mK(K.clone()),mDistCoef(distCoef.clone()), mbf(bf), mThDepth(thDepth),
          mImDepth(imDepth)
      {
	  // Frame ID
	// int类型  帧id ++
	  mnId=nNextId++;

	  // Scale Level Info
	  // 特征点匹配 图像金字塔参数
	  mnScaleLevels = mpORBextractorLeft->GetLevels();
	  mfScaleFactor = mpORBextractorLeft->GetScaleFactor();    
	  mfLogScaleFactor = log(mfScaleFactor);
	  mvScaleFactors = mpORBextractorLeft->GetScaleFactors();
	  mvInvScaleFactors = mpORBextractorLeft->GetInverseScaleFactors();
	  mvLevelSigma2 = mpORBextractorLeft->GetScaleSigmaSquares();
	  mvInvLevelSigma2 = mpORBextractorLeft->GetInverseScaleSigmaSquares();

	  // ORB extraction
	  // 图像 ORB特征提取   未校正的图像  得到 关键点位置后 直接对 关键点坐标进行校正
	  ExtractORB(0,imGray);//  

	  N = mvKeys.size();// 特征点数量

	  if(mvKeys.empty())
	      return;//没有提取到特征点
	  // 对关键点坐标进行校正    关键点   mvKeys -----> 畸变校正------>  mvKeysUn 
	  UndistortKeyPoints();
	  
      // 深度相机 计算  深度值 根据未校正的关键点 在深度图中的值 获得
         // 匹配点横坐标 有原特征点校正的后横坐标 -  视差；    视差 = bf / 深度
	 // 深度存放在 mvDepth 中  右图 匹配点横坐标放在 mvuRight  
	  ComputeStereoFromRGBD(imDepth);

	  // 关键点 转成 的地图点
	  mvpMapPoints = vector<MapPoint*>(N,static_cast<MapPoint*>(NULL));
	  mvbOutlier = vector<bool>(N,false);

	  // This is done only for the first Frame (or after a change in the calibration)
	  if(mbInitialComputations)//第一帧 进行计算
	  {
	    // 对于未校正的图像 计算校正后 图像的 尺寸 mnMinX  , mnMaxX   mnMinY,  mnMaxY
	      ComputeImageBounds(imGray);
	    // 640*480 图像 分成 64*48 个网格
	    // 计算每个 像素 占据的 网格量   像素差×该量 得到 网格下标
	      mfGridElementWidthInv=static_cast<float>(FRAME_GRID_COLS)/static_cast<float>(mnMaxX-mnMinX);
	      mfGridElementHeightInv=static_cast<float>(FRAME_GRID_ROWS)/static_cast<float>(mnMaxY-mnMinY);

	      fx = K.at<float>(0,0);
	      fy = K.at<float>(1,1);
	      cx = K.at<float>(0,2);
	      cy = K.at<float>(1,2);
	      invfx = 1.0f/fx;
	      invfy = 1.0f/fy;

	      mbInitialComputations=false;
	  }

	  mb = mbf/fx;// 双目相机基线长度
	
	// 按照 特征点 的像素坐标  分配到各个网格内
	// 每个网格记录了 特征点的 序列下标
	// 最后将关键点分布到64*48分割而成的网格中（目的是加速匹配以及均匀化关键点分布）
	  AssignFeaturesToGrid();
      }

/* 
添加 关键点剔除/保留mask，用来剔除 认为是 动态点的 关键点

*/
      Frame::Frame(const cv::Mat &imGray, const cv::Mat &imDepth,
                   const cv::Mat &imMask,// 动静mask，1的部分保留，0的部分（大概率为动态物体点）去除
                   const double &timeStamp, ORBextractor* extractor,ORBVocabulary* voc, 
                   cv::Mat &K, cv::Mat &distCoef, const float &bf, const float &thDepth)
	  :mpORBvocabulary(voc),mpORBextractorLeft(extractor),
           mpORBextractorRight(static_cast<ORBextractor*>(NULL)),
	   mTimeStamp(timeStamp), mK(K.clone()),
           mDistCoef(distCoef.clone()), mbf(bf), mThDepth(thDepth),
           mImDepth(imDepth),mImDynm_mask(imMask)
{
	  // Frame ID
	// int类型  帧id ++
	  mnId=nNextId++;

	  // Scale Level Info
	  // 特征点匹配 图像金字塔参数
	  mnScaleLevels = mpORBextractorLeft->GetLevels();
	  mfScaleFactor = mpORBextractorLeft->GetScaleFactor();    
	  mfLogScaleFactor = log(mfScaleFactor);
	  mvScaleFactors = mpORBextractorLeft->GetScaleFactors();
	  mvInvScaleFactors = mpORBextractorLeft->GetInverseScaleFactors();
	  mvLevelSigma2 = mpORBextractorLeft->GetScaleSigmaSquares();
	  mvInvLevelSigma2 = mpORBextractorLeft->GetInverseScaleSigmaSquares();

	  // ORB extraction
	  // 图像 ORB特征提取   未校正的图像  得到 关键点位置后 直接对 关键点坐标进行校正
	  ExtractORB(0,imGray);//  

/////////////////////////////////////////////////////////=========剔除运动点==============
            cv::Scalar s = sum(imMask);//各通道求和
           if(s.val[0] > imMask.rows * imMask.cols * 0.65) 
// 避免运动过大，导致判断出错，导致关键点数量较少=================
           {
	    std::vector<cv::KeyPoint> _mvKeys;
	    cv::Mat _mDescriptors;
	    for (size_t i(0); i < mvKeys.size(); ++i)
	    {
		int val = (int)imMask.at<uchar>(mvKeys[i].pt.y,mvKeys[i].pt.x);
		if (val == 1)// 在mask内部=============
		{
		    _mvKeys.push_back(mvKeys[i]);
		    _mDescriptors.push_back(mDescriptors.row(i));
		}
	    }
	    mvKeys = _mvKeys;
	    mDescriptors = _mDescriptors;
           }
//////////////////////==========================================================


	  N = mvKeys.size();// 特征点数量

	  if(mvKeys.empty())
	      return;//没有提取到特征点
	  // 对关键点坐标进行校正    关键点   mvKeys -----> 畸变校正------>  mvKeysUn 
	  UndistortKeyPoints();
	  
      // 深度相机 计算  深度值 根据未校正的关键点 在深度图中的值 获得
         // 匹配点横坐标 有原特征点校正的后横坐标 -  视差；    视差 = bf / 深度
	 // 深度存放在 mvDepth 中  右图 匹配点横坐标放在 mvuRight  
	  ComputeStereoFromRGBD(imDepth);

	  // 关键点 转成 的地图点
	  mvpMapPoints = vector<MapPoint*>(N,static_cast<MapPoint*>(NULL));
	  mvbOutlier = vector<bool>(N,false);

	  // This is done only for the first Frame (or after a change in the calibration)
	  if(mbInitialComputations)//第一帧 进行计算
	  {
	    // 对于未校正的图像 计算校正后 图像的 尺寸 mnMinX  , mnMaxX   mnMinY,  mnMaxY
	      ComputeImageBounds(imGray);
	    // 640*480 图像 分成 64*48 个网格
	    // 计算每个 像素 占据的 网格量   像素差×该量 得到 网格下标
	      mfGridElementWidthInv=static_cast<float>(FRAME_GRID_COLS)/static_cast<float>(mnMaxX-mnMinX);
	      mfGridElementHeightInv=static_cast<float>(FRAME_GRID_ROWS)/static_cast<float>(mnMaxY-mnMinY);

	      fx = K.at<float>(0,0);
	      fy = K.at<float>(1,1);
	      cx = K.at<float>(0,2);
	      cy = K.at<float>(1,2);
	      invfx = 1.0f/fx;
	      invfy = 1.0f/fy;

	      mbInitialComputations=false;
	  }

	  mb = mbf/fx;// 双目相机基线长度
	
	// 按照 特征点 的像素坐标  分配到各个网格内
	// 每个网格记录了 特征点的 序列下标
	// 最后将关键点分布到64*48分割而成的网格中（目的是加速匹配以及均匀化关键点分布）
	  AssignFeaturesToGrid();
}



      /**
      * @brief  单目相机 初始化帧  
      * @param mGray		 灰度图
      * 
      */
      Frame::Frame(const cv::Mat &imGray, const double &timeStamp, ORBextractor* extractor,ORBVocabulary* voc, cv::Mat &K, cv::Mat &distCoef, const float &bf, const float &thDepth)
	  :mpORBvocabulary(voc),mpORBextractorLeft(extractor),mpORBextractorRight(static_cast<ORBextractor*>(NULL)),
	  mTimeStamp(timeStamp), mK(K.clone()),mDistCoef(distCoef.clone()), mbf(bf), mThDepth(thDepth)
      {
	  // Frame ID
	  mnId=nNextId++;

	  // Scale Level Info
	  // 特征点匹配 图像金字塔参数
	  mnScaleLevels = mpORBextractorLeft->GetLevels();
	  mfScaleFactor = mpORBextractorLeft->GetScaleFactor();
	  mfLogScaleFactor = log(mfScaleFactor);
	  mvScaleFactors = mpORBextractorLeft->GetScaleFactors();
	  mvInvScaleFactors = mpORBextractorLeft->GetInverseScaleFactors();
	  mvLevelSigma2 = mpORBextractorLeft->GetScaleSigmaSquares();
	  mvInvLevelSigma2 = mpORBextractorLeft->GetInverseScaleSigmaSquares();

	  // ORB extraction
	  // 图像 ORB特征提取   未校正的图像  得到 关键点位置后 直接对 关键点坐标进行校正
	  ExtractORB(0,imGray);

	  N = mvKeys.size();// 特征点数量

	  if(mvKeys.empty())
	      return;

	  // 对关键点坐标进行校正    关键点   mvKeys -----> 畸变校正------>  mvKeysUn 
	  UndistortKeyPoints();

	  // Set no stereo information
	  // 初始化 匹配点 横坐标  和对应特征点的深度  单目一开始算不出来 深度 和 匹配点
	  // 但是不包含匹配信息
	  mvuRight = vector<float>(N,-1);//匹配点 横坐标 无 为-1
	  mvDepth = vector<float>(N,-1);//匹配点深度 无 为-1
	  
	  // 地图点
	  mvpMapPoints = vector<MapPoint*>(N,static_cast<MapPoint*>(NULL));
	  mvbOutlier = vector<bool>(N,false);

	  // This is done only for the first Frame (or after a change in the calibration)
	  if(mbInitialComputations)//第一帧 进行计算
	  {
	    // 对于未校正的图像 计算校正后 图像的 尺寸 mnMinX  , mnMaxX   mnMinY,  mnMaxY
	      ComputeImageBounds(imGray);
	    // 640*480 图像 分成 64*48 个网格
	    // 计算每个 像素 占据的 网格量   像素差×该量 得到 网格下标
	      mfGridElementWidthInv=static_cast<float>(FRAME_GRID_COLS)/static_cast<float>(mnMaxX-mnMinX);
	      mfGridElementHeightInv=static_cast<float>(FRAME_GRID_ROWS)/static_cast<float>(mnMaxY-mnMinY);

	      fx = K.at<float>(0,0);
	      fy = K.at<float>(1,1);
	      cx = K.at<float>(0,2);
	      cy = K.at<float>(1,2);
	      invfx = 1.0f/fx;
	      invfy = 1.0f/fy;

	      mbInitialComputations=false;
	  }

	  mb = mbf/fx;// 双目相机基线长度
	
	// 按照 特征点 的像素坐标  分配到各个网格内
	// 每个网格记录了 特征点的 序列下标
	// 最后将关键点分布到64*48分割而成的网格中（目的是加速匹配以及均匀化关键点分布）
	  AssignFeaturesToGrid();
      }
  
  
/**
 * @brief        关键点按网格分配 来加速 匹配
 * 640 *480 的图像  分成 10 64*48 个网格  总关键点 个数 N  每个网格 分到的 关键点个数
 */    
      void Frame::AssignFeaturesToGrid()
      {
	  int nReserve = 0.5f*N/(FRAME_GRID_COLS*FRAME_GRID_ROWS);
	  for(unsigned int i=0; i<FRAME_GRID_COLS;i++)
	      for (unsigned int j=0; j<FRAME_GRID_ROWS;j++)
		  mGrid[i][j].reserve(nReserve);
//每个小网格容器 又变成 分关键点个数个大小的 子容器 可以动态调整大小

	  for(int i=0;i<N;i++)
	  {
	      const cv::KeyPoint &kp = mvKeysUn[i];//每一个 校正后的关键点

	      int nGridPosX, nGridPosY;
	      if(PosInGrid(kp,nGridPosX,nGridPosY))
		  mGrid[nGridPosX][nGridPosY].push_back(i);//网格容器内 填充关键点的 序列  动态调整大小
	  }
      }

/* 
关键点提取 + 描述子
*/
      void Frame::ExtractORB(int flag, const cv::Mat &im)
      {
	  if(flag==0)
	      (*mpORBextractorLeft)(im,cv::Mat(),mvKeys,mDescriptors);//左图提取器 提取 关键点 和描述子
	  else
	      (*mpORBextractorRight)(im,cv::Mat(),mvKeysRight,mDescriptorsRight);// 右图提取器  提取 关键点 和描述子
      }

/**
 * @brief Set the camera pose.
 * 
 * 设置相机姿态，随后会调用 UpdatePoseMatrices() 来改变mRcw,mRwc等变量的值
 * @param Tcw Transformation from world to camera
 */
      void Frame::SetPose(cv::Mat Tcw)
      {
	  mTcw = Tcw.clone();// 设置位姿 变换矩阵  到 类内 变量
	  // Tcw_.copyTo(mTcw);// 拷贝到 类内变量 w2c
	  UpdatePoseMatrices();// 更新旋转矩阵 平移向量 世界中心点
      }
 
/**
 * @brief Computes rotation, translation and camera center matrices from the camera pose.
 *
 * 根据Tcw计算mRcw、mtcw和mRwc、mOw
 */     
      void Frame::UpdatePoseMatrices()
      { 
	 // [x_camera 1] = [R|t]*[x_world 1]，坐标为齐次形式
         // x_camera = R*x_world + t
	  mRcw = mTcw.rowRange(0,3).colRange(0,3);// 世界 到 相机 旋转矩阵
	  mRwc = mRcw.t();// t() 逆矩阵                            // 相机 到 世界  旋转矩阵
	  mtcw = mTcw.rowRange(0,3).col(3);                // 世界 到 相机 平移向量
	  // mtcw, 即相机坐标系下相机坐标系到世界坐标系间的向量, 向量方向由相机坐标系指向世界坐标系
	  // mOw, 即世界坐标系下世界坐标系到相机坐标系间的向量, 向量方向由世界坐标系指向相机坐标系    
	  // mtwc  = - mtcw // 相机到 世界 位置  I*mtwc
	  mOw = -mRwc*mtcw;// 相机中心点在世界坐标系坐标  相机00点--->mRwc------>mtwc--------
      }
      
/**
 * @brief 判断一个点是否在视野内
 * 检查地图点 是否在 当前视野中
 * 相机坐标系下 点 深度小于0 点不在视野中
 * 像素坐标系下 点 横纵坐标在 校正后的图像尺寸内
 * 计算了重投影坐标，观测方向夹角，预测在当前帧的尺度
 * @param  pMP             MapPoint
 * @param  viewingCosLimit 视角和平均视角的方向阈值
 * @return                 true if is in view
 * @see SearchLocalPoints()
 */
      bool Frame::isInFrustum(MapPoint *pMP, float viewingCosLimit)
      {
	  pMP->mbTrackInView = false;// 初始 设置为 不在视野内

	  // 3D in absolute coordinates
	  cv::Mat P = pMP->GetWorldPos();// 世界坐标系下的点

	  // 3D in camera coordinates
	  // 3D点转换到 相机坐标系下
	  const cv::Mat Pc = mRcw*P+mtcw;  // 世界坐标系 转到 相机坐标系
	  const float &PcX = Pc.at<float>(0);
	  const float &PcY= Pc.at<float>(1);
	  const float &PcZ = Pc.at<float>(2);

	  // Check positive depth
	  if(PcZ<0.0f)// 深度为 负  错误
	      return false;

	  // Project in image and check it is not outside
	  // 相机像素坐标系下的点   应该在 校正的图像内
	  const float invz = 1.0f/PcZ;
	  const float u=fx*PcX*invz+cx;
	  const float v=fy*PcY*invz+cy;
	  if(u<mnMinX || u>mnMaxX)
	      return false;
	  if(v<mnMinY || v>mnMaxY)
	      return false;

	  // Check distance is in the scale invariance region of the MapPoint
	  // 每一个地图点都是对应于若干尺度的金字塔提取出来的，具有一定的有效深度，
	// 如果相对当前帧的深度超过此范围，返回False
	// V-D 3) 计算MapPoint到相机中心的距离, 并判断是否在尺度变化的距离内
	  const float maxDistance = pMP->GetMaxDistanceInvariance();
	  const float minDistance = pMP->GetMinDistanceInvariance();
	  // 世界坐标系下，相机到3D点P的向量, 向量方向由相机指向3D点P
	  const cv::Mat PO = P-mOw;// 世界坐标系下，相机到3D点P的向量, 向量方向由相机指向3D点P
	  const float dist = cv::norm(PO);
	  if(dist<minDistance || dist>maxDistance)
	      return false;

	// Check viewing angle
	  // 每一个地图点都有其平均视角，是从能够观测到地图点的帧位姿中计算出
	  // 如果当前帧的视角和其平均视角相差太大，返回False
	   // V-D 2) 计算当前视角和平均视角夹角的余弦值, 若小于cos(60), 即夹角大于60度则返回
	  cv::Mat Pn = pMP->GetNormal();// |Pn|=1
	  const float viewCos = PO.dot(Pn)/dist;//  = P0.dot(Pn)/(|P0|*|Pn|); |P0|=dist 
	  if(viewCos < viewingCosLimit)
	      return false;

	  // Predict scale in the image
	  // 根据深度预测尺度（对应特征点在一层）
	  // V-D 4) 根据深度预测尺度（对应特征点在一层）
	  const int nPredictedLevel = pMP->PredictScale(dist,this);

	  // Data used by the tracking
	  //  标记该点将来要被投影
	  pMP->mbTrackInView = true;//在视野中
	  pMP->mTrackProjX = u;// 投影点 像素横坐标
	  pMP->mTrackProjXR = u - mbf*invz;// 双目右侧相机 匹配点 像素 横坐标 = 投影点 像素横坐标 - 视差 = 投影点 像素横坐标 - bf / 深度
	  pMP->mTrackProjY = v;// 投影点 像素 纵坐标
	  pMP->mnTrackScaleLevel= nPredictedLevel;
	  pMP->mTrackViewCos = viewCos;

	  return true;
      }
      
/**
 * @brief 找到在 以x,y为中心,边长为2r的方形内且在[minLevel, maxLevel]的特征点
 * 在某块区域内获取特帧点
 * 其中，minLevel和maxLevel考察特征点是从图像金字塔的哪一层提取出来的。
 * @param x        图像坐标u
 * @param y        图像坐标v
 * @param r        边长
 * @param minLevel 最小尺度
 * @param maxLevel 最大尺度
 * @return         满足条件的特征点的序号
 */

      vector<size_t> Frame::GetFeaturesInArea(
              const float &x, const float  &y, 
              const float  &r,
              const int minLevel, const int maxLevel) const
      {
	  vector<size_t> vIndices;
	  vIndices.reserve(N);

	  const int nMinCellX = max(0,(int)floor((x-mnMinX-r)*mfGridElementWidthInv));
	  if(nMinCellX>=FRAME_GRID_COLS)
	      return vIndices;

	  const int nMaxCellX = min((int)FRAME_GRID_COLS-1,(int)ceil((x-mnMinX+r)*mfGridElementWidthInv));
	  if(nMaxCellX<0)
	      return vIndices;

	  const int nMinCellY = max(0,(int)floor((y-mnMinY-r)*mfGridElementHeightInv));
	  if(nMinCellY>=FRAME_GRID_ROWS)
	      return vIndices;

	  const int nMaxCellY = min((int)FRAME_GRID_ROWS-1,(int)ceil((y-mnMinY+r)*mfGridElementHeightInv));
	  if(nMaxCellY<0)
	      return vIndices;

	  const bool bCheckLevels = (minLevel>0) || (maxLevel>=0);

	  for(int ix = nMinCellX; ix<=nMaxCellX; ix++)
	  {
	      for(int iy = nMinCellY; iy<=nMaxCellY; iy++)
	      {
		  const vector<size_t> vCell = mGrid[ix][iy];
		  if(vCell.empty())
		      continue;

		  for(size_t j=0, jend=vCell.size(); j<jend; j++)
		  {
		      const cv::KeyPoint &kpUn = mvKeysUn[vCell[j]];
		      if(bCheckLevels)
		      {
			  if(kpUn.octave<minLevel)
			      continue;
			  if(maxLevel>=0)
			      if(kpUn.octave>maxLevel)
				  continue;
		      }

		      const float distx = kpUn.pt.x-x;
		      const float disty = kpUn.pt.y-y;

		      if(fabs(distx)<r && fabs(disty)<r)
			  vIndices.push_back(vCell[j]);
		  }
	      }
	  }

	  return vIndices;
      }


  /*
检查点是否在 某个划分的格子内 
*/

      bool Frame::PosInGrid(const cv::KeyPoint &kp, int &posX, int &posY)
      {
	// 被 划分到的 格子坐标
	  posX = round((kp.pt.x-mnMinX)*mfGridElementWidthInv);
	  posY = round((kp.pt.y-mnMinY)*mfGridElementHeightInv);
	  //Keypoint's coordinates are undistorted, which could cause to go out of the image
	  if(posX<0 || posX>=FRAME_GRID_COLS || posY<0 || posY>=FRAME_GRID_ROWS)
	      return false;// 不在 某个格子里
	  return true;//在某个格子里
      }
      

/**
 * @brief Bag of Words Representation
 *
 * 用单词(ORB单词词典) 线性表示 一帧所有描述子  相当于  一个句子 用几个单词 来表示
 * 词典 N个M维的单词
 * 一帧的描述子  n个M维的描述子
 * 生成一个 N*1的向量 记录一帧的描述子 使用词典单词的情况
 * 4. 将当前帧的描述子矩阵（可以转换成向量），转换成词袋模型向量
 * （DBoW2::BowVector mBowVec； DBoW2::FeatureVector mFeatVec；）：
 * 计算词包mBowVec和mFeatVec，其中mFeatVec记录了属于第i个node（在第4层）的ni个描述子
 * @see CreateInitialMapMonocular() TrackReferenceKeyFrame() Relocalization()
 */
      void Frame::ComputeBoW()
      {
	  if(mBowVec.empty())//词典表示向量为空
	  {
	      vector<cv::Mat> vCurrentDesc = Converter::toDescriptorVector(mDescriptors);//mat类型转换到 vector类型描述子向量
	      // Feature vector associate features with nodes in the 4th level (from leaves up)
	      // We assume the vocabulary tree has 6 levels, change the 4 otherwise
	      mpORBvocabulary->transform(vCurrentDesc,mBowVec,mFeatVec,4);// 计算 描述子向量 用词典线性表示的向量
	  }
      }

      // 对元素图像中的点 利用 畸变 参数进行校正 得到校正后的坐标 
      // 不是对 整个图像进行校正(时间长)
      void Frame::UndistortKeyPoints()
      {
	  if(mDistCoef.at<float>(0)==0.0)// 畸变校正参数没有 
	  {
	      mvKeysUn=mvKeys;//校正后的点 和 未校正 的点坐标 二位像素坐标
	      return;
	  }
	  // Fill matrix with points
	  // 关键点坐标（未校正） 转成 opencv mat类型
	  cv::Mat mat(N,2,CV_32F);
	  for(int i=0; i<N; i++)
	  {
	      mat.at<float>(i,0)=mvKeys[i].pt.x;
	      mat.at<float>(i,1)=mvKeys[i].pt.y;
	  }

	  // 校正关键点坐标 Undistort points
	// 函数的功能：将一些 来自 原始图像的2维点坐标进行校正 得到  相应的矫正点坐标。
	  mat=mat.reshape(2);
	  cv::undistortPoints(mat,mat,mK,mDistCoef,cv::Mat(),mK);
	  mat=mat.reshape(1);

	  // Fill undistorted keypoint vector 填充校正后的坐标
	  mvKeysUn.resize(N);
	  for(int i=0; i<N; i++)
	  {
	      cv::KeyPoint kp = mvKeys[i];
	      kp.pt.x=mat.at<float>(i,0);
	      kp.pt.y=mat.at<float>(i,1);
	      mvKeysUn[i]=kp;
	  }
      }

      // 对于未校正的图像 计算校正后 图像的 尺寸
      void Frame::ComputeImageBounds(const cv::Mat &imLeft)
      {
	  if(mDistCoef.at<float>(0)!=0.0)// 畸变校正参数 有
	  {
	      cv::Mat mat(4,2,CV_32F);
	      // 图像四个顶点位置
	      // (0,0)  (col,0)  (0,row)   (col,row)
	      mat.at<float>(0,0)=0.0; mat.at<float>(0,1)=0.0;
	      mat.at<float>(1,0)=imLeft.cols; mat.at<float>(1,1)=0.0;
	      mat.at<float>(2,0)=0.0; mat.at<float>(2,1)=imLeft.rows;
	      mat.at<float>(3,0)=imLeft.cols; mat.at<float>(3,1)=imLeft.rows;

	      // Undistort corners
	      // 对图像四个点进行 畸变校正
	      mat=mat.reshape(2);
	      cv::undistortPoints(mat,mat,mK,mDistCoef,cv::Mat(),mK);
	      mat=mat.reshape(1);

	      mnMinX = min(mat.at<float>(0,0),mat.at<float>(2,0));
	      mnMaxX = max(mat.at<float>(1,0),mat.at<float>(3,0));
	      mnMinY = min(mat.at<float>(0,1),mat.at<float>(1,1));
	      mnMaxY = max(mat.at<float>(2,1),mat.at<float>(3,1));

	  }
	  else // 无畸变校正参数  也就是校正后的图像 图像大小不变
	  {
	      mnMinX = 0.0f;
	      mnMaxX = imLeft.cols;
	      mnMinY = 0.0f;
	      mnMaxY = imLeft.rows;
	  }
      }
      
/**
 * @brief 双目匹配   双目匹配  特征点对匹配
 *
 * 为左图的每一个特征点在右图中找到匹配点 \n
 * 根据基线(有冗余范围)上描述子距离找到匹配, 再进行SAD精确定位 \n
 * 最后对所有SAD的值进行排序, 剔除SAD值较大的匹配对，然后利用抛物线拟合得到亚像素精度的匹配 \n
 * 匹配成功后会更新 mvuRight(ur) 和 mvDepth(Z)
 */
      /*
      * 1】为左目每个特征点建立带状区域搜索表，限定搜索区域，（前已进行极线校正）
      * 2】在限定区域内 通过描述子进行 特征点匹配，得到每个特征点最佳匹配点（scaleduR0）   bestIdxR  uR0 = mvKeysRight[bestIdxR].pt.x;   scaleduR0 = round(uR0*scaleFactor);
      * 3】通过SAD滑窗得到匹配修正量 bestincR
      * 4】(bestincR, dist)  (bestincR - 1, dist)  (bestincR +1, dist) 三点拟合出抛物线，得到亚像素修正量 deltaR
      剔除SAD匹配偏差较大的匹配特征点。
      * 5】最终匹配点位置 为 : scaleduR0 + bestincR  + deltaR
      */
      void Frame::ComputeStereoMatches()
      {
	  mvuRight = vector<float>(N,-1.0f);// 左图关键点 对应 右图匹配点
	  mvDepth = vector<float>(N,-1.0f);// 关键点对于的深度

	  const int thOrbDist = (ORBmatcher::TH_HIGH+ORBmatcher::TH_LOW)/2;// 匹配距离

	  const int nRows = mpORBextractorLeft->mvImagePyramid[0].rows;

    //Assign keypoints to row table
// 步骤1：建立特征点搜索范围对应表，一个特征点在一个带状区域内搜索匹配特征点
    // 匹配搜索的时候，不仅仅是在一条横线上搜索，而是在一条横向搜索带上搜索,简而言之，原本每个特征点的纵坐标为1，这里把特征点体积放大，纵坐标占好几行
    // 例如左目图像某个特征点的纵坐标为20，那么在右侧图像上搜索时是在纵坐标为18到22这条带上搜索，搜索带宽度为正负2，搜索带的宽度和特征点所在金字塔层数有关
    // 简单来说，如果纵坐标是20，特征点在图像第20行，那么认为18 19 20 21 22行都有这个特征点
    // vRowIndices[18]、vRowIndices[19]、vRowIndices[20]、vRowIndices[21]、vRowIndices[22]都有这个特征点编号
	  vector<vector<size_t> > vRowIndices(nRows,vector<size_t>());

	  for(int i=0; i<nRows; i++)
	      vRowIndices[i].reserve(200);

	  const int Nr = mvKeysRight.size();// 右图关键点数量

	  // 生成 右图 关键点 匹配块 限制 搜索 带状 区域
	  for(int iR=0; iR<Nr; iR++)
	  {
	    // !!在这个函数中没有对双目进行校正，双目校正是在外层程序中实现的
	      const cv::KeyPoint &kp = mvKeysRight[iR];
	      const float &kpY = kp.pt.y;
        // 计算匹配搜索的纵向宽度，尺度越大（层数越高，距离越近），搜索范围越大
        // 如果特征点在金字塔第一层，则搜索范围为:正负2
        // 尺度越大其位置不确定性越高，所以其搜索半径越大	      
	      const float r = 2.0f*mvScaleFactors[mvKeysRight[iR].octave];
	      const int maxr = ceil(kpY+r);
	      const int minr = floor(kpY-r);

	      for(int yi=minr;yi<=maxr;yi++)
		  vRowIndices[yi].push_back(iR);// 右图 带状区域
	  }

	  // Set limits for search
	  const float minZ = mb;        // NOTE bug mb没有初始化，mb的赋值在构造函数中放在ComputeStereoMatches函数的后面
	  const float minD = 0;        // 最小视差, 设置为0即可
	  const float maxD = mbf/minZ;  // 最大视差, 对应最小深度 mbf/minZ = mbf/mb = mbf/(mbf/fx) = fx (wubo???)


	  // For each left keypoint search a match in the right image
	  // 在右图限定区域内为 左图 关键点 匹配一个 关键点
	  vector<pair<int, int> > vDistIdx;
	  vDistIdx.reserve(N);
	  
// 步骤2：对左目相机每个特征点，通过描述子在右目带状搜索区域找到匹配点, 再通过SAD做亚像素匹配
    // 注意：这里是校正前的mvKeys，而不是校正后的mvKeysUn
    // KeyFrame::UnprojectStereo和Frame::UnprojectStereo函数中不一致
    // 这里是不是应该对校正后特征点求深度呢？(wubo???)
	  for(int iL=0; iL<N; iL++)// 每一个 左图关键点 和 限定区域 右图关键点 进行描述子匹配 找到 距离最近的 匹配点  + 修正量
	  {
	    // 创建引用
	      const cv::KeyPoint &kpL = mvKeys[iL];
	      const int &levelL = kpL.octave;
	      const float &vL = kpL.pt.y;
	      const float &uL = kpL.pt.x;

	      const vector<size_t> &vCandidates = vRowIndices[vL];// 关键点匹配 候选区域

	      if(vCandidates.empty())
		  continue;

	      const float minU = uL-maxD;
	      const float maxU = uL-minD;

	      if(maxU<0)
		  continue;

	      int bestDist = ORBmatcher::TH_HIGH;
	      size_t bestIdxR = 0;

	      const cv::Mat &dL = mDescriptors.row(iL);//左图关键点 描述子
  // 步骤2.1：遍历右目所有可能的匹配点，找出最佳匹配点（描述子距离最小）
	      // Compare descriptor to right keypoints  和右图 关键点 描述子进行比较
	      for(size_t iC=0; iC<vCandidates.size(); iC++)// 每一个 右图 候选区域 关键点
	      {
		  const size_t iR = vCandidates[iC];
		  const cv::KeyPoint &kpR = mvKeysRight[iR];//右图关键点

		  if(kpR.octave<levelL-1 || kpR.octave>levelL+1)
		      continue;

		  const float &uR = kpR.pt.x;

		  if(uR>=minU && uR<=maxU)
		  {
		      const cv::Mat &dR = mDescriptorsRight.row(iR);// 右图  关键点描述子
		      const int dist = ORBmatcher::DescriptorDistance(dL,dR);// 二进制描述子 相似度距离   汉明距离

		      if(dist<bestDist)
		      {
			  bestDist = dist;//保存最小的 匹配距离
			  bestIdxR = iR;// 最小匹配距离对应的 右图点 bestIdxR
		      }
		  }
	      }
	      
   // 步骤2.2：通过SAD匹配提高像素匹配修正量bestincR
	      // Subpixel match by correlation
	      // 亚像素修正量
	      if(bestDist<thOrbDist)// 匹配距离较小
	      {
		  // coordinates in image pyramid at keypoint scale
		// 图像金字塔中的 匹配点坐标 
		// kpL.pt.x对应金字塔最底层坐标，将最佳匹配的特征点对尺度变换到尺度对应层 (scaleduL, scaledvL) (scaleduR0, )
		  const float uR0 = mvKeysRight[bestIdxR].pt.x;
		  const float scaleFactor = mvInvScaleFactors[kpL.octave];
		  const float scaleduL = round(kpL.pt.x*scaleFactor);
		  const float scaledvL = round(kpL.pt.y*scaleFactor);
		  const float scaleduR0 = round(uR0*scaleFactor);

		  // sliding window search
		  // 滑窗方法
		  const int w = 5;
		  cv::Mat IL = mpORBextractorLeft->mvImagePyramid[kpL.octave].rowRange(scaledvL-w,scaledvL+w+1).colRange(scaleduL-w,scaleduL+w+1);
		  IL.convertTo(IL,CV_32F);
		  IL = IL - IL.at<float>(w,w) *cv::Mat::ones(IL.rows,IL.cols,CV_32F);

		  int bestDist = INT_MAX;
		  int bestincR = 0;
		  const int L = 5;// 10*10窗口
		  vector<float> vDists;
		  vDists.resize(2*L+1);
                 // 滑动窗口的滑动范围为（-L, L）,提前判断滑动窗口滑动过程中是否会越界
		  const float iniu = scaleduR0+L-w;
		  const float endu = scaleduR0+L+w+1;
		  if(iniu<0 || endu >= mpORBextractorRight->mvImagePyramid[kpL.octave].cols)
		      continue;

		  for(int incR=-L; incR<=+L; incR++)
		  {
		      cv::Mat IR = mpORBextractorRight->mvImagePyramid[kpL.octave].rowRange(scaledvL-w,scaledvL+w+1).colRange(scaleduR0+incR-w,scaleduR0+incR+w+1);
		      IR.convertTo(IR,CV_32F);
		      IR = IR - IR.at<float>(w,w) *cv::Mat::ones(IR.rows,IR.cols,CV_32F);//窗口中的每个元素减去正中心的那个元素，简单归一化，减小光照强度影响

		      float dist = cv::norm(IL,IR,cv::NORM_L1);// 一范数，计算差的绝对值
		      if(dist<bestDist)
		      {
                         bestDist =  dist;// SAD匹配目前最小匹配偏差
                         bestincR = incR; // SAD匹配目前最佳的修正量
		      }

		      vDists[L+incR] = dist;// 距离
		  }

		  if(bestincR==-L || bestincR==L)
		      continue;
       // 步骤2.3：做抛物线拟合找谷底得到亚像素匹配deltaR
            // (bestincR,dist) (bestincR-1,dist) (bestincR+1,dist)三个点拟合出抛物线
            // bestincR+deltaR就是抛物线谷底的位置，相对SAD匹配出的最小值bestincR的修正量为deltaR
		  // Sub-pixel match (Parabola fitting)
		  const float dist1 = vDists[L+bestincR-1];
		  const float dist2 = vDists[L+bestincR];
		  const float dist3 = vDists[L+bestincR+1];

		  const float deltaR = (dist1-dist3)/(2.0f*(dist1+dist3-2.0f*dist2));
                   // 抛物线拟合得到的修正量不能超过一个像素，否则放弃求该特征点的深度
		  if(deltaR<-1 || deltaR>1)
		      continue;

		  // Re-scaled coordinate
		  // 最终 右图 匹配点 横坐标   带状块区域 描述子匹配 坐标 + 滑窗校正  + 抛物线拟合校正
		  float bestuR = mvScaleFactors[kpL.octave]*((float)scaleduR0+(float)bestincR+deltaR);

		  // 通过描述子匹配得到匹配点位置为scaleduR0
		  // 通过SAD匹配找到修正量bestincR
		  // 通过抛物线拟合找到亚像素修正量deltaR  
		  float disparity = (uL-bestuR);// 双目相机 匹配点对 视差

		  if(disparity>=minD && disparity<maxD)
		  {
		      if(disparity<=0)
		      {
			  disparity=0.01;
			  bestuR = uL-0.01;
		      }
		      mvDepth[iL]=mbf/disparity; //根据视差得到深度信息    z = bf /d      b 双目相机基线长度  f为焦距  d为视差(同一点在两相机像素平面 水平方向像素单位差值)
		      mvuRight[iL] = bestuR;
		      vDistIdx.push_back(pair<int,int>(bestDist,iL));
		  }
	      }
	  }
      // 以上 计算了 所有 关键点的匹配点对(以及最佳匹配距离) 和 对应的深度
// 步骤3：剔除SAD匹配偏差较大的匹配特征点
    // 前面SAD匹配只判断滑动窗口中是否有局部最小值，这里通过对比剔除SAD匹配偏差比较大的特征点的深度
	  sort(vDistIdx.begin(),vDistIdx.end());// 所有关键点 匹配距离排序  根据所有匹配对的SAD偏差进行排序, 距离由小到大
	  const float median = vDistIdx[vDistIdx.size()/2].first;// 距离中值
	  const float thDist = 1.5f*1.4f*median; // 计算自适应距离, 大于此距离的匹配对将剔除

	  for(int i=vDistIdx.size()-1;i>=0;i--)
	  {
	      if(vDistIdx[i].first<thDist)
		  break;
	      else // 距离过大的匹配点对 任务是误匹配  设置为 -1 用以标记
	      {
		  mvuRight[vDistIdx[i].second]=-1;
		  mvDepth[vDistIdx[i].second]=-1;
	      }
	  }
      }

      // 深度相机 计算  深度值 根据未校正的关键点 在深度图中的值 获得
      // 匹配点横坐标 有原特征点校正的后横坐标 -  视差；    视差 = bf / 深度
      void Frame::ComputeStereoFromRGBD(const cv::Mat &imDepth)
      {
	  mvuRight = vector<float>(N,-1);// 初始匹配点
	  mvDepth = vector<float>(N,-1);// 初始深度

	  for(int i=0; i<N; i++)
	  {
	      const cv::KeyPoint &kp = mvKeys[i];// 关键点 未校正  用于从深度图 获取深度值
	      const cv::KeyPoint &kpU = mvKeysUn[i];//校正后的 关键点

	      const float &v = kp.pt.y;
	      const float &u = kp.pt.x;

	      const float d = imDepth.at<float>(v,u);// 对应的深度值 

	      if(d>0)
	      {
		  mvDepth[i] = d;
		  mvuRight[i] = kpU.pt.x-mbf/d;// 深度 = bf / 视差  ---> 视差 = bf / 深度  ----> 原校正后的坐标 - 视差 得到匹配点 x方向坐标值
	      }
	  }
      }
      
/**
 * @brief Backprojects a keypoint (if stereo/depth info available) into 3D world coordinates.
 * @param  i 第i个keypoint
 * @return   3D点（相对于世界坐标系）
 */
      // 得到 世界坐标系 坐标
      cv::Mat Frame::UnprojectStereo(const int &i)
      {
	  // mvDepth是在ComputeStereoMatches函数中求取的
          // mvDepth对应的校正前的特征点，可这里却是对校正后特征点反投影
          // KeyFrame::UnprojectStereo中是对校正前的特征点mvKeys反投影
          // 在ComputeStereoMatches函数中应该对校正后的特征点求深度？？ (wubo???)
	  const float z = mvDepth[i];// 深度值
	  if(z>0)
	  {
	    // 像素坐标值
	      const float u = mvKeysUn[i].pt.x;
	      const float v = mvKeysUn[i].pt.y;
	    // 像极坐标系 坐标
	      const float x = (u-cx)*z*invfx;
	      const float y = (v-cy)*z*invfy;
	      cv::Mat x3Dc = (cv::Mat_<float>(3,1) << x, y, z);
	      return mRwc*x3Dc+mOw;
	  }
	  else
	      return cv::Mat();
      }

} //namespace ORB_SLAM
