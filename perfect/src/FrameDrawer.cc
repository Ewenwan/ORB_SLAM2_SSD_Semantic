/**
* This file is part of ORB-SLAM2.
* 获取帧 显示 图像+关键点====
*/

#include "FrameDrawer.h"
#include "Tracking.h"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include<mutex>

#include "MapDrawer.h"

namespace ORB_SLAM2
{

FrameDrawer::FrameDrawer(Map* pMap):mpMap(pMap)
{
    mState=Tracking::SYSTEM_NOT_READY;
    mIm = cv::Mat(480,640,CV_8UC3, cv::Scalar(0,0,0));// 初始化一个空的三通道图像
}

FrameDrawer::FrameDrawer(Map* pMap, MapDrawer* pMapDrawer, const string &strSettingPath):
                     mpMap(pMap),
                     mpMapDrawer(pMapDrawer)
{
    mState=Tracking::SYSTEM_NOT_READY;
    mIm = cv::Mat(480,640,CV_8UC3, cv::Scalar(0,0,0));// 初始化一个空的三通道图像

    cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);
    float fx = fSettings["Camera.fx"];
    float fy = fSettings["Camera.fy"];
    float cx = fSettings["Camera.cx"];
    float cy = fSettings["Camera.cy"];

    cv::Mat K = cv::Mat::eye(3,3,CV_32F);
    K.at<float>(0,0) = fx;
    K.at<float>(1,1) = fy;
    K.at<float>(0,2) = cx;
    K.at<float>(1,2) = cy;
    K.copyTo(mK);// 相机内参数======
}

cv::Mat FrameDrawer::DrawFrame()
{
    cv::Mat im,mask;
    vector<cv::KeyPoint> vIniKeys; // 初始化参考帧关键点 Initialization: KeyPoints in reference frame
    vector<int> vMatches;          // 匹配点 Initialization: correspondeces with reference keypoints
    vector<cv::KeyPoint> vCurrentKeys; // 当前帧关键点 KeyPoints in current frame
    vector<bool> vbVO, vbMap;          // 跟踪的关键点 Tracked MapPoints in current frame
                                       // vbMap 匹配到地图上一个点
                                       // vbVO    匹配到 无观察帧的地图点
    int state; // Tracking state

    //Copy variables within scoped mutex   大括号，表示数据上锁区域
    {
        unique_lock<mutex> lock(mMutex);// 对数据上锁====
        state=mState;
        if(mState==Tracking::SYSTEM_NOT_READY)
            mState=Tracking::NO_IMAGES_YET;// 切换成 没有图像==

        mIm.copyTo(im);                    // 由update函数从 tracer内拷贝过来======
        mDynMask.copyTo(mask);
        mask.cv::Mat::convertTo(mask,CV_8U);// 0～1,1处动，0处静

        if(mState==Tracking::NOT_INITIALIZED)
        {
            vCurrentKeys = mvCurrentKeys;// 类对象 复制过来
            vIniKeys = mvIniKeys;        // 初始关键帧 关键点
            vMatches = mvIniMatches;     // 初始关键帧 关键帧匹配点
        }
        else if(mState==Tracking::OK)
        {
            vCurrentKeys = mvCurrentKeys;// 当前关键帧 关键点
            vbVO = mvbVO;   // 跟踪到的
            vbMap = mvbMap;
        }
        else if(mState==Tracking::LOST)// 跟丢了，关键点就没有匹配上===
        {
            vCurrentKeys = mvCurrentKeys;// 只有 当前帧 检测到的关键点
        }
    } // destroy scoped mutex -> release mutex

    if(im.channels()<3) //this should be always true
        cvtColor(im,im,CV_GRAY2BGR);// 变成三通道 可显示彩色===

    //Draw
    if(state==Tracking::NOT_INITIALIZED) //INITIALIZING=====初始化====
    {
        for(unsigned int i=0; i<vMatches.size(); i++)
        {
            if(vMatches[i]>=0)
            {
                cv::line(im,vIniKeys[i].pt,vCurrentKeys[vMatches[i]].pt,
                        cv::Scalar(0,255,0));
            }
        }        
    }
    else if(state==Tracking::OK) //TRACKING  跟踪=====
    {
        mnTracked=0;
        mnTrackedVO=0;
        const float r = 5;
        //const int n = vCurrentKeys.size();
        for(int i=0;i<N;i++)// Update()函数中已经获取到===
        {
            if(vbVO[i] || vbMap[i]) // 跟踪到 的关键点=====
            {
                cv::Point2f pt1,pt2;
                pt1.x=vCurrentKeys[i].pt.x-r;// 左上方点
                pt1.y=vCurrentKeys[i].pt.y-r;
                pt2.x=vCurrentKeys[i].pt.x+r;// 右下方点
                pt2.y=vCurrentKeys[i].pt.y+r;

                // This is a match to a MapPoint in the map  匹配到地图上一个点=====
                if(vbMap[i])
                {
                    cv::rectangle(im,pt1,pt2,cv::Scalar(0,255,0));// bgr 绿色  正方形
                    cv::circle(im,vCurrentKeys[i].pt,2,cv::Scalar(0,255,0),-1);// 内画圆
                    mnTracked++; // 跟踪到的地图点数量====
                }
                else // 跟踪到的上一帧创建的 视觉里程记点 (部分会被删除掉)
                {
                    cv::rectangle(im,pt1,pt2,cv::Scalar(255,0,0));// bgr  蓝色===
                    cv::circle(im,vCurrentKeys[i].pt,2,cv::Scalar(255,0,0),-1);
                    mnTrackedVO++;
                }
// 显示===============光流信息==动静信息===========
            }
        }
        // cvCopy(im,im,mask);
       // im.copyTo(im, mask);
     //cv::floodFill(im, mask,  cv::Point(200,200), cv::Scalar(255 , 0, 0));
     // #mask不为0的区域不会被填充，mask为0的区域才会被填充
    cv::Scalar s = sum(mask);//各通道求和
    if(!mask.empty()&&(s.val[0] > mask.rows * mask.cols * 0.65))
       FillImage(im, mask,cv::Scalar(255 , 0, 0));
    }

    cv::Mat imWithInfo;
    DrawTextInfo(im,state, imWithInfo);// 显示文字

    return imWithInfo;
}

void FrameDrawer::FillImage(cv::Mat &im, const cv::Mat &mask, cv::Scalar scalar)
{
 unsigned char* color = (unsigned char*)im.data; //3 通道
 unsigned char* dyna = (unsigned char*)mask.data;//1 通道 动静mask

	#pragma omp parallel for   // =======================omp 多线程 并行处理
	for(int r=0; r<im.rows; r++ ) // y
	{
	 for(int c=0; c<im.cols; c++) // x
	 {
            int index = r*im.cols + c;// 总索引
            if(!dyna[index])
            {
               color[index*3+0] = scalar.val[0];
               color[index*3+1] = scalar.val[1]; 
               color[index*3+2] = scalar.val[2]; 
            }

	 }
	}
}



void FrameDrawer::generatePC(void)
{
    cv::Mat Depth,ImRGB,Tcw;
    {
     unique_lock<mutex> lock(mMutex);// 打括号区域，对数据上锁====
     mImDep.copyTo(Depth);
     mImRGB.copyTo(ImRGB);
     mTcw.copyTo(Tcw); 
    }
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    int n,m;
    for ( m=0; m < Depth.rows; m+=1 )// 每一行
    {
          for ( n=0; n < Depth.cols; n+=1 )//每一列
          {
              float d = Depth.ptr<float>(m)[n];// 深度 m为单位 保留0～2m内的点
              if (d < 0.01 || d>2.0)
                 continue;
              pcl::PointXYZRGB p;
              p.z = d;
              p.x = ( n - mK.at<float>(0,0)) * p.z / mK.at<float>(0,2);
              p.y = ( m - mK.at<float>(1,1)) * p.z / mK.at<float>(1,2);
              if(p.y<-3.0 || p.y>3.0) continue;// 保留 垂直方向 -3～3m范围内的点 
              p.b = ImRGB.ptr<uchar>(m)[n*3+0];// 点颜色=====
              p.g = ImRGB.ptr<uchar>(m)[n*3+1];
              p.r = ImRGB.ptr<uchar>(m)[n*3+2];
              cloud->points.push_back( p );
          }
    }

// 体素格滤波======
    pcl::VoxelGrid<pcl::PointXYZRGB> vg;
    vg.setInputCloud(cloud);
    vg.setLeafSize(0.01,0.01, 0.01);// 体素格子 尺寸
    vg.filter(*cloud);

// 转换到世界坐标下====
    Eigen::Isometry3d T = ORB_SLAM2::Converter::toSE3Quat( Tcw );
    pcl::PointCloud<pcl::PointXYZRGB> temp;
    pcl::transformPointCloud( *cloud, temp, T.inverse().matrix());
    mpMapDrawer->RegisterObs(temp);// 传递给 地图显示====
}

// 显示文字========================================================
void FrameDrawer::DrawTextInfo(cv::Mat &im, int nState, cv::Mat &imText)
{
    stringstream s;
    if(nState==Tracking::NO_IMAGES_YET)
        s << " WAITING FOR IMAGES";
    else if(nState==Tracking::NOT_INITIALIZED)
        s << " TRYING TO INITIALIZE ";
    else if(nState==Tracking::OK)
    {
        if(!mbOnlyTracking)
            s << "SLAM MODE |  ";// 定位+建图
        else
            s << "LOCALIZATION | ";// 定位
        int nKFs = mpMap->KeyFramesInMap();// 地图中 关键帧数量
        int nMPs = mpMap->MapPointsInMap();// 地图中 地图点数量
        s << "KFs: " << nKFs << ", MPs: " << nMPs << ", current Matches: " << mnTracked;
        if(mnTrackedVO>0)
            s << ", + current VO matches: " << mnTrackedVO;
    }
    else if(nState==Tracking::LOST)
    {
        s << " TRACK LOST. TRYING TO RELOCALIZE ";
    }
    else if(nState==Tracking::SYSTEM_NOT_READY)
    {
        s << " LOADING ORB VOCABULARY. PLEASE WAIT...";
    }

    int baseline=0;
    cv::Size textSize = cv::getTextSize(s.str(),cv::FONT_HERSHEY_PLAIN,1,1,&baseline);//文字

    imText = cv::Mat(im.rows+textSize.height+10,im.cols,im.type());// 图片扩展几行，用来显示文字=========

    im.copyTo(imText.rowRange(0,im.rows).colRange(0,im.cols));// 图像拷贝到 带文字框 的图像

    imText.rowRange(im.rows,imText.rows) = cv::Mat::zeros(textSize.height+10,im.cols,im.type());// 上次文字区域 清空

    cv::putText(imText,s.str(),cv::Point(5,imText.rows-5),cv::FONT_HERSHEY_PLAIN,1,cv::Scalar(255,255,255),1,8);// 更新字体=====

}

// 从Track对象中 更新本 类内数据======== Tracking::Track()中会执行==================
void FrameDrawer::Update(Tracking *pTracker)
{
    unique_lock<mutex> lock(mMutex);// 上锁====
    pTracker->mImGray.copyTo(mIm);// 图像，灰度图
    pTracker->mimMask.copyTo(mDynMask);// 动态点mask
    pTracker->mImDepth.copyTo(mImDep);// 深度图
    pTracker->mImRGB.copyTo(mImRGB);  // 彩色图
   // pTracker->mK.copyTo(mK);// 相机内参数===
    pTracker->mCurrentFrame.mTcw.copyTo(mTcw);

    mvCurrentKeys=pTracker->mCurrentFrame.mvKeys;// 当前帧 关键帧
    N = mvCurrentKeys.size();// 关键帧数量
    mvbVO = vector<bool>(N,false);
    mvbMap = vector<bool>(N,false);
    mbOnlyTracking = pTracker->mbOnlyTracking;// 模式


    if(pTracker->mLastProcessedState==Tracking::NOT_INITIALIZED)
    {
        mvIniKeys=pTracker->mInitialFrame.mvKeys;// 初始化关键帧 关键点
        mvIniMatches=pTracker->mvIniMatches;     // 匹配点====
    }

    else if(pTracker->mLastProcessedState==Tracking::OK)// 跟踪ok
    {
        for(int i=0;i<N;i++)
        {
            MapPoint* pMP = pTracker->mCurrentFrame.mvpMapPoints[i];//当前帧的地图点
            if(pMP)
            {
                if(!pTracker->mCurrentFrame.mvbOutlier[i])// 是否是外点
                {
                    if(pMP->Observations()>0)// 该地图点也被其他 帧观测到，是实实在在的地图点
                        mvbMap[i]=true;
                    else
                        mvbVO[i]=true;// 没有被 观测到，只存在与上一帧== 视觉里程记点=====
                }
            }
        }
    }
    mState=static_cast<int>(pTracker->mLastProcessedState);// 跟踪器状态=========
}

} //namespace ORB_SLAM
