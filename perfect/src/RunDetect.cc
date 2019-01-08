/* This file is part of ORB-SLAM2-SSD-Semantic.
* 2d目标检测
*/
#include "RunDetect.h"

RunDetect::RunDetect()
{
   mDetector = new(Detector);
   mvKeyframes.clear();// 关键帧数组清空===
   colorImgs.clear();  // 彩色图像====
   mRunThread = make_shared<thread>( bind(&RunDetect::Run, this ) );// 可视化线程 共享指针 绑定 RunDetect::Run()函数 
}
RunDetect::~RunDetect()
{
   delete mDetector;
}
// 地图中插入一个关键帧=======tracker的构建关键帧函数执行===================
void RunDetect::insertKFColorImg(KeyFrame* kf, cv::Mat color)
{
    cout<<"receive a keyframe, id = "<<kf->mnId<<endl;
    unique_lock<mutex> lck(colorImgMutex); // 对关键帧上锁
    colorImgs.push_back( color.clone() );  // 图像数组  加入一个 图像  深拷贝
    mvKeyframes.push_back(kf);

    colorImgUpdated.notify_one();         
    // 数据更新线程 解除一个阻塞的，来对彩色图像进行 目标检测===
}

void RunDetect::Run(void)
{
 while(1)
 {
    {
       unique_lock<mutex> lck_colorImgUpdated( colorImgMutex); // 关键帧更新锁
       colorImgUpdated.wait( lck_colorImgUpdated );// 阻塞 关键帧更新锁
       // 需要等待 insertKeyFrame() 函数中完成 添加 关键帧 后，执行后面的!!
    }
    // 彩色图像===
    size_t N=0;
    {
       unique_lock<mutex> lck( colorImgMutex );// 关键帧锁
       N = colorImgs.size();                   // 当前 保存的 关键帧数量
    }  
    for ( size_t i=lastKeyframeSize; i<N ; i++ )// 接着上次处理的地方开始
    {
       std::vector<Object> vobject;
       mDetector->Run(colorImgs[i], vobject);
       if(vobject.size()>0)
       {  
           std::cout << "detect : " << vobject.size() << " uums obj" << std::endl;
           for(unsigned int j =0; j<vobject.size(); j++)
           {
               unique_lock<mutex> lckObj(mvKeyframesMutex); // 2d检测结果上锁
               mvKeyframes[i]->mvObject.push_back(vobject[j]);// 存入目标检测结果库====
           }
       }
    }

    lastKeyframeSize = N;
 }
    
}
