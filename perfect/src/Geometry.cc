/**
* This file is part of DynaSLAM.
* Copyright (C) 2018 Berta Bescos <bbescos at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/bertabescos/DynaSLAM>.
*   	运动点判断 利用深度信息 区域增长扩充 结合mask-rcnn 优化 
*/

#include "Geometry.h"
#include <algorithm>
#include "Frame.h"
#include "Tracking.h"

namespace DynaSLAM
{

Geometry::Geometry()
{
    vAllPixels = cv::Mat(640*480,2,CV_32F);// 两列  存储行列id  固定尺寸值需要修改
    int m(0);
    for (int i(0); i < 640; i++){
        for (int j(0); j < 480; j++){
            vAllPixels.at<float>(m,0) = i;// 0～639 行
            vAllPixels.at<float>(m,1) = j;// 0～479列
            m++;// 行id
        }
    }
}

/*
// 融合 几何运动检测点集  和 mask分割信息============
void Geometry::GeometricModelCorrection(
                                        const ORB_SLAM2::Frame &currentFrame,// 定位等信息
                                        cv::Mat &imDepth, // 深度图
                                        cv::Mat &mask)    // mask 类别掩码
{
    if(currentFrame.mTcw.empty()){
        std::cout << "Geometry not working." << std::endl;
    }
    else if (mDB.mNumElem >= ELEM_INITIAL_MAP)// >5
   {
        vector<ORB_SLAM2::Frame> vRefFrames = GetRefFrames(currentFrame);// 从关键帧数据库中获取当前帧的参考帧====
        vector<DynKeyPoint> vDynPoints = ExtractDynPoints(vRefFrames,currentFrame);// 提取关键点===运动点=
        mask = DepthRegionGrowing(vDynPoints,imDepth);// 对稀疏的运动点 使用深度信息 进行扩展=====
        CombineMasks(currentFrame,mask);// 几何运动检测点集 再结合 mask分割信息=======
    }
}
*/

//  多视角几何 检测 运动点   
void Geometry::GeometricModelCorrection(
                                        const ORB_SLAM2::Frame &currentFrame,// 定位等信息
                                        cv::Mat &imDepth,// 深度信息用于 区域增长
                                        cv::Mat &mask)   // 获取的动态信息掩码 
{
    if(currentFrame.mTcw.empty()){
        std::cout << "Geometry not working." << std::endl;
    }
    else if (mDB.mNumElem >= ELEM_INITIAL_MAP)// >5
   {
        std::cout << "Geometry DB size ." << mDB.mNumElem << std::endl;
        vector<ORB_SLAM2::Frame> vRefFrames = GetRefFrames(currentFrame);
// 从关键帧数据库中获取当前帧的参考帧====

       vector<DynKeyPoint> vDynPoints = ExtractDynPoints(vRefFrames,currentFrame);// 提取运动点=
        mask = DepthRegionGrowing(vDynPoints,imDepth);// 对稀疏的运动点 使用深度信息 进行扩展=====

        // CombineMasks(currentFrame,mask);// 几何运动检测点集 再结合 mask分割信息=======
    }
}


// 使用关键帧更新数据库=====
void Geometry::GeometricModelUpdateDB(const ORB_SLAM2::Frame &currentFrame)
{
    if (currentFrame.mIsKeyFrame)// 是关键帧才更新
    {
        mDB.InsertFrame2DB(currentFrame);// 插入数据库====
    }
}

// 从关键帧数据库中获取当前帧的参考帧====
// 选取的原则是相似性差值最大的几个???
vector<ORB_SLAM2::Frame> Geometry::GetRefFrames(const ORB_SLAM2::Frame &currentFrame)
{
    cv::Mat rot1      = currentFrame.mTcw.rowRange(0,3).colRange(0,3);// 当前帧旋转矩阵  世界坐标系 到 相机===
    cv::Mat eul1      = rotm2euler(rot1);// 转欧拉角
    cv::Mat trans1 = currentFrame.mTcw.rowRange(0,3).col(3);// 平移向量
    cv::Mat vDist;// 平移向量 差值 L2范数  差平方和
    cv::Mat vRot;// 欧拉角度 差值L2范数差平方和  集合

    for (int i(0); i < mDB.mNumElem; i++)// 遍例数据库中的每一个，最近20个关键帧集合
   {
        cv::Mat rot2 = mDB.mvDataBase[i].mTcw.rowRange(0,3).colRange(0,3);// 数据库 关键帧 的旋转矩阵
        cv::Mat eul2 = rotm2euler(rot2);// 转欧拉角
        double distRot = cv::norm(eul2,eul1,cv::NORM_L2);// 欧拉角度 差值 L2范数  差平方和
        vRot.push_back(distRot);

        cv::Mat trans2 = mDB.mvDataBase[i].mTcw.rowRange(0,3).col(3);// 平移向量
        double dist = cv::norm(trans2,trans1,cv::NORM_L2);// 平移向量 差值 L2范数  差平方和
        vDist.push_back(dist);
    }
// 获取最大最小 角度差 和 距离差 再归一化===============
    double minvDist, maxvDist;
    cv::minMaxLoc(vDist, &minvDist, &maxvDist);// 最大最小 距离差
    vDist /= maxvDist;// 归一化，方便融合

    double minvRot, maxvRot;
    cv::minMaxLoc(vRot, &minvRot, &maxvRot);// 最大最小 角度差
    vRot /= maxvRot;// 归一化

    vDist = 0.7*vDist + 0.3*vRot;// 归一化后的角度差 和 距离差 加权融合
    cv::Mat vIndex;
    cv::sortIdx(vDist,vIndex,CV_SORT_EVERY_COLUMN + CV_SORT_DESCENDING);// 按列降序排列=====

    mnRefFrames = std::min(MAX_REF_FRAMES,vDist.rows);//  帧数< 5帧

    vector<ORB_SLAM2::Frame> vRefFrames;

    for (int i(0); i < mnRefFrames; i++)
    {
        int ind = vIndex.at<int>(0,i);// 帧id
        vRefFrames.push_back(mDB.mvDataBase[ind]);
        // 上面是降序排列，这里从前面取，是相似性差值最大的几个???

    }
    return(vRefFrames);
}

// 提取动态点=============这是不是可以考虑用光流来计算动态点===============
// 1. 选取 参考帧关键点 深度(0~6m) 计算参考帧下3d点 再变换到 世界坐标系下
// 2. 保留 当前帧 到 世界点 向量 与 参考帧到世界点向量 夹角 小于30的点， 不会太近的点
// 3. 保留世界点 反投影到当前帧坐标系下深度值 <7m的点
// 4. 保留世界点 反投影到当前帧像素坐标系下 浓缩平面( 20～620 & 20～460=)内的点,且该点，当前帧深度!=0
// 5. 根据投影点深度值和其 周围20×20领域点当前帧深度值 筛选出 深度差值较小的 领域点 的深度值 来更新当前帧 深度值
// 6. 点投影深度值 和 特征点当前帧下深度 差值过大，且该点周围深度方差小，确定该点为运动点
vector<Geometry::DynKeyPoint> Geometry::ExtractDynPoints(
      const vector<ORB_SLAM2::Frame> &vRefFrames,
      const ORB_SLAM2::Frame &currentFrame)
{
// 相机内参数
    cv::Mat K = cv::Mat::eye(3,3,CV_32F);
    K.at<float>(0,0) = currentFrame.fx;
    K.at<float>(1,1) = currentFrame.fy;
    K.at<float>(0,2) = currentFrame.cx;
    K.at<float>(1,2) = currentFrame.cy;

// 所有参考帧 筛选下来的 点
    cv::Mat vAllMPw;             // 世界坐标系下的点  4×M   (X/Z，Y/Z，1，1/Z)
    cv::Mat vAllMatRefFrame;     // 参考帧上 像素坐标齐次形式   M×3  (u,v,1)
    cv::Mat vAllLabels;          // 关键点所属 关键帧 id     M×1 
    cv::Mat vAllDepthRefFrame;   // 参考帧上的 深度值  M×1 

    for (int i(0); i < mnRefFrames; i++)// 遍历参考帧====
    {
        ORB_SLAM2::Frame refFrame = vRefFrames[i];

        // Fill matrix with points  参考帧上的 关键点====
        cv::Mat matRefFrame(refFrame.N,3,CV_32F);// N为关键点数量， 3通道, 关键点像素坐标 u，v ,1    像素坐标齐次形式
        cv::Mat matDepthRefFrame(refFrame.N,1,CV_32F);// 深度
        cv::Mat matInvDepthRefFrame(refFrame.N,1,CV_32F);// 逆深度
        cv::Mat vLabels(refFrame.N,1,CV_32F);//  关键点所属参考帧id
        int k(0);
        for(int j(0); j < refFrame.N; j++)// 参考帧的每个关键点=======
       {

// 步骤1：计算参考帧关键点 的世界坐标系坐标=============================================================
            const cv::KeyPoint &kp = refFrame.mvKeys[j];// 关键点====
            const float &v = kp.pt.y;
            const float &u = kp.pt.x;
            const float d = refFrame.mImDepth.at<float>(v,u);// 深度值
            if (d > 0 && d < 6)// 0～6米范围内 合理...
             {
                matRefFrame.at<float>(k,0) = refFrame.mvKeysUn[j].pt.x;
                matRefFrame.at<float>(k,1) = refFrame.mvKeysUn[j].pt.y;
                matRefFrame.at<float>(k,2) = 1.;// 第三通道 初始化为1，  像素坐标齐次形式
                matInvDepthRefFrame.at<float>(k,0) = 1./d;// 逆深度,其实可以不用============
                matDepthRefFrame.at<float>(k,0) = d;// 深度
                vLabels.at<float>(k,0) = i;// 关键点所属参考帧id
                k++;// k<= N
            }
        }

        matRefFrame = matRefFrame.rowRange(0,k);// 深度值合理的关键点数量 k，
        matInvDepthRefFrame = matInvDepthRefFrame.rowRange(0,k);// 逆深度
        matDepthRefFrame = matDepthRefFrame.rowRange(0,k);             // 深度
        vLabels = vLabels.rowRange(0,k);// 关键点所属参考帧id
        cv::Mat vMPRefFrame = K.inv()*matRefFrame.t();// 像素坐标齐次形式----> 当前帧坐标下的3d坐标， ----> 维度为: 3*k,  	X/Z,Y/Z,1
	// K = [fx, 0,  cx]
	//     [0,  fy, cy]
	//     [0,  0,  1 ]

	//K.inv() =
	//        [1/fx, 0,  -cx/fx]
	//        [0,  1/fy,  -cy/fy]
	//        [0,   0,       1 ]

	// vconcat（B,C，A）; // 等同于A=[B ;C]
        cv::vconcat(vMPRefFrame,matInvDepthRefFrame.t(),vMPRefFrame);
       //vMPRefFrame 维度为 4×k    (X/Z，Y/Z，1，1/Z)， 如果要是乘以Z的话就变成(X,Y,Z,1),3D坐标的齐次形式
        cv::Mat vMPw = refFrame.mTcw.inv() * vMPRefFrame;
       // 相机坐标系转换到 世界坐标下 --->   维度还是   4×k, (X/Z，Y/Z，1，1/Z), 不过值变了...

// 步骤2：选取 参考帧的世界坐标3d点 与当前帧 观察角度 角度 30度的 点======================================
        // 中间变量=====
        cv::Mat _vMPw = cv::Mat(4,vMPw.cols,CV_32F);
        cv::Mat _vLabels = cv::Mat(vLabels.rows,1,CV_32F);
        cv::Mat _matRefFrame = cv::Mat(matRefFrame.rows,3,CV_32F);
        cv::Mat _matDepthRefFrame = cv::Mat(matDepthRefFrame.rows,1,CV_32F);

        int h(0);
        mParallaxThreshold = 30;// 观测角度阈值 30度========论文来源
        for (int j(0); j < k; j++)// 遍例每一个 参考帧上的关键点=====
        {
            cv::Mat mp = cv::Mat(3,1,CV_32F);// 3D点===世界坐标系下
            mp.at<float>(0,0) = vMPw.at<float>(0,j)/matInvDepthRefFrame.at<float>(0,j);// X  直接乘以 matDepthRefFrame.at<float>(0,j) 也可以
            mp.at<float>(1,0) = vMPw.at<float>(1,j)/matInvDepthRefFrame.at<float>(0,j);// Y
            mp.at<float>(2,0) = vMPw.at<float>(2,j)/matInvDepthRefFrame.at<float>(0,j);// Z
            cv::Mat tRefFrame = refFrame.mTcw.rowRange(0,3).col(3);// 参考帧位置

            cv::Mat tCurrentFrame = currentFrame.mTcw.rowRange(0,3).col(3);// 当前帧位置=====
            cv::Mat nMPRefFrame = mp - tRefFrame;// 参考帧--->P
            cv::Mat nMPCurrentFrame = mp - tCurrentFrame;// 当前帧----->P

            double dotProduct = nMPRefFrame.dot(nMPCurrentFrame);// ||A|| × ||B||×cos()
            double normMPRefFrame = cv::norm(nMPRefFrame,cv::NORM_L2);// ||A||
            double normMPCurrentFrame = cv::norm(nMPCurrentFrame,cv::NORM_L2);// ||B||
            double angle = acos(dotProduct/(normMPRefFrame*normMPCurrentFrame))*180/M_PI;// 观测夹角===
            if (angle < mParallaxThreshold)// 观测角度小于阈值
            {
                _vMPw.at<float>(0,h) = vMPw.at<float>(0,j);// X/Z
                _vMPw.at<float>(1,h) = vMPw.at<float>(1,j);// Y/Z
                _vMPw.at<float>(2,h) = vMPw.at<float>(2,j);// 1
                _vMPw.at<float>(3,h) = vMPw.at<float>(3,j);// 1/Z
                _vLabels.at<float>(h,0) = vLabels.at<float>(j,0);//关键点所属参考帧id
                _matRefFrame.at<float>(h,0) = matRefFrame.at<float>(j,0);// u
                _matRefFrame.at<float>(h,1) = matRefFrame.at<float>(j,1);// v
                _matRefFrame.at<float>(h,2) = matRefFrame.at<float>(j,2);// 1
                _matDepthRefFrame.at<float>(h,0) = matDepthRefFrame.at<float>(j,0);// Z 深度值
                h++;// 观察角度小于30度 剩下的点数量=====
            }
        }
        // 更新 
        vMPw = _vMPw.colRange(0,h);       // 世界坐标点 4×h     (X/Z，Y/Z，1，1/Z)
        vLabels = _vLabels.rowRange(0,h);// 关键点所属参考帧id     h*1 
        matRefFrame = _matRefFrame.rowRange(0,h);//   像素坐标齐次形式  (u，v，1)   h*3
        matDepthRefFrame = _matDepthRefFrame.rowRange(0,h);// 深度值  h*1

// 步骤3： 所有参考帧 筛选下来的 点=============================================================

        if (vAllMPw.empty())// 空的时候直接添加进入
        {
            vAllMPw = vMPw;
            vAllMatRefFrame = matRefFrame;
            vAllLabels = vLabels;
            vAllDepthRefFrame = matDepthRefFrame;
        }
        else
        {
            if (!vMPw.empty())
            {
                hconcat(vAllMPw,vMPw,vAllMPw);// 按列扩充
                vconcat(vAllMatRefFrame,matRefFrame,vAllMatRefFrame);//按行扩充===
                vconcat(vAllLabels,vLabels,vAllLabels);
                vconcat(vAllDepthRefFrame,matDepthRefFrame,vAllDepthRefFrame);
            }
        }
    }// 所有参考帧已筛选完毕===

    cv::Mat vLabels = vAllLabels;// 关键点所属 关键帧 id     M×1 ==

// 步骤4： 后处理 将筛选后的点变换到当前相机坐标系下进行分析========================================
    // 比较参考帧下的深度 和 当前帧下的深度=====来确定是否是动态点
    if (!vAllMPw.empty())
    {
        cv::Mat vMPCurrentFrame = currentFrame.mTcw * vAllMPw;// 变换到当前相机坐标系下=============

        // Divide by last column   (X/Z，Y/Z，1，1/Z) ----> (X,Y,Z,1)
        for (int i(0); i < vMPCurrentFrame.cols; i++)// 4×M  (X/Z，Y/Z，1，1/Z)
        {
            vMPCurrentFrame.at<float>(0,i) /= vMPCurrentFrame.at<float>(3,i);
            vMPCurrentFrame.at<float>(1,i) /= vMPCurrentFrame.at<float>(3,i);
            vMPCurrentFrame.at<float>(2,i) /= vMPCurrentFrame.at<float>(3,i);
            vMPCurrentFrame.at<float>(3,i) /= vMPCurrentFrame.at<float>(3,i);
        }
        cv::Mat matProjDepth = vMPCurrentFrame.row(2); // 反投影后在当前帧下的 Z,深度值====1×M
        
// 步骤4.1 保留 反投影后深度值小于 7m的点==================================================== 
        // 中间变量 
        cv::Mat _vMPCurrentFrame = cv::Mat(vMPCurrentFrame.size(),CV_32F);
        //  当前坐标系下3d坐标齐次形式     (X,Y,Z,1)    4×M
        cv::Mat _vAllMatRefFrame = cv::Mat(vAllMatRefFrame.size(),CV_32F);   
        //  参考帧像素坐标齐次形式 (u,v,1)      M×3
        cv::Mat _vLabels = cv::Mat(vLabels.size(),CV_32F);                  
        //  所属 参考帧id                           M×1
        cv::Mat __vAllDepthRefFrame = cv::Mat(vAllDepthRefFrame.size(),CV_32F);
        // 参考帧上深度                            M×1
        int h(0);
        cv::Mat __matProjDepth = cv::Mat(matProjDepth.size(),CV_32F);
        for (int i(0); i < matProjDepth.cols; i++)//  关键帧点 反投影后的 Z,深度值====1×M
        {
            if (matProjDepth.at<float>(0,i) < 7)// 小于7m======
            {
                __matProjDepth.at<float>(0,h) = matProjDepth.at<float>(0,i);
                // 关键帧点 反投影后在当前帧下的 Z,深度值
                _vMPCurrentFrame.at<float>(0,h) = vMPCurrentFrame.at<float>(0,i);
                // 当前坐标系下3d坐标齐次形式     (X,Y,Z,1)  
                _vMPCurrentFrame.at<float>(1,h) = vMPCurrentFrame.at<float>(1,i);
                _vMPCurrentFrame.at<float>(2,h) = vMPCurrentFrame.at<float>(2,i);
                _vMPCurrentFrame.at<float>(3,h) = vMPCurrentFrame.at<float>(3,i);

                _vAllMatRefFrame.at<float>(h,0) = vAllMatRefFrame.at<float>(i,0);
                //  参考帧像素坐标齐次形式 (u,v,1)  
                _vAllMatRefFrame.at<float>(h,1) = vAllMatRefFrame.at<float>(i,1);
                _vAllMatRefFrame.at<float>(h,2) = vAllMatRefFrame.at<float>(i,2);

                _vLabels.at<float>(h,0) = vLabels.at<float>(i,0);// 所属 参考帧id      

                __vAllDepthRefFrame.at<float>(h,0) = vAllDepthRefFrame.at<float>(i,0);
                //  参考帧上深度   

                h++;
            }
        }
        // 更新=============
        matProjDepth = __matProjDepth.colRange(0,h);//  关键帧点 反投影后在当前帧下的 Z,深度值  1×h
        vMPCurrentFrame = _vMPCurrentFrame.colRange(0,h);
        //  当前坐标系下3d坐标齐次形式     (X,Y,Z,1)    4×h
        vAllMatRefFrame = _vAllMatRefFrame.rowRange(0,h);    //  参考帧像素坐标齐次形式 (u,v,1)     h*3
        vLabels = _vLabels.rowRange(0,h);                                          // 所属 参考帧id       h*1
        vAllDepthRefFrame = __vAllDepthRefFrame.rowRange(0,h);//  参考帧上深度   h*1

// 步骤4.2 保留反投影后点在当前真帧像素平面上的关键点，且当前测量深度有效================
        cv::Mat aux;
        cv::hconcat(cv::Mat::eye(3,3,CV_32F),cv::Mat::zeros(3,1,CV_32F),aux);
	// 变形矩阵=====   3×4   *  4×h ----> 3×h
	//  [1 0 0 0]
	//  [0 1 0 0]
	//  [0 0 1 0]
        cv::Mat matCurrentFrame = K*aux*vMPCurrentFrame;// 投影到当前帧像素坐标下==== 齐次坐标

        cv::Mat mat2CurrentFrame(matCurrentFrame.cols,2,CV_32F); // 当前帧下 像素坐标 (u,v):
        cv::Mat v2AllMatRefFrame(matCurrentFrame.cols,3,CV_32F);    // 参考帧下像素 齐次坐标
        cv::Mat mat2ProjDepth(matCurrentFrame.cols,1,CV_32F);         // 关键帧点 投影到当前帧下的深度
        cv::Mat v2Labels(matCurrentFrame.cols,1,CV_32F);                       //  所属关键帧 id
        cv::Mat _vAllDepthRefFrame(matCurrentFrame.cols,1,CV_32F);// 参考帧下的深度

        int j = 0;
        for (int i(0); i < matCurrentFrame.cols; i++)
        {
            float x = ceil(matCurrentFrame.at<float>(0,i)/matCurrentFrame.at<float>(2,i));// 齐次坐标转非齐次 (U,V,W) ----->(U/W,V/W,1)---->(u,v)
            float y = ceil(matCurrentFrame.at<float>(1,i)/matCurrentFrame.at<float>(2,i));
            if (IsInFrame(x,y,currentFrame))// 该点需要在当前帧 像素平面内   640×480  落在 外围款20×20之内   20～620 & 20～460======
            {
                const float d = currentFrame.mImDepth.at<float>(y,x);// 当前帧下该点的深度
                if (d > 0)// 当前帧的深度可靠===============================================
                {
                    mat2CurrentFrame.at<float>(j,0) = x;// 对应当前帧下的像素坐标
                    mat2CurrentFrame.at<float>(j,1) = y;
                    v2AllMatRefFrame.at<float>(j,0) = vAllMatRefFrame.at<float>(i,0);// 参考帧下像素 齐次坐标
                    v2AllMatRefFrame.at<float>(j,1) = vAllMatRefFrame.at<float>(i,1);
                    v2AllMatRefFrame.at<float>(j,2) = vAllMatRefFrame.at<float>(i,2);
                    _vAllDepthRefFrame.at<float>(j,0) = vAllDepthRefFrame.at<float>(i,0);// 参考帧下的深度
                    float d1 = matProjDepth.at<float>(0,i); // 关键帧点 投影到当前帧下的深度
                    mat2ProjDepth.at<float>(j,0) = d1;
                    v2Labels.at<float>(j,0) = vLabels.at<float>(i,0);// 所属关键帧 id
                    j++;
                }
            }
        }
        // 更新===
        vAllDepthRefFrame = _vAllDepthRefFrame.rowRange(0,j);// 参考帧下的深度 j*1
        vAllMatRefFrame = v2AllMatRefFrame.rowRange(0,j);    // 参考帧下像素 齐次坐标  j*3
        matProjDepth = mat2ProjDepth.rowRange(0,j);               // 关键帧点 投影到当前帧下的深度 j*1
        matCurrentFrame = mat2CurrentFrame.rowRange(0,j);// 当前帧下 像素坐标 (u,v)                 j*2
        vLabels = v2Labels.rowRange(0,j);// 所属关键帧 id


// 步骤4.3 投影关键点深度 和 周围点测量深度比较================================
        // 周围框 20×20======
        cv::Mat u1((2*mDmax+1)*(2*mDmax+1),2,CV_32F); // mDmax = 20   IsInFrame() 中=
        int m(0);
        for (int i(-mDmax); i <= mDmax; i++){    // -20～20  周围 20×20的框
            for (int j(-mDmax); j <= mDmax; j++){// -20～20
                u1.at<float>(m,0) = i;// 周围 20×20领域点相对坐标
                u1.at<float>(m,1) = j;
                m++;
            }
        }
        cv::Mat matDepthCurrentFrame(matCurrentFrame.rows,1,CV_32F);// 当前帧下深度  j*1
        cv::Mat _matProjDepth(matCurrentFrame.rows,1,CV_32F);       // 关键帧点 投影到当前帧下的深度 j*1
        cv::Mat _matCurrentFrame(matCurrentFrame.rows,2,CV_32F);    // 当前帧下 像素坐标 (u,v)                 j*2

        int _s(0);
        for (int i(0); i < matCurrentFrame.rows; i++)// 每一个 靠谱的 当前帧下的 像素坐标 (u,v)  
        {
            int s(0);
            cv::Mat _matDiffDepth(u1.rows,1,CV_32F);// 深度差值 =关键帧关键点点投影到当前帧下的深度 -周围点的深度
            cv::Mat _matDepth(u1.rows,1,CV_32F);// 保留的周围点的深度
            for (int j(0); j < u1.rows; j++)// 41×41  周围 20×20的框
            {
                // 关键点周围 20×20框内的点=====
                int x = (int)matCurrentFrame.at<float>(i,0) + (int)u1.at<float>(j,0);
                int y = (int)matCurrentFrame.at<float>(i,1) + (int)u1.at<float>(j,1);
                float _d = currentFrame.mImDepth.at<float>(y,x);// 对应的深度
                if ((_d > 0) && (_d < matProjDepth.at<float>(i,0)))//  周围点深度 小于  关键帧关键点点投影到当前帧下的深度==================
                {// 前方的点
                    _matDepth.at<float>(s,0) = _d;// 保留的周围点的深度
                    _matDiffDepth.at<float>(s,0) = matProjDepth.at<float>(i,0) - _d;// 深度差值 =关键帧关键点点投影到当前帧下的深度 -周围点的深度
                    s++;// 该 投影关键点 20×20领域点中 选出来的点数量====
                }
            }
            // 
            if (s > 0)// 该 投影关键点 附近 有 深度小的点,前方有点
            {
                _matDepth = _matDepth.rowRange(0,s);// 保留的周围点的深度
                _matDiffDepth = _matDiffDepth.rowRange(0,s);// 深度差值 
                double minVal, maxVal;
                cv::Point minIdx, maxIdx;
                cv::minMaxLoc(_matDiffDepth,&minVal,&maxVal,&minIdx,&maxIdx);// 深度差值最小点 和 深度差值最大点====
                int xIndex = minIdx.x;// 在关键点前方，靠的较近，深度差值小
                int yIndex = minIdx.y;
                matDepthCurrentFrame.at<float>(_s,0) = _matDepth.at<float>(yIndex,0);// 周围点深度 设置为当前帧下关键点的深度=========================
                _matProjDepth.at<float>(_s,0) = matProjDepth.at<float>(i,0);// 投影深度
                _matCurrentFrame.at<float>(_s,0) = matCurrentFrame.at<float>(i,0);// 当前帧下 像素坐标 (u,v) 
                _matCurrentFrame.at<float>(_s,1) = matCurrentFrame.at<float>(i,1);
                _s++;// 注意为 _s
            }
        }
        matDepthCurrentFrame = matDepthCurrentFrame.rowRange(0,_s);// 筛选出来的 周围有深度值小的 关键点，深度用 与其差值最小的代替====
        matProjDepth = _matProjDepth.rowRange(0,_s);// 关键帧点投影到当前帧下的 深度
        matCurrentFrame = _matCurrentFrame.rowRange(0,_s);// 当前帧下 像素坐标 (u,v) 

// 步骤4.4 特征点投影深度值 和特征点当前帧下深度 差值过大，且该点周围深度方差小，确定该点为运动点
        mDepthThreshold = 0.6;// 投影前后深度差值阈值，确定为运动点=========
        cv::Mat matDepthDifference = matProjDepth - matDepthCurrentFrame;// 投影深度 和  优化后的当前帧点深度差值
        mVarThreshold = 0.001; //0.040; //周围领域方差 阈值，避免选到边界点=====

        vector<Geometry::DynKeyPoint> vDynPoints;// 筛选出来的运动的点==================================

        for (int i(0); i < matCurrentFrame.rows; i++)
        {
            if (matDepthDifference.at<float>(i,0) > mDepthThreshold)// 按照投影点附近领域点 当前帧深度 更新当前帧点深度值 在和投影点深度值做差，差值过大为运动点====
            {
                int xIni = (int)matCurrentFrame.at<float>(i,0) - mDmax;// 再做判断 该关键点附近领域
                int yIni = (int)matCurrentFrame.at<float>(i,1) - mDmax;
                int xEnd = (int)matCurrentFrame.at<float>(i,0) + mDmax + 1;
                int yEnd = (int)matCurrentFrame.at<float>(i,1) + mDmax + 1;
                cv::Mat patch = currentFrame.mImDepth.rowRange(yIni,yEnd).colRange(xIni,xEnd);// 关键点附近领域 深度值块=====
                cv::Mat mean, stddev;
                cv::meanStdDev(patch,mean,stddev);// 深度值块 均值和标准差=====
                double _stddev = stddev.at<double>(0,0);
                double var = _stddev*_stddev;// 方差===
                if (var < mVarThreshold)//周围领域方差 阈值，避免选到边界点=====
                {
                    DynKeyPoint dynPoint;
                    dynPoint.mPoint.x = matCurrentFrame.at<float>(i,0);// 运动点 当前帧像素坐标
                    dynPoint.mPoint.y = matCurrentFrame.at<float>(i,1);// 
                    dynPoint.mRefFrameLabel = vLabels.at<float>(i,0);// 所属参考帧id
                    vDynPoints.push_back(dynPoint);
                }
            }
        }

        return vDynPoints;
    }
    else
    {
        vector<Geometry::DynKeyPoint> vDynPoints;
        return vDynPoints;
    }
}


// 动态点 依据深度信息 进行区域增长扩充=================================
cv::Mat Geometry::DepthRegionGrowing(
                  const vector<DynKeyPoint> &vDynPoints,
                  const cv::Mat &imDepth)
{

    cv::Mat maskG = cv::Mat::zeros(480,640,CV_32F);// 默认0

    if (!vDynPoints.empty())
    {
        mSegThreshold = 0.20;// 分割阈值

        for (size_t i(0); i < vDynPoints.size(); i++)
       {
           // 动态点=====
            int xSeed = vDynPoints[i].mPoint.x;
            int ySeed = vDynPoints[i].mPoint.y;
           // 深度点
            const float d = imDepth.at<float>(ySeed,xSeed);
            if (maskG.at<float>(ySeed,xSeed) != 1. && d > 0)
            {
                // 动态点附近，根据深度值 区域增长分割扩充
                cv::Mat J = RegionGrowing(imDepth,xSeed,ySeed,mSegThreshold);
                maskG = maskG | J;// 合并 动态点分割mask
            }
        }

// 膨胀，选大的
        int dilation_size = 15;// 膨胀核大小===
        cv::Mat kernel = getStructuringElement(cv::MORPH_ELLIPSE,
                                               cv::Size( 2*dilation_size + 1, 2*dilation_size+1 ),
                                               cv::Point( dilation_size, dilation_size ) );
        maskG.cv::Mat::convertTo(maskG,CV_8U);// 0～1,1处动，0处静
        cv::dilate(maskG, maskG, kernel);// 膨胀，15×15核内 选最大的
    }
    else
    {
        maskG.cv::Mat::convertTo(maskG,CV_8U);
    }

    cv::Mat _maskG = cv::Mat::ones(480,640,CV_8U);// 全1
    maskG = _maskG - maskG;// 1-mask， 0处动，1处静

    return maskG;
}


// 面积
float Area(float x1, float x2, float y1, float y2)
{
    float xc1 = max(x1-0.5,x2-0.5);
    float xc2 = min(x1+0.5,x2+0.5);
    float yc1 = max(y1-0.5,y2-0.5);
    float yc2 = min(y1+0.5,y2+0.5);
    return (xc2-xc1)*(yc2-yc1);
}

// 插入数据库====================================================
void Geometry::DataBase::InsertFrame2DB(const ORB_SLAM2::Frame &currentFrame)
{

    if (!IsFull())
    {
        mvDataBase[mFin] = currentFrame;
        mFin = (mFin + 1) % MAX_DB_SIZE; // 最多存放 MAX_DB_SIZE = 20个
        mNumElem += 1;
    }
    else {
        mvDataBase[mIni] = currentFrame;
        mFin = mIni;
        mIni = (mIni + 1) % MAX_DB_SIZE;
    }
}

bool Geometry::DataBase::IsFull(){
    return (mIni == (mFin+1) % MAX_DB_SIZE);
}

// 旋转矩阵 转 欧拉角============================
cv::Mat Geometry::rotm2euler(const cv::Mat &R)
{
    assert(isRotationMatrix(R));
    float sy = sqrt(R.at<double>(0,0) * R.at<double>(0,0) +  R.at<double>(1,0) * R.at<double>(1,0) );
    bool singular = sy < 1e-6;
    float x, y, z;
    if (!singular)
    {
        x = atan2(R.at<double>(2,1) , R.at<double>(2,2));
        y = atan2(-R.at<double>(2,0), sy);
        z = atan2(R.at<double>(1,0), R.at<double>(0,0));
    }
    else
    {
        x = atan2(-R.at<double>(1,2), R.at<double>(1,1));
        y = atan2(-R.at<double>(2,0), sy);
        z = 0;
    }
    cv::Mat res = (cv::Mat_<double>(1,3) << x, y, z);
    return res;
}

// 判断是否是旋转矩阵=============================
bool Geometry::isRotationMatrix(const cv::Mat &R)
{
    cv::Mat Rt;
    transpose(R,Rt);// 转置
    cv::Mat shouldBeIdentity = Rt*R;//  I 单位矩阵
    cv::Mat I = cv::Mat::eye(3,3,shouldBeIdentity.type());
    return norm(I,shouldBeIdentity) < 1e-6;
}

// 点是否在帧上===================
bool Geometry::IsInFrame(
           const float &x, const float &y, 
           const ORB_SLAM2::Frame &Frame)
{
    mDmax = 20; // 点在 外围 20像素之内，认为在 图像上.....
    // 后面有 20×20 领域深度值比较分析========
    return (x > (mDmax + 1) && x < (Frame.mImDepth.cols - mDmax - 1) && y > (mDmax + 1) && y < (Frame.mImDepth.rows - mDmax - 1));
}

// 在 图像的尺寸范围内==============
bool Geometry::IsInImage(
            const float &x, const float &y, 
            const cv::Mat image)
{
    return (x >= 0 && x < (image.cols) && y >= 0 && y < image.rows);
}

// 区域增长==================没看懂???====
cv::Mat Geometry::RegionGrowing(
             const cv::Mat &im,// 深度图
             int &x,int &y,// 动态点
             const float &reg_maxdist)//增长分割阈值
{

    cv::Mat J = cv::Mat::zeros(im.size(),CV_32F);// 全1mask

    float reg_mean = im.at<float>(y,x);// 动态点 处 深度值
    int reg_size = 1;

    int _neg_free = 10000;
    int neg_free = 10000;
    int neg_pos = -1;
    cv::Mat neg_list = cv::Mat::zeros(neg_free,3,CV_32F);// 2d点+深度值

    double pixdist=0;

    //Neighbor locations (footprint)
    cv::Mat neigb(4,2,CV_32F);// 周围4点 
    neigb.at<float>(0,0) = -1;
    neigb.at<float>(0,1) = 0;

    neigb.at<float>(1,0) = 1;
    neigb.at<float>(1,1) = 0;

    neigb.at<float>(2,0) = 0;
    neigb.at<float>(2,1) = -1;

    neigb.at<float>(3,0) = 0;
    neigb.at<float>(3,1) = 1;

    while(pixdist < reg_maxdist && reg_size < im.total())
    {
        for (int j(0); j< 4; j++)
        {
            //Calculate the neighbour coordinate
            int xn = x + neigb.at<float>(j,0);// 周围4点 
            int yn = y + neigb.at<float>(j,1);

            bool ins = ((xn >= 0) && (yn >= 0) && (xn < im.cols) && (yn < im.rows));
            if (ins && (J.at<float>(yn,xn) == 0.))
            {
                neg_pos ++;
                neg_list.at<float>(neg_pos,0) = xn;
                neg_list.at<float>(neg_pos,1) = yn;
                neg_list.at<float>(neg_pos,2) = im.at<float>(yn,xn);// 周围四点 的深度值
                J.at<float>(yn,xn) = 1.;// 已经处理过
            }
        }

        // Add a new block of free memory
        if((neg_pos + 10) > neg_free){
            cv::Mat _neg_list = cv::Mat::zeros(_neg_free,3,CV_32F);
            neg_free += 10000;
            vconcat(neg_list,_neg_list,neg_list);// 扩展矩阵
        }

        // Add pixel with intensity nearest to the mean of the region, to the region
        cv::Mat dist;
        for (int i(0); i < neg_pos; i++){
            double d = abs(neg_list.at<float>(i,2) - reg_mean);// reg_mean: 动态点 处 深度值，与周围点深度值之差
            dist.push_back(d);
        }
        double max;
        cv::Point ind, maxpos;
        cv::minMaxLoc(dist, &pixdist, &max, &ind, &maxpos);// 深度值差
        int index = ind.y;

        if (index != -1)
        {
            J.at<float>(y,x) = -1.;
            reg_size += 1;

            // Calculate the new mean of the region
            reg_mean = (reg_mean*reg_size + neg_list.at<float>(index,2))/(reg_size+1);

            // Save the x and y coordinates of the pixel (for the neighbour add proccess)
            x = neg_list.at<float>(index,0);
            y = neg_list.at<float>(index,1);

            // Remove the pixel from the neighbour (check) list
            neg_list.at<float>(index,0) = neg_list.at<float>(neg_pos,0);
            neg_list.at<float>(index,1) = neg_list.at<float>(neg_pos,1);
            neg_list.at<float>(index,2) = neg_list.at<float>(neg_pos,2);
            neg_pos -= 1;
        }
        else
        {
            pixdist = reg_maxdist;
        }

    }

    J = cv::abs(J);
    return(J);
}

}
