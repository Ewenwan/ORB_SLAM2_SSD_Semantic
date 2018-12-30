/**
* This file is part of ORB-SLAM2.
* 
*/

#ifndef CONVERTER_H
#define CONVERTER_H

#include<opencv2/core/core.hpp>

#include<Eigen/Dense>
#include"Thirdparty/g2o/g2o/types/types_six_dof_expmap.h"
#include"Thirdparty/g2o/g2o/types/types_seven_dof_expmap.h"

// ORB里用的一些Convert函数，虽然我自己不用，但是有一些ORB里的旧代码用了，我也懒得改。。

namespace ORB_SLAM2
{

class Converter
{
public:
   // 矩阵形式的描述子 转向量形式的描述子
    static std::vector<cv::Mat> toDescriptorVector(const cv::Mat &Descriptors);

   // 4*4 mat形式的 位姿态  R t 转化成 G2O 6自由度顶点优化变量类型  李代数形式 SE3Quat  
    static g2o::SE3Quat toSE3Quat(const cv::Mat &cvT);

      /** unimplemented */
    static g2o::SE3Quat toSE3Quat(const g2o::Sim3 &gSim3);// 李代数形式

    // g2o 类型转Mat=======
    static cv::Mat toCvMat(const g2o::SE3Quat &SE3);
    static cv::Mat toCvMat(const g2o::Sim3 &Sim3);

     // Eigen 转 Mat========
    static cv::Mat toCvMat(const Eigen::Matrix<double,4, 4> &m);
    static cv::Mat toCvMat(const Eigen::Matrix3d &m);
    static cv::Mat toCvMat(const Eigen::Matrix<double,3, 1> &m);
    static cv::Mat toCvSE3(const Eigen::Matrix<double, 3, 3> &R,
                                                   const Eigen::Matrix<double, 3,1> &t);
       
 // Mat 转 Eigen============================
    static Eigen::Matrix<double, 3, 1>  toVector3d(const cv::Mat &cvVector);
    static Eigen::Matrix<double, 3, 1>  toVector3d(const cv::Point3f &cvPoint);
    static Eigen::Matrix<double, 3, 3>  toMatrix3d(const cv::Mat &cvMat3);
// 转四元素==========
    static std::vector<float>  toQuaternion(const cv::Mat &M);
// 载入地图时会用到=====
    void RmatOfQuat(cv::Mat &M, const cv::Mat &q);

};

}// namespace ORB_SLAM

#endif // CONVERTER_H
