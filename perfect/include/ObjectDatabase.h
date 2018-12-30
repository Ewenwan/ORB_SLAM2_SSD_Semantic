/* This file is part of ORB-SLAM2-SSD-Semantic.
 * 语义目标数据库=======
 * 添加，删除，融合目标数据
 */

#ifndef OBJECTDATABASE_H
#define OBJECTDATABASE_H

#include "System.h"

#include <Eigen/Core>
#include <vector>
#include <string>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>

// 目标语义信息
typedef struct Cluster
{
 Eigen::Vector3f size;    // 3d框尺寸
 Eigen::Vector3f centroid;// 点云中心点 
 float prob;              // 置信度
 std::string object_name; // 物体类别名
 int class_id;            // 对应类别id
 int object_id;           // 物体编号
 bool operator ==(const std::string &x);
} Cluster;


class ObjectDatabase
{
public:
    ObjectDatabase();
    ~ObjectDatabase();
    void addObject(Cluster& cluster);
    cv::Scalar  getObjectColor(int class_id); // 定义的物体颜色
    float getObjectSize(int class_id);        // 定义的物体尺寸

    std::vector<Cluster>  getObjectByName(std::string objectName);// 返回数据库中 同名字的物体数据

    std::vector<Cluster> mClusters;   // 语义点云目标数组
protected:
    std::vector<cv::Scalar> mvColors;// 每种物体的颜色
    std::vector<float>      mvSizes; // 每种物体的大小  
    int DataBaseSize; 
};

#endif // OBJECTDATABASE_H
