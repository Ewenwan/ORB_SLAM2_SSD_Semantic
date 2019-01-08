/* This file is part of ORB-SLAM2-SSD-Semantic.
 * 语义数据库===========
 * 添加，删除，融合目标数据
 */

#include "ObjectDatabase.h"

// 重写 Cluster的等号操作符，方便按名字查找
bool Cluster::operator ==(const std::string &x){
    return(this->object_name == x);
} 

// 拷贝对象函数=====


// 类构造函数=======
ObjectDatabase::ObjectDatabase()
{
    DataBaseSize = 0;
    mClusters.clear();
// 不同颜色对应不同物体 ================================================
    for (int i = 0; i < 21; i++) // 带有背景
    {  // voc数据集 20类物体=======
       mvColors.push_back(cv::Scalar( i*10 + 40, i*10 + 40, i*10 + 40));
       // "background","aeroplane", "bicycle", "bird", "boat",
       // "bottle", "bus", "car", "cat", "chair",
       // "cow", "diningtable", "dog", "horse","motorbike", "person", 
       // "pottedplant","sheep", "sofa", "train", "tvmonitor"
    }
    mvColors[5] = cv::Scalar(255,0,255); // 瓶子 粉色    bgr
    mvColors[9] = cv::Scalar(255,0,0);   // 椅子 蓝色
    mvColors[15] = cv::Scalar(0,0,255);  // 人 红色
    mvColors[20] = cv::Scalar(0,255,0);  // 显示器 绿色 
    
// 物体尺寸大小 =========================================================
    for (int i = 0; i < 21; i++)  
    { // voc数据集 20类物体=======
      mvSizes.push_back(0.6);
    }
    mvSizes[5] = 0.06;   // 瓶子  0.06 米以内认为是同一个物体 
    mvSizes[9] = 0.5;    // 椅子  
    mvSizes[15] = 0.35;  // 人   
    mvSizes[20] = 0.25;  // 显示器  
}
// 类析构函数===============
ObjectDatabase::~ObjectDatabase()
{
}
// 返回 定义的物体颜色====================
cv::Scalar  ObjectDatabase::getObjectColor(int class_id)
{
   return mvColors[class_id];
}
// 返回 定义的物体尺寸==================
float ObjectDatabase::getObjectSize(int class_id)
{
   return mvSizes[class_id];
}       
// 返回数据库中 同名字的物体数据=========
std::vector<Cluster>  ObjectDatabase::getObjectByName(std::string objectName)
{

        // 按名字查物体是否在数据库
	std::vector<Cluster>::iterator iter   = mClusters.begin()-1;
	std::vector<Cluster>::iterator it_end = mClusters.end(); 
        std::vector<Cluster> sameName;// 同名字的物体 
	while(true) 
        {
	    iter = find(++iter, it_end, objectName);// 按照名字查找
	    if (iter != it_end )// 找到一个，存放起来
                sameName.push_back(*iter);
	    else//已经找不到了
	        break;  
	}
        return sameName; // 这里可以传递引用过来，减少赋值时间===
}

void ObjectDatabase::addObject(Cluster& cluster)
{
    // 1. 查看总数量,数据库为空直接加入
    if(!mClusters.size())
    {
        DataBaseSize++;
        cluster.object_id = DataBaseSize;
        mClusters.push_back(cluster);
        return;
    }
    else
    {
        // 2. 数据库内已经存在物体了，查找新物体是否在数据库内已经存在
	std::vector<Cluster>::iterator iter   = mClusters.begin()-1;
	std::vector<Cluster>::iterator it_end = mClusters.end(); 
        std::vector<std::vector<Cluster>::iterator> likely_obj;// 同名字的物体的迭代器
	while(true) 
        {
	    iter = find(++iter, it_end, cluster.object_name);// 按照名字查找
	    if (iter != it_end )// 找到一个，存放起来
                likely_obj.push_back(iter);
	    else//已经找不到了
	        break;  
	}

        // 3. 如果没找到，则直接添加 进数据库
        std::vector<Cluster>::iterator best_close;// 最近的索引
        float center_distance=100;// 对应的距离
        if(!likely_obj.size())
        {
            DataBaseSize++;
            cluster.object_id = DataBaseSize;
            mClusters.push_back(cluster);
            return;
        }
        else//找到多个和数据库里同名字的物体
        {
            // 4. 遍例每一个同名字的物体，找到中心点最近的一个
            for(unsigned int j=0; j<likely_obj.size(); j++)
            {
                std::vector<Cluster>::iterator& temp_iter = likely_obj[j];
                Cluster& temp_cluster = *temp_iter;
                Eigen::Vector3f dis_vec = cluster.centroid - temp_cluster.centroid;// 中心点连接向量
                float dist = dis_vec.norm();
                if(dist < center_distance)
                {
                    center_distance = dist; // 最短的距离
                    best_close      = temp_iter;// 对应的索引
                }
            }
            // 5. 如果距离小于物体尺寸，则认为是同一个空间中的同一个物体，更新数据库中该物体的信息
            if(center_distance < mvSizes[cluster.class_id])
            {
                best_close->prob    = (best_close->prob + cluster.prob)/2.0; // 综合置信度
                best_close->centroid = (best_close->centroid + cluster.centroid)/2.0; // 中心平均
                best_close->size     = (best_close->size + cluster.size)/2.0; // 中心尺寸
            }
            else 
            {
            // 6. 如果距离超过物体尺寸则认为是不同位置的同一种物体，直接放入数据库
                DataBaseSize++;
                cluster.object_id = DataBaseSize;
                mClusters.push_back(cluster); 
            }
        }
    }

// 数据库大小限制======，超过一定大小，删除掉 置信度低的目标====
    return; 
}



