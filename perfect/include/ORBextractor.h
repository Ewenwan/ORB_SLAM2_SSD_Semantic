/**
* This file is part of ORB-SLAM2.
* 
*作者对opencv中的orb源码进行了修改，将特征进行均匀化
* 
* 特征提取也就是对图像进行一定的操作，也就是对像素点进行一些操作，
* 跟相邻的一些像素点进行比较，通过一些模板进行滤波卷积等操作，再通过阈值进行一些控制，
* 找到了可以代表该图像的某些位置，这也就是特征提取。 
* 
*/

#ifndef ORBEXTRACTOR_H
#define ORBEXTRACTOR_H

#include <vector>
#include <list>
#include <opencv/cv.h>


namespace ORB_SLAM2
{

    class ExtractorNode
    {
    public:
	ExtractorNode():bNoMore(false){}

	void DivideNode(ExtractorNode &n1, ExtractorNode &n2, ExtractorNode &n3, ExtractorNode &n4);

	std::vector<cv::KeyPoint> vKeys;
	cv::Point2i UL, UR, BL, BR;
	std::list<ExtractorNode>::iterator lit;
	bool bNoMore;
    };

    class ORBextractor
    {
    public:
	
	enum {HARRIS_SCORE=0, FAST_SCORE=1 };

	ORBextractor(int nfeatures, float scaleFactor, int nlevels,
		    int iniThFAST, int minThFAST);
// int features_num = 500, float scale_factor = 1.2f, int levels_num = 8,int default_fast_threshold = 20, int min_fast_threshold = 7
// 特征点总数  尺度因子  金字塔总层数  快速角点提取阈值大 小	
// 为了防止用默认阈值fast角点检测检测的特征数过少，
// 添加设置min_fast_threshold最小的fast特征检测阈值，以保证检测的特征数目。
	
	~ORBextractor(){}//析构函数

	// Compute the ORB features and descriptors on an image.  计算 特征 和 描述子
	// ORB are dispersed on the image using an octree.  将orb特征分配到一个八叉树当中
	// Mask is ignored in the current implementation. 目前mask参数是被忽略的，没有实现
	void operator()( cv::InputArray image, cv::InputArray mask,
	  std::vector<cv::KeyPoint>& keypoints,
	  cv::OutputArray descriptors);

	// 内联函数
	int inline GetLevels(){
	    return nlevels;}// 返回 金字塔层数

	float inline GetScaleFactor(){
	    return scaleFactor;}// 返回尺度因子

	std::vector<float> inline GetScaleFactors(){
	    return mvScaleFactor;// 返回每一层级的 尺度因子
	}

	std::vector<float> inline GetInverseScaleFactors(){
	    return mvInvScaleFactor;// 返回每一层级的尺度因子 的 倒数
	}

	std::vector<float> inline GetScaleSigmaSquares(){
	    return mvLevelSigma2;
	}

	std::vector<float> inline GetInverseScaleSigmaSquares(){
	    return mvInvLevelSigma2;
	}

	std::vector<cv::Mat> mvImagePyramid;// 图像金字塔 每一层 一张图像 大小   H* 1/sc   W*1/sc

    protected:
     
        // 计算图像金字塔
	void ComputePyramid(cv::Mat image);
	// 通过八叉树的方式计算特征点
	void ComputeKeyPointsOctTree(std::vector<std::vector<cv::KeyPoint> >& allKeypoints);    
	
	// 通过八叉树的方式分配特征点
	std::vector<cv::KeyPoint> DistributeOctTree(const std::vector<cv::KeyPoint>& vToDistributeKeys, const int &minX,
					      const int &maxX, const int &minY, const int &maxY, const int &nFeatures, const int &level);

	void ComputeKeyPointsOld(std::vector<std::vector<cv::KeyPoint> >& allKeypoints);
	std::vector<cv::Point> pattern;

	int nfeatures;// 需要的特征点总数
	double scaleFactor;// 尺度因子
	// 金字塔共n层，与SIFT不同，每层仅有一副图像；
	//  第s层的尺度为sc= Fator^c，Fator初始尺度(默认为1.2)，原图在第0层；sc0= 1  sc1 = 1.2^1 sc2 =1.2 ^2
	// 每一层图像大小 H* 1/sc   W*1/sc
	// 在每一层上按公式计算需要提取的特征点数n,在本层上按Fast角点响应值排序，提取前2n个特征点，
	// 然后根据Harris   角点响应值排序， 取前n个特征点，作为本层的特征点；
	int nlevels;//金字塔层数
	int iniThFAST;//FAST 角点提取 阈值
	int minThFAST;//FAST 角点提取 最小 阈值

	std::vector<int> mnFeaturesPerLevel;// 每一层 特征点 个数

	std::vector<int> umax; //用于计算特征方向时，每个v坐标对应最大的u坐标

	std::vector<float> mvScaleFactor;//  每一层级的 尺度因子sc
	std::vector<float> mvInvScaleFactor; // 每一层级的 尺度因子 倒数  1/sc   是乘以 原图像尺寸的
	std::vector<float> mvLevelSigma2;//  尺度因子的 平方
	std::vector<float> mvInvLevelSigma2;// // 尺度因子 平方 的 倒数
    };

} //namespace ORB_SLAM

#endif

