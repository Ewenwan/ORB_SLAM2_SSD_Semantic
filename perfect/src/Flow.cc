/* This file is part of ORB-SLAM2-SSD-Semantic.
利用光流来 估计运动点
*/
#include "Flow.h"

namespace FlowSLAM
{
	Flow::Flow()
	{
	  // mBInaryThreshold=4.0;
	}

// 未考虑相机运动，直接计算光流=======
	void Flow::ComputeMask(const cv::Mat& GrayImg, cv::Mat& mask, 
                               float BInaryThreshold)
	{
         if(!GrayImg.empty())
         {
          if(BInaryThreshold<40.0) BInaryThreshold=40.0;
          mask = cv::Mat::ones(GrayImg.rows,GrayImg.cols,CV_8U);// 默认1,静止的物体
          pyrDown(GrayImg, mImGrayCurrent, Size(GrayImg.cols / 2, GrayImg.rows / 2));
          // 下采样加快速度======
          if( mImGrayLast.data ) 
	  { 
            cv::Mat flow,flow2;
            calcOpticalFlowFarneback(mImGrayLast, mImGrayCurrent, flow, 0.5, 3, 15, 3, 5, 1.2, 0); 
            pyrUp(flow, flow2, Size(flow.cols * 2, flow.rows * 2));// 上采样回 原尺寸

            for(int y=0; y<flow2.rows; y++) // 每隔5行 y+= 5
            { 
             for(int x=0; x<flow2.cols; x++)
              { 
                const Point2f xy = flow2.at<Point2f>(y, x);
                float tep2 = xy.x * xy.x + xy.y * xy.y ;
	       // float tep = sqrt(xy.x * xy.x + xy.y * xy.y); 
               // 调整这个阈值可以适当降低 相机移动的影响，不过又会对 小运动的物体 灵敏度下降。。
	        if(tep2 < BInaryThreshold) // 剔除光流过小的 光流值 >8*8
	          continue; 
                mask.at<unsigned char>(y,x) = 0;//光流值过大的，为运动物体区域
              } 
            }
	    int dilation_size = 10;// 核大小===
	    cv::Mat kernel = getStructuringElement(cv::MORPH_ELLIPSE,
			                       cv::Size( 2*dilation_size + 1, 2*dilation_size+1 ),
			                       cv::Point( dilation_size, dilation_size ) );
            cv::erode(mask, mask, kernel);// 腐蚀，选小的
            cv::erode(mask, mask, kernel);// 腐蚀，选小的
	    cv::dilate(mask, mask, kernel);// 膨胀，选大的

          }
          std::swap(mImGrayLast, mImGrayCurrent); // 更新上一帧灰度图
         }
	}

// CalcOpticalFlowFarneback()函数是利用用Gunnar Farneback的算法,
// 计算全局性的稠密光流算法（即图像上所有像素点的光流都计算出来），
// 由于要计算图像上所有点的光流，故计算耗时，速度慢。
// 参数说明如下： 
// _prev0：输入前一帧图像 
// _next0：输入后一帧图像 
// _flow0：输出的光流 
// pyr_scale：金字塔上下两层之间的尺度关系 
// levels：金字塔层数 
// winsize：均值窗口大小，越大越能denoise并且能够检测快速移动目标，但会引起模糊运动区域 
// iterations：迭代次数
 // poly_n：像素领域大小，一般为5，7等 
// poly_sigma：高斯标注差，一般为1-1.5 
// flags：计算方法。主要包括 OPTFLOW_USE_INITIAL_FLOW 和 OPTFLOW_FARNEBACK_GAUSSIAN 


// 使用匹配点对计算 单元变换 对 mImGrayCurrent 进行 反单应变换后 再进行 光流计算===
// 考虑到 运行速度 暂时不 编写了
	void Flow::ComputeMask(const cv::Mat& GrayImg, const cv::Mat& Homo, 
                               cv::Mat& mask, float BInaryThreshold)
	{
             cv::Mat dest;
             //  myWarpPerspective(GrayImg, dest, Homo);  // 段错误 H矩阵维度???
             cv::warpPerspective( GrayImg, dest, Homo, GrayImg.size() );
             ComputeMask(dest, mask, BInaryThreshold);
        }


// 对帧图进行单应矩阵反变换 去除相机移动的影响==================
void Flow::myWarpPerspective(const cv::Mat& src, cv::Mat& dst, const cv::Mat& H)
{
    if(!src.empty() && !H.empty())
    {
	dst= cv::Mat::zeros(src.rows,src.cols,src.type());//先全部置零
	float u = 0,v = 0;//i，j就是变换后图像上的点；u，v就是原图像上的点
	for (int i = 0; i < dst.rows; i++)
	{
	  for (int j = 0; j < dst.cols; j++)
	  {
// 这里 可能有问题===H矩阵维度???
	     u = H.at<float>(0, 0)*i + H.at<float>(0, 1)*j + H.at<float>(0, 2);//cols
             v = H.at<float>(1, 0)*i + H.at<float>(1, 1)*j + H.at<float>(1, 2);//rows

	     if (int(u)>=0 && int(v)>=0 && int(u+0.5) < src.rows && int(v+0.5) < src.cols)//找到对应的原图像上的点是在原图像范围内
	     {
		if (float(u-int(u))<0.5)
		{
		  if (float( v - int(v)) < 0.5)
	            //1象限
		    dst.at<uchar>(i, j) = src.at<uchar>(int(u), int(v));
		  else
		    //2象限
		    dst.at<uchar>(i, j) = src.at<uchar>(int(u), int(v+0.5));
		}
		else
		{
		  if (float(v - int(v)) < 0.5)
		    //3象限
		    dst.at<uchar>(i, j) = src.at<uchar>(int(u+0.5), int(v));
		  else
		    //4象限
		    dst.at<uchar>(i, j) = src.at<uchar>(int(u+0.5), int(v+0.5));
		}

	     }
	   }
       }
    }
}


}


