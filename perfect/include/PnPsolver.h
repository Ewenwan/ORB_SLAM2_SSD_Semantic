/**
* This file is part of ORB-SLAM2.
 2d-3d点对求解变换
矩阵线性方程式求解，G2O优化式求解
这个PnP Solver 只在重定位里用了

//这里的pnp求解用的是EPnP的算法。
// 参考论文：EPnP:An Accurate O(n) Solution to the PnP problem
// https://en.wikipedia.org/wiki/Perspective-n-Point
// http://docs.ros.org/fuerte/api/re_vision/html/classepnp.html
// 如果不理解，可以看看中文的："摄像机位姿的高精度快速求解" "摄像头位姿的加权线性算法"

// PnP求解：已知世界坐标系下的3D点与图像坐标系对应的2D点，
// 求解相机的外参(R t)，即从世界坐标系到相机坐标系的变换。
// 而EPnP的思想是：
// 将世界坐标系所有的3D点用四个虚拟的控制点来表示，
// 将图像上对应的特征点转化为相机坐标系下的四个控制点
// 根据世界坐标系下的四个控制点与相机坐标系下对应的
// 四个控制点（与世界坐标系下四个控制点有相同尺度）即可恢复出(R t)


//                                   |x|
//   |u|   |fx r  u0||r11 r12 r13 t1||y|
// s |v| = |0  fy v0||r21 r22 r23 t2||z|
//   |1|   |0  0  1 ||r32 r32 r33 t3||1|

// step1:用四个控制点来表达所有的3D点
// p_w = sigma(alphas_j * pctrl_w_j), j从0到4
// p_c = sigma(alphas_j * pctrl_c_j), j从0到4
// sigma(alphas_j) = 1,  j从0到4

// step2:根据针孔投影模型
// s * u = K * sigma(alphas_j * pctrl_c_j), j从0到4

// step3:将step2的式子展开, 消去s
// sigma(alphas_j * fx * Xctrl_c_j) + alphas_j * (u0-u)*Zctrl_c_j = 0
// sigma(alphas_j * fy * Xctrl_c_j) + alphas_j * (v0-u)*Zctrl_c_j = 0

// step4:将step3中的12未知参数（4个控制点*3维参考点坐标）提成列向量
// Mx = 0,计算得到初始的解x后可以用Gauss-Newton来提纯得到四个相机坐标系的控制点

// step5:根据得到的p_w和对应的p_c，最小化重投影误差即可求解出R t

*/

#ifndef PNPSOLVER_H
#define PNPSOLVER_H

#include <opencv2/core/core.hpp>
#include "MapPoint.h"
#include "Frame.h"

namespace ORB_SLAM2
{

class PnPsolver {
 public:
  PnPsolver(const Frame &F, const vector<MapPoint*> &vpMapPointMatches);

  ~PnPsolver();

        
  // 设置RANSAC迭代的参数
        void SetRansacParameters(double probability = 0.99,
                                                               int minInliers = 8, 
                                                               int maxIterations = 300,
                                                               int minSet = 4,
                                                               float epsilon = 0.4,
                                                               float th2 = 5.991);


  cv::Mat find(vector<bool> &vbInliers, int &nInliers);

  cv::Mat iterate(int nIterations, bool &bNoMore, vector<bool> &vbInliers, int &nInliers);

 private:
// 通过之前求解的(R t)检查哪些3D-2D点对属于inliers
  void CheckInliers();
  bool Refine();

  // Functions from the original EPnP code
// number_of_correspondences为RANSAC每次PnP求解时时3D点和2D点匹配对数
// RANSAC需要很多次，maximum_number_of_correspondences为匹配对数最大值
// 这个变量用于决定pws us alphas pcs容器的大小，因此只能逐渐变大不能减小
// 如果maximum_number_of_correspondences之前设置的过小，则重新设置，
// 并重新初始化pws us alphas pcs的大小
  void set_maximum_number_of_correspondences(const int n);
  void reset_correspondences(void);
  void add_correspondence(const double X, const double Y, const double Z,
              const double u, const double v);

  double compute_pose(double R[3][3], double T[3]);

  void relative_error(double & rot_err, double & transl_err,
              const double Rtrue[3][3], const double ttrue[3],
              const double Rest[3][3],  const double test[3]);

  void print_pose(const double R[3][3], const double t[3]);
  double reprojection_error(const double R[3][3], const double t[3]);

  void choose_control_points(void);
// 求解四个控制点的系数alphas
// (a2 a3 a4)' = inverse(cws2-cws1 cws3-cws1 cws4-cws1)*(pws-cws1)，a1 = 1-a2-a3-a4
// 每一个3D控制点，都有一组alphas与之对应
// cws1 cws2 cws3 cws4为四个控制点的坐标
// pws为3D参考点的坐标
  void compute_barycentric_coordinates(void);
  void fill_M(CvMat * M, const int row, const double * alphas, const double u, const double v);
  void compute_ccs(const double * betas, const double * ut);
  void compute_pcs(void);

  void solve_for_sign(void);

  void find_betas_approx_1(const CvMat * L_6x10, const CvMat * Rho, double * betas);
  void find_betas_approx_2(const CvMat * L_6x10, const CvMat * Rho, double * betas);
  void find_betas_approx_3(const CvMat * L_6x10, const CvMat * Rho, double * betas);
  void qr_solve(CvMat * A, CvMat * b, CvMat * X);

  double dot(const double * v1, const double * v2);
  double dist2(const double * p1, const double * p2);

  void compute_rho(double * rho);
  void compute_L_6x10(const double * ut, double * l_6x10);

  void gauss_newton(const CvMat * L_6x10, const CvMat * Rho, double current_betas[4]);
  void compute_A_and_b_gauss_newton(const double * l_6x10, const double * rho,
				    double cb[4], CvMat * A, CvMat * b);

  double compute_R_and_t(const double * ut, const double * betas,
			 double R[3][3], double t[3]);

  void estimate_R_and_t(double R[3][3], double t[3]);

  void copy_R_and_t(const double R_dst[3][3], const double t_dst[3],
		    double R_src[3][3], double t_src[3]);

  void mat_to_quat(const double R[3][3], double q[4]);


  double uc, vc, fu, fv;

  double * pws, * us, * alphas, * pcs;
  int maximum_number_of_correspondences;
  int number_of_correspondences;

  double cws[4][3], ccs[4][3];
  double cws_determinant;

  vector<MapPoint*> mvpMapPointMatches;

  // 2D Points
  vector<cv::Point2f> mvP2D;
  vector<float> mvSigma2;

  // 3D Points
  vector<cv::Point3f> mvP3Dw;

  // Index in Frame
  vector<size_t> mvKeyPointIndices;

  // Current Estimation
  double mRi[3][3];
  double mti[3];
  cv::Mat mTcwi;
  vector<bool> mvbInliersi;
  int mnInliersi;

  // Current Ransac State
  int mnIterations;
  vector<bool> mvbBestInliers;
  int mnBestInliers;
  cv::Mat mBestTcw;

  // Refined
  cv::Mat mRefinedTcw;
  vector<bool> mvbRefinedInliers;
  int mnRefinedInliers;

  // Number of Correspondences
  int N;

  // Indices for random selection [0 .. N-1]
  vector<size_t> mvAllIndices;

  // RANSAC probability
  double mRansacProb;

  // RANSAC min inliers
  int mRansacMinInliers;

  // RANSAC max iterations
  int mRansacMaxIts;

  // RANSAC expected inliers/total ratio
  float mRansacEpsilon;

  // RANSAC Threshold inlier/outlier. Max error e = dist(P1,T_12*P2)^2
  float mRansacTh;

  // RANSAC Minimun Set used at each iteration
  int mRansacMinSet;

  // Max square error associated with scale level. Max error = th*th*sigma(level)*sigma(level)
  vector<float> mvMaxError;

};

} //namespace ORB_SLAM

#endif //PNPSOLVER_H
