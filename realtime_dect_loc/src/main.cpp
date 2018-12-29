
#include "../include/ncnn_dect.h"// 目标检测
#include "../include/ty_camera.h"// ty相机类
#include "../include/XYZRGBPCViewer.hpp"// 点云显示
#include "../include/2d_3d_merge.h"   // 2d 3d融合  Merge

#include "../include/mergeSG.h"       // 点云分割聚类 后 融合2d  MergeSG


#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/ModelCoefficients.h>

#include <pcl/io/pcd_io.h>

// 计时模块
#include <sys/time.h>
#include <unistd.h>

#include <string>// c++11 std::to_string(j)

// 计算帧率
long time_last=0, time_now=0;
// 添加 计时
long getTimeUsec()
{
    
    struct timeval t;
    gettimeofday(&t,0);
    return (long)((long)t.tv_sec*1000*1000 + t.tv_usec);
}

int main(int argc, char* argv[])
{
    TYcamera* TYcamera_ptr = new(TYcamera);// 图漾相机类
    //cv::Mat* img_ptr = new(cv::Mat);// 图像
    cv::Mat img;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_ptr(new pcl::PointCloud<pcl::PointXYZRGB>());// 点云
    XYZRGBPCViewer* PCViewer= new(XYZRGBPCViewer);// 点云显示

    Detector* ncnn_detector_ptr = new(Detector);// ncnn mobilenetv2ssdlite 目标检测对象
    std::vector<Object> objects;// 2d 检测对象：边框rect，物体类别名object_name，置信度prob
    
   //  Merge* merge_ptr = new(Merge); // 2d 3d融合
    MergeSG* merge_ptr = new(MergeSG);  // 点云分割聚类 后 融合2d  MergeSG

    std::vector<Cluster> clusters; // 3d 检测对象: 2d 检测框 + 点云 + 点云中心点 + 点云范围


   //创建视窗对象并给标题栏设置一个名称“3D Viewer”并将它设置为boost::shared_ptr智能共享指针，
   // 这样可以保证指针在程序中全局使用，而不引起内存错误
   // boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer")); 

    pcl::visualization::PCLVisualizer viewer;
    viewer.setBackgroundColor(0.0, 0.0, 0.0);
    //viewer.setCameraPosition(0, 0, 0, 0, 0, 1, 0, -1, 0);
    viewer.setCameraPosition(
        0, 0, 0,                                // camera位置
        0, 0, 2.5,                              // view向量 ： 观察位置视角 2.5 米
        0, -1, 0                                // up向量  y方向调换
        );
    //viewer.resetCamera();
    //viewer.initCameraParameters ();
    viewer.addCoordinateSystem(0.5);
    //viewer.setPosition(0, 0);
    viewer.setSize(600, 600);
    //viewer.setSize(1280, 960);
    //pcl::PCDWriter pcdwriter;

    static volatile bool exit_main = false;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr obj_pt;
    while(!exit_main)
    {
	//TYcamera_ptr->run(point_ptr, img_ptr);// 获取一帧数据
	//ncnn_detector_ptr->Run(*img_ptr, objects);
        TYcamera_ptr->run(point_ptr, img);// 获取一帧数据
        //std::cout<< img.size() << std::endl;
        ncnn_detector_ptr->Run(img, objects);
        //cv::imshow("image", img);
	ncnn_detector_ptr->Show(img, objects);
        viewer.removeAllShapes();// 去除 上一帧 已经显示的形状
        if(objects.size()>0)
        {               
            merge_ptr->extract(objects, point_ptr, clusters);
            //viewer.removeAllShapes();// 去除 上一帧 已经显示的形状
	    //viewer.removeShape(cluster_large.object.object_name.c_str());
            for(int j=0; j<clusters.size(); j++)
            {
		    const Cluster & cluster = clusters[j];// 识别到的每一个点云团
		    const Object & obj = cluster.object;  // 2d 框信息
		    const Eigen::Vector4f & centroid = cluster.centroid;// 点云中心

		    fprintf(stderr, "3d ==== %s = %.5f at %.2f %.2f %.2f\n", 
		            obj.object_name.c_str(), obj.prob, centroid[0], centroid[1], centroid[2]);// 打印名字、置信度、中心点坐标

		    const Eigen::Quaternionf quat(Eigen::Quaternionf::Identity());// 姿态 四元素
                    std::string name = cluster.object.object_name + boost::chrono::to_string(j);      // 包围框的名字
		    viewer.addCube(cluster.boxCenter, quat, 
				   cluster.sizePt[0], cluster.sizePt[1], cluster.sizePt[2], 
				   name.c_str());

		    viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, 
				                       pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, 
				                       name.c_str());
		    viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, 
				                        name.c_str());// 红色框
            }
        }

        pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(point_ptr);
        viewer.removePointCloud("Point3D");// 去除原来的点云====
        viewer.addPointCloud<pcl::PointXYZRGB> (point_ptr, rgb, "Point3D");
        viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "Point3D");	       
        //obj_pt = cluster_large.c_ptr;
        viewer.spinOnce ();

        if(viewer.wasStopped())
        {
            //pcdwriter.write<pcl::PointXYZRGB>("3dobject.pcd", *obj_pt);   
            exit_main = true; // 退出
            //return;
        }
/*
	PCViewer->show(point_ptr, "Point3D");
        if(PCViewer->isStopped("Point3D")){// 点云界面被关闭===
            exit_main = true; // 退出
            return;
        }
*/

        int key = cv::waitKey(1);
        switch(key)
        {
            case -1:
                break;
            case 'q': 
            case 1048576 + 'q':
                exit_main = true;
                break;
            //case 's': 
            //case 1048576 + 's':
            //    save_frame = true;
            //    break;
            default:
                LOGD("Pressed key %d", key);
        }

    }
    delete TYcamera_ptr;
    delete PCViewer;
    delete ncnn_detector_ptr;
    delete merge_ptr;
    return 0;
}




