# 动态环境下 语义地图构建

完整代码 已上传

# 编译
	cd build
	cmake ..
	make


# 运行
    cd ../Examples/RGB-D/
	
    ./ty_rgbd ../../Vocabulary/ORBvoc.bin ./my_rgbd_ty_api_adj.yaml
    
    ./rgbd_tum ../../Vocabulary/ORBvoc.bin ./TUM3.yaml ../../../../data/f3_walking_xyz/ ../../../../data/f3_walking_xyz/associate.txt

# octomap地图可视化
	 octovis octomap.ot

# 新功能
    1. 光流/多视角几何 剔除 运动点
    2. 稠密重建 octomap地图
    3. 语义目标数据库构建
    4. octomap地图的保存与载入
    5. orbslam2 地图的保存与载入

#  改进
    1. 剔除相机运动的光流，计算 单应变换矩阵
    2. 在建立稠密octomap地图时，进行目标检测，构建语义目标数据库
    3. 2d检测 融合 3d点云
    
# 遗留问题

    1. 目标检出率低(更好的目标检测/实例分割模型、地图点和语义信息化挂钩，将语义信息加入到数据关联目标函数中)
    2. octomap的动态地图更新效果不太理想(octomap 概率更新探索，建图时直接剔除动态点)
    3. 地图大了之后，系统 响应变慢 (地图维护)
    4. 地图导航、路径规划、智能导航

# 参考 
    1. mobilenetv2-ssd-lite 目标检测
           https://github.com/Ewenwan/MVision/tree/master/CNN/HighPerformanceComputing/example
    2. 图漾RGBD相机
           https://github.com/Ewenwan/MVision/tree/master/stereo/RGBD/orb_slam2_rgbd
    3. oRB_SLAM2
           https://github.com/Ewenwan/MVision/tree/master/vSLAM/oRB_SLAM2
    4. Octomap建图参考
           https://github.com/Ewenwan/ORB-SLAM-RGBD-with-Octomap
    5. 实时目标检测、跟踪、定位
           https://github.com/Ewenwan/ros_object_analytics





