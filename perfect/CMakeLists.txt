#cmake 版本限制
cmake_minimum_required(VERSION 2.8)
#工程名
project(ORB_SLAM2_pc)

SET(CMAKE_EXPORT_COMPILE_COMMANDS "ON")
# 使用此选项，cmake会生成一个JSON文件，其中包含包含路径
# 生成一个JSON编译数据库。
# 编译数据库JSON文件是在 cmake 执行时生成的，而不是在 make 编译 时生成。

#编译模式 使用 IF(NOT ) ENDIF 放置重复设置
IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE Release)
ENDIF()

# 显示 编译模式信息
MESSAGE("Build type: " ${CMAKE_BUILD_TYPE})

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O3 -march=native ")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall   -O3 -march=native")

# 检查c++11或者 c++0x 编译支持  Check C++11 or C++0x support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
   add_definitions(-DCOMPILEDWITHC11)
   message(STATUS "Using flag -std=c++11.")
elseif(COMPILER_SUPPORTS_CXX0X)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
   add_definitions(-DCOMPILEDWITHC0X)
   message(STATUS "Using flag -std=c++0x.")
else()
   message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

# 附加 模块cmakeList.txt
LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

#opencv  模块 版本大于 2.4.3
find_package(OpenCV 3.0 QUIET)
if(NOT OpenCV_FOUND)
   find_package(OpenCV 2.4.3 QUIET)
   if(NOT OpenCV_FOUND)
      message(FATAL_ERROR "OpenCV > 2.4.3 not found.")
   endif()
endif()

 # 矩阵 Eigen3
find_package(Eigen3 3.1.0 REQUIRED)
# 可视化gui Pangolin
find_package(Pangolin REQUIRED)

# 点云显示 adding for point cloud viewer and mapper
find_package( PCL 1.7 REQUIRED )

# octomap==================
FIND_PACKAGE( octomap REQUIRED )



#包含库文件
include_directories(
${PROJECT_SOURCE_DIR}
${PROJECT_SOURCE_DIR}/include
${EIGEN3_INCLUDE_DIR}
${Pangolin_INCLUDE_DIRS}# 可视化gui
${PCL_INCLUDE_DIRS}#点云库

${PROJECT_SOURCE_DIR}/Examples/RGB-D/ty/common/include # 图漾头文件===============
${PROJECT_SOURCE_DIR}/Thirdparty/ncnn/include          # ncnn 目标检测库
${OCTOMAP_INCLUDE_DIRS}# octomap
)

add_definitions( ${PCL_DEFINITIONS} )
link_directories( ${PCL_LIBRARY_DIRS} )

# 自建库生成路径
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)

# 创建共享库 SHARED 动态链接库
add_library(${PROJECT_NAME} SHARED
src/System.cc
src/Tracking.cc
src/LocalMapping.cc
src/LoopClosing.cc
src/ORBextractor.cc
src/ORBmatcher.cc
src/FrameDrawer.cc
src/Converter.cc
src/MapPoint.cc
src/KeyFrame.cc
src/Map.cc       # add 保存、载入orbslam 地图点、关键帧、最小生成树  
src/MapDrawer.cc # add 创建、显示、保存、载入octomap
src/Optimizer.cc
src/PnPsolver.cc
src/Frame.cc
src/KeyFrameDatabase.cc
src/Sim3Solver.cc
src/Initializer.cc
src/Viewer.cc
src/pointcloudmapping.cc # pcl点云建图线程，已丢弃==
src/Geometry.cc          # 多视角几何判断动静mask     较慢
src/Flow.cc              # 光流判断动静mask           快
src/ObjectDatabase.cc    # 语义目标数据库
src/Merge2d3d.cc         # 2d框和点云获取3d语义对象 深度信息滤波方法       快
src/MergeSG.cc           # 2d框和点云获取3d语义对象 点云分割聚类求交集方法  稍慢
src/Detector.cc          # 2d目标检测 / 实例分割 / 语义分割
src/RunDetect.cc         # 2d目标检测 运行线程 类 ，为了并行==
)

set(NCNN_TARGET_LIB ncnn) # ncnn 库
# 连接库文件
target_link_libraries(
${PROJECT_NAME}
${OpenCV_LIBS}
${EIGEN3_LIBS}
${Pangolin_LIBRARIES}
${PROJECT_SOURCE_DIR}/Thirdparty/DBoW2/lib/libDBoW2.so
${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o.so
${PCL_LIBRARIES}

${OCTOMAP_LIBRARIES} # octomap

${NCNN_TARGET_LIB}
-fopenmp
)


#可执行文件放入 /bin 目录
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

set(TY_TARGET_LIB tycam) # 图漾相机库==
# Build examples
# rgb-d 相机示例   图漾 ====
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/RGB-D)
add_executable(rgbd_ty
Examples/RGB-D/my_rgbd.cc)
target_link_libraries(rgbd_ty 
${PROJECT_NAME} 
${TY_TARGET_LIB}
)
# 多两个链接库

# 双目相机示例
#set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/Stereo)
#add_executable(stereo_kitti #KITTI 数据集
#Examples/Stereo/stereo_kitti.cc)
#target_link_libraries(stereo_kitti ${PROJECT_NAME})
#add_executable(stereo_euroc # EuRoC数据集
#Examples/Stereo/stereo_euroc.cc)
#target_link_libraries(stereo_euroc ${PROJECT_NAME})

#add_executable(my_stereo # stereo_on_line
#Examples/Stereo/my_stereo.cc)
#target_link_libraries(my_stereo ${PROJECT_NAME})


#单目相机示例 
#set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/Monocular)
#add_executable(mono_tum # TUM数据集
#Examples/Monocular/mono_tum.cc)
#target_link_libraries(mono_tum ${PROJECT_NAME})
#add_executable(mono_kitti #KITTI 数据集
#Examples/Monocular/mono_kitti.cc)
#target_link_libraries(mono_kitti ${PROJECT_NAME})
#3add_executable(mono_euroc # EuRoC数据集
#Examples/Monocular/mono_euroc.cc)
#target_link_libraries(mono_euroc ${PROJECT_NAME})

## .txt >>> .bin 文件转换
#set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/tools)
#add_executable(
#bin_vocabulary
#tools/bin_vocabulary.cc)
#target_link_libraries(bin_vocabulary ${PROJECT_NAME})

