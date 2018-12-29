cmake_minimum_required(VERSION 2.8.10)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE release CACHE STRING "Choose the type of build" FORCE)
endif()

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")  

project(3d_object_dect)

# 寻找OpenCV库
find_package( OpenCV REQUIRED )
# 添加头文件
include_directories( ${OpenCV_INCLUDE_DIRS} )

# 找到PCL
find_package(PCL REQUIRED)
# 添加头文件
include_directories( ${PCL_INCLUDE_DIRS} )
link_directories(${PCL_LIBRARY_DIRS})
add_definitions( ${PCL_DEFINITIONS} )

# 添加 ncnn头文件
include_directories(${CMAKE_SOURCE_DIR}/include/ncnn/include)
# 添加ncnn 链接库
#link_directories("./ncnn/lib")
#link_libraries(${CMAKE_SOURCE_DIR}/ncnn/lib/libncnn.so)

# 添加 图漾相机 头文件
include_directories(${CMAKE_SOURCE_DIR}/include/ty/include)
include_directories(${CMAKE_SOURCE_DIR}/include/ty/common)

# 添加自定义类头文件
include_directories(${CMAKE_SOURCE_DIR}/include)

# 依赖库
set(libs 
${PCL_LIBRARIES} 
${OpenCV_LIBS}
-ltycam
-lncnn
-fopenmp
)

# 源文件
set(src_file 
src/ncnn_dect.cpp 
src/ty_camera.cpp 
src/XYZRGBPCViewer.cpp
src/2d_3d_merge.cpp
src/mergeSG.cpp
)

# 生成可执行文件=====
add_executable(${PROJECT_NAME} src/main.cpp ${src_file})
target_link_libraries(${PROJECT_NAME} ${libs})



