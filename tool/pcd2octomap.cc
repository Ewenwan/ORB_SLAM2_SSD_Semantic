//
// Created by ichigoi7e on 13/07/2018.
//

#include <iostream>
#include <assert.h>

//pcl
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

//octomap
#include <octomap/octomap.h>
#include <octomap/ColorOcTree.h>

using namespace std;

int main(int argc, char** argv)
{
    if(argc != 3) {
        cout << "Usage: pcd2colorOctomap <input_file> <output_file>" << endl;
        return -1;
    }

    string input_file = argv[1];
    string output_file = argv[2];
    pcl::PointCloud<pcl::PointXYZRGBA> cloud;
    pcl::io::loadPCDFile<pcl::PointXYZRGBA> (input_file, cloud);

    cout << "point cloud loaded, point size = " << cloud.points.size() << endl;

    //声明octomap变量
    cout << "copy data into octomap..." << endl;
    //创建带颜色的八叉树对象，参数为分辨率，这里设成了0.04
    octomap::ColorOcTree tree( 0.04 );

    for(auto p:cloud.points) {
        //将点云里的点插入到octomap中
        tree.updateNode(octomap::point3d(p.x, p.y, p.z), true);
    }

    //设置颜色
    for(auto p:cloud.points) {
        tree.integrateNodeColor(p.x, p.y, p.z, p.r, p.g, p.b);
    }

    //更新octomap
    tree.updateInnerOccupancy();
    //存储octomap, 注意要存成.ot文件而非.bt文件
    tree.write(output_file);
    cout << "done." << endl;

    return 0;
}