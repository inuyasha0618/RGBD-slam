#include <iostream>
#include <string>

#include "slamBase.h"

using namespace std;

const double camera_factor = 1000;
const double cx = 325.5, cy = 253.5, fx = 518.0, fy = 519.0;

string rgb_path = "./data/rgb.png";
string depth_path = "./data/depth.png";

int main(int argc, char** argv) {
    cv::Mat rgb_image = cv::imread(rgb_path);
    cv::Mat depth_image = cv::imread(depth_path, -1);

    CAMERA_INTRINSIC_PARAMS camera(cx, cy, fx, fy, camera_factor);

    PointCloud::Ptr cloud = img2PointCloud(rgb_image, depth_image, camera);

    cout << "点云size: " << cloud->points.size() << endl;
    pcl::io::savePCDFile("./output.pcd", *cloud);
    cloud->points.clear();
    return 0;
}