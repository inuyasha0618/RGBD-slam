#include <iostream>
#include <string>

using namespace std;

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

typedef pcl::PointXYZRGBA Point;
typedef pcl::PointCloud<Point> PointCloud;

const double camera_factor = 1000;
const double cx = 325.5, cy = 253.5, fx = 518.0, fy = 519.0;

string rgb_path = "./data/rgb.png";
string depth_path = "./data/depth.png";

int main(int argc, char** argv) {
    cv::Mat rgb_image = cv::imread(rgb_path);
    cv::Mat depth_image = cv::imread(depth_path, -1);

    PointCloud::Ptr cloud(new PointCloud);

    for (int row = 0; row < depth_image.rows; ++row) {
        for (int col = 0; col < depth_image.cols; ++col) {
            Point p;
            ushort depth = depth_image.ptr<ushort>(row)[col];
            if (depth == 0) continue;
            p.z = double(depth) / camera_factor;
            p.x = (col - cx) * p.z / fx;
            p.y = (row - cy) * p.z / fy;

            p.b = rgb_image.ptr<uchar>(row)[3 * col];
            p.g = rgb_image.ptr<uchar>(row)[3 * col + 1];
            p.r = rgb_image.ptr<uchar>(row)[3 * col + 2];

            cloud->points.push_back(p);
        }
    }
    cloud->height = 1;
    cloud->width = cloud->points.size();
    cloud->is_dense = false;
    cout << "点云size: " << cloud->points.size() << endl;
    pcl::io::savePCDFile("./output.pcd", *cloud);
    cloud->points.clear();
    return 0;
}