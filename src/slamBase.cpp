#include "slamBase.h"

PointCloud::Ptr img2PointCloud(cv::Mat rgb_img, cv::Mat depth_img, CAMERA_INTRINSIC_PARAMS& camera_params) {
    PointCloud::Ptr cloud(new PointCloud);
    for (int row = 0; row < depth_img.rows; ++row) {
        for (int col = 0; col < depth_img.cols; ++col) {
            Point p;
            ushort depth = depth_img.ptr<ushort>(row)[col];
            if (depth == 0) continue;
            p.z = double(depth) / camera_params.scale;
            p.x = (col - camera_params.cx) * p.z / camera_params.fx;
            p.y = (row - camera_params.cy) * p.z / camera_params.fy;

            p.b = rgb_img.ptr<uchar>(row)[3 * col];
            p.g = rgb_img.ptr<uchar>(row)[3 * col + 1];
            p.r = rgb_img.ptr<uchar>(row)[3 * col + 2];

            cloud -> points.push_back(p);
        }
    }

    cloud -> height = 1;
    cloud -> width = cloud -> points.size();
    cloud -> is_dense = false;

    return cloud;
}

cv::Point3f point2dTo3d(cv::Point3f& point, CAMERA_INTRINSIC_PARAMS& camera_params) {
    cv::Point3f p;
    p.z = double(point.z) / camera_params.scale;
    p.x = (point.x - camera_params.cx) * p.z / camera_params.fx;
    p.y = (point.y - camera_params.cy) * p.z / camera_params.fy;
    return p;
}