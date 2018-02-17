#include <iostream>
#include "slamBase.h"
#include <opencv2/core/eigen.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <pcl/common/transforms.h>
#include <pcl/visualization/cloud_viewer.h>

using namespace std;

const double cx = 325.5;
const double cy = 253.5;
const double fx = 518.0;
const double fy = 519.0;
const double scale = 1000.0;

int main(int argc, char** argv) {

    FRAME frame1, frame2;
    frame1.frameName = "rgb1";
    frame1.rgb = cv::imread("./data/rgb1.png");
    frame1.depth = cv::imread("./data/depth1.png", -1);
    
    frame2.frameName = "rgb2";
    frame2.rgb = cv::imread("./data/rgb2.png");
    frame2.depth = cv::imread("./data/depth2.png", -1);
    
    computeKeyPointAndDesp(frame1);
    computeKeyPointAndDesp(frame2);
    
    
    CAMERA_INTRINSIC_PARAMS camera(cx, cy, fx, fy, scale);
    
    RESULT_OF_PNP res = estimateMotion(frame1, frame2, camera);
    
    cout << "inlier_nums: " << res.inlier_nums << endl;

    // 计算用Eigen表示的旋转矩阵
    cv::Mat R;
    cv::Rodrigues(res.rvec, R);
    Eigen::Matrix3d R_in_Eigen;
    cv::cv2eigen(R, R_in_Eigen);

    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();

    T.rotate(R_in_Eigen);
    T(0, 3) = res.tvec.at<double>(0, 0);
    T(1, 3) = res.tvec.at<double>(1, 0);
    T(2, 3) = res.tvec.at<double>(2, 0);

    PointCloud::Ptr cloud1 = img2PointCloud(frame1.rgb, frame1.depth, camera);
    PointCloud::Ptr cloud2 = img2PointCloud(frame2.rgb, frame2.depth, camera);

    PointCloud::Ptr sum (new PointCloud());
    pcl::transformPointCloud(*cloud1, *sum, T.matrix());
    *sum += *cloud2;
    pcl::io::savePCDFile("data/jointCloud.pcd", *sum);
    cout << "joint complete!" << endl;

    return 0;
}