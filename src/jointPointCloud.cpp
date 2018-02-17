#include <iostream>
#include "slamBase.h"

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
    
    
    CAMERA_INTRINSIC_PARAMS camera = getCamera();
    
    RESULT_OF_PNP res = estimateMotion(frame1, frame2, camera);
    
    cout << "inlier_nums: " << res.inlier_nums << endl;

    Eigen::Isometry3d T = getIsometry(res.rvec, res.tvec);

    PointCloud::Ptr cloud1 = img2PointCloud(frame1.rgb, frame1.depth, camera);

    PointCloud::Ptr sum = jointPointCloud(cloud1, frame2, T, camera);
    pcl::io::savePCDFile("data/jointCloud.pcd", *sum);
    cout << "joint complete!" << endl;

    return 0;
}