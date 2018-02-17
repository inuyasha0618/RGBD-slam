# pragma once

#include <fstream>
#include <vector>
using namespace std;

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

typedef pcl::PointXYZRGBA Point;
typedef pcl::PointCloud<Point> PointCloud;

struct CAMERA_INTRINSIC_PARAMS
{
    CAMERA_INTRINSIC_PARAMS(double cx, double cy, double fx, double fy, double scale): cx(cx), cy(cy), fx(fx), fy(fy), scale(scale) {}
    double cx, cy, fx, fy, scale;
};

// 帧结构
struct FRAME
{
  // 彩色图以及对应的深度图
  string frameName;
  cv::Mat rgb, depth;
  // 描述子（一大堆）
  cv::Mat desp;
  vector<cv::KeyPoint> kps; //关键点（一大堆）
};

// ＰnP结果
struct RESULT_OF_PNP
{
  cv::Mat rvec, tvec;
  // Todo: inliers;
  int inlier_nums;
};

PointCloud::Ptr img2PointCloud(cv::Mat rgb_img, cv::Mat depth_img, CAMERA_INTRINSIC_PARAMS& camera_params);

cv::Point3f point2dTo3d(cv::Point3f& point, CAMERA_INTRINSIC_PARAMS& camera_params);

void computeKeyPointAndDesp(FRAME& frame);

RESULT_OF_PNP estimateMotion(FRAME& frame1, FRAME& frame2, CAMERA_INTRINSIC_PARAMS& camera);