# pragma once

#include <fstream>
#include <sstream>
#include <vector>
#include <map>
using namespace std;

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <opencv2/core/core.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/cloud_viewer.h>

typedef pcl::PointXYZRGBA Point;
typedef pcl::PointCloud<Point> PointCloud;

struct CAMERA_INTRINSIC_PARAMS
{
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

Eigen::Isometry3d getIsometry(cv::Mat& rvec, cv::Mat& tvec);

PointCloud::Ptr jointPointCloud(PointCloud::Ptr original, FRAME& newFrame, Eigen::Isometry3d T, CAMERA_INTRINSIC_PARAMS& camera);

class ParameterReader {
public:
    ParameterReader(string filename = "./parameters.txt") {
        ifstream fin(filename.c_str());

        if (!fin) {
            cerr << "No parameter file" << endl;
        }

        while (!fin.eof()) {
            string currLine;
            getline(fin, currLine);

            if (currLine[0] == '#') {
                continue;
            }

            int pos = currLine.find("=");
            if (pos == -1) continue;

            string key = currLine.substr(0, pos);
            string value = currLine.substr(pos + 1, currLine.length());
            data[key] = value;

            if (!fin.good()) {
                break;
            }
        }
    }

    string getData(string key) {
        map<string, string>::iterator iter = data.find(key);
        if (iter == data.end()) {
            cerr << "Parameter name " << key << " not found!" << endl;
            return string("NOT_FOUND");
        }
        return iter->second;
    }


    map<string, string> data;
};


CAMERA_INTRINSIC_PARAMS getCamera();

FRAME readFrame(int index, ParameterReader pd);