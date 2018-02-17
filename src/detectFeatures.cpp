#include <iostream>
#include "slamBase.h"

using namespace std;

const double cx = 325.5;
const double cy = 253.5;
const double fx = 518.0;
const double fy = 519.0;
const double scale = 1000.0;

int main(int argc, char** argv) {
    cv::Mat rgb1 = cv::imread("./data/rgb1.png");
//     cv::Mat rgb2 = cv::imread("./data/rgb2.png");
    cv::Mat depth1 = cv::imread("./data/depth1.png", -1);
//     cv::Mat depth2 = cv::imread("./data/depth2.png", -1);

//     cv::Ptr<cv::ORB> orbDetector = cv::ORB::create();
// 
//     vector<cv::KeyPoint> kps1, kps2;
//     cv::Mat descriptors_1, descriptors_2;
// 
//     orbDetector->detect(rgb1, kps1);
//     orbDetector->detect(rgb2, kps2);
// 
//     cout << "rgb1 keyPoints: " << kps1.size() << endl;
//     cout << "rgb2 keyPoints: " << kps2.size() << endl;
// 
//     cv::Mat imgShow_rgb1;
//     cv::drawKeypoints(rgb1, kps1, imgShow_rgb1, cv::Scalar::all(-1), cv::DrawMatchesFlags::DEFAULT);
//     cv::imshow("rgb1的特征点", imgShow_rgb1);
//     cv::imwrite("./data/features_rgb1.png", imgShow_rgb1);
//     cv::waitKey(0);
// 
//     orbDetector->compute(rgb1, kps1, descriptors_1);
//     orbDetector->compute(rgb2, kps2, descriptors_2);
    FRAME frame1, frame2;
    frame1.frameName = "rgb1";
    frame1.rgb = rgb1;
    frame1.depth = depth1;
    
    frame2.frameName = "rgb2";
    frame2.rgb = cv::imread("./data/rgb2.png");
    frame2.depth = cv::imread("./data/depth2.png", -1);
    
    computeKeyPointAndDesp(frame1);
    computeKeyPointAndDesp(frame2);
    
    
    CAMERA_INTRINSIC_PARAMS camera(cx, cy, fx, fy, scale);
    
    RESULT_OF_PNP res = estimateMotion(frame1, frame2, camera);
    

//     vector<cv::DMatch> matches, goodMatches;
//     cv::BFMatcher matcher(cv::NORM_HAMMING);
//     matcher.match(descriptors_1, descriptors_2, matches, cv::noArray());
// 
//     cv::Mat matchesImg;
// 
//     cv::drawMatches(rgb1, kps1, rgb2, kps2, matches, matchesImg);
//     cv::imshow("matches", matchesImg);
//     cv::imwrite("./data/matches.png", matchesImg);
//     cv::waitKey(0);
// 
//     //　过滤一下匹配
//     double minDistance = 9999999;
//     for (size_t i = 0; i < matches.size(); ++i) {
//         if (matches[i].distance < minDistance) {
//             minDistance = matches[i].distance;
//         }
//     }
// 
//     cout << "min distance: " << minDistance << endl;
// 
//     for (size_t j = 0; j < matches.size(); ++j) {
//         if (matches[j].distance < max(2 * minDistance, 30.0)) {
//             goodMatches.push_back(matches[j]);
//         }
//     }
// 
//     cout << "good matches size: " << goodMatches.size() << endl;
// 
//     cv::Mat goodMatchesImg;
//     cv::drawMatches(rgb1, kps1, rgb2, kps2, goodMatches, goodMatchesImg);
//     cv::imshow("good Matches", goodMatchesImg);
//     cv::imwrite("./data/good_matches.png", goodMatchesImg);
//     cv::waitKey(0);
// 
//     // 第一帧三维点
//     vector<cv::Point3f> pts_3d;
//     //　第二帧像素点
//     vector<cv::Point2f> pts_2d;
// 
//     CAMERA_INTRINSIC_PARAMS camera(cx, cy, fx, fy, scale);
// 
//     // 构建ＰnP所需的三维点以及对应的二维点
//     for (size_t k = 0; k < goodMatches.size(); ++k) {
// 
//         cv::Point2f pixel_in_rgb1 = kps1[goodMatches[k].queryIdx].pt;
//         ushort depth = depth1.ptr<ushort>(int(pixel_in_rgb1.y))[int(pixel_in_rgb1.x)];
// 
//         //　如果这一点没有深度值，则跳过
//         if (depth == 0) continue;
// 
//         cv::Point3f pixel_depth_in_rgb1(pixel_in_rgb1.x, pixel_in_rgb1.y, depth);
// 
//         cv::Point2f pixel_in_rgb2 = kps2[goodMatches[k].trainIdx].pt;
// 
//         pts_3d.push_back(point2dTo3d(pixel_depth_in_rgb1, camera));
//         pts_2d.push_back(cv::Point2f(pixel_in_rgb2));
//     }
// 
//     double camera_mx_data[3][3] = {
//             {fx, 0, cx},
//             {0, fy, cy},
//             {0, 0, 1}
//     };
// 
//     cv::Mat cameraMx(3, 3, CV_64F, camera_mx_data);
//     cv::Mat rvec, tvec, inliers;
// 
//     cv::solvePnPRansac(pts_3d, pts_2d, cameraMx, cv::Mat(), rvec, tvec, false, 100, 1.0, 0.99, inliers);
// 
//     vector<cv::DMatch> final_goodmatches;
// 
//     cout << "final good matches: " << inliers.rows << endl;
//     cout << "R=" << rvec << endl;
//     cout << "t=" << tvec << endl;
// 
//     for (size_t l = 0; l < inliers.rows; ++l) {
//         final_goodmatches.push_back(goodMatches[inliers.ptr<int>(l)[0]]);
//     }
// 
//     cv::Mat finalMatchesImg;
// 
//     cv::drawMatches(rgb1, kps1, rgb2, kps2, final_goodmatches, finalMatchesImg);
// 
//     cv::imshow("经RANSAC过滤后的匹配结果", finalMatchesImg);
//     cv::imwrite("./data/final_good_matches.png", finalMatchesImg);
//     cv::waitKey(0);

    return 0;
}