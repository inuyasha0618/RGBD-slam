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

void computeKeyPointAndDesp(FRAME& frame)
{
    cv::Ptr<cv::ORB> orbDetector = cv::ORB::create();

    orbDetector->detect(frame.rgb, frame.kps);

    cout << frame.frameName << " keyPoints: " << frame.kps.size() << endl;

    cv::Mat imgShow_kps;
    cv::drawKeypoints(frame.rgb, frame.kps, imgShow_kps, cv::Scalar::all(-1), cv::DrawMatchesFlags::DEFAULT);
    cv::imshow(frame.frameName, imgShow_kps);
    // cv::imwrite("./data/features_rgb1.png", imgShow_rgb1);
    cv::waitKey(0);

    orbDetector->compute(frame.rgb, frame.kps, frame.desp);

}

RESULT_OF_PNP estimateMotion(FRAME& frame1, FRAME& frame2, CAMERA_INTRINSIC_PARAMS& camera)
{
    // validGoodMatches 用于存放第一张图中depth不为０的matches
    vector<cv::DMatch> matches, goodMatches, validGoodMatches;
    cv::BFMatcher matcher(cv::NORM_HAMMING);
    matcher.match(frame1.desp, frame2.desp, matches, cv::noArray());

    cv::Mat matchesImg;
    
    cv::drawMatches(frame1.rgb, frame1.kps, frame2.rgb, frame2.kps, matches, matchesImg);
    cv::imshow("matches", matchesImg);
    cv::waitKey(0);

    //　过滤一下匹配
    double minDistance = 9999999;
    for (size_t i = 0; i < matches.size(); ++i) {
        if (matches[i].distance < minDistance) {
            minDistance = matches[i].distance;
        }
    }

    cout << "min distance: " << minDistance << endl;

    for (size_t j = 0; j < matches.size(); ++j) {
        if (matches[j].distance < max(2 * minDistance, 30.0)) {
            goodMatches.push_back(matches[j]);
        }
    }

    cout << "good matches size: " << goodMatches.size() << endl;

    cv::Mat goodMatchesImg;
    cv::drawMatches(frame1.rgb, frame1.kps, frame2.rgb, frame2.kps, goodMatches, goodMatchesImg);
    cv::imshow("good Matches", goodMatchesImg);
    cv::waitKey(0);

    // 第一帧三维点
    vector<cv::Point3f> pts_3d;
    //　第二帧像素点
    vector<cv::Point2f> pts_2d;

    // 构建ＰnP所需的三维点以及对应的二维点
    for (size_t k = 0; k < goodMatches.size(); ++k) {

        cv::Point2f pixel_in_rgb1 = frame1.kps[goodMatches[k].queryIdx].pt;
        ushort depth = frame1.depth.ptr<ushort>(int(pixel_in_rgb1.y))[int(pixel_in_rgb1.x)];

        //　如果这一点没有深度值，则跳过
        if (depth == 0) continue;
	validGoodMatches.push_back(goodMatches[k]);
        cv::Point3f pixel_depth_in_rgb1(pixel_in_rgb1.x, pixel_in_rgb1.y, depth);

        cv::Point2f pixel_in_rgb2 = frame2.kps[goodMatches[k].trainIdx].pt;

        pts_3d.push_back(point2dTo3d(pixel_depth_in_rgb1, camera));
        pts_2d.push_back(cv::Point2f(pixel_in_rgb2));
    }

    double camera_mx_data[3][3] = {
            {camera.fx, 0, camera.cx},
            {0, camera.fy, camera.cy},
            {0, 0, 1}
    };

    cv::Mat cameraMx(3, 3, CV_64F, camera_mx_data);
    cv::Mat rvec, tvec, inliers;

    cv::solvePnPRansac(pts_3d, pts_2d, cameraMx, cv::Mat(), rvec, tvec, false, 100, 1.0, 0.99, inliers);

    vector<cv::DMatch> final_goodmatches;

    cout << "final good matches: " << inliers.rows << endl;
    cout << "R=" << rvec << endl;
    cout << "t=" << tvec << endl;

    cout << "R rows: " << rvec.rows << " R cols: " << rvec.cols << endl;
    cout << "t rows: " << tvec.rows << " t cols: " << tvec.cols << endl;

    for (size_t l = 0; l < inliers.rows; ++l) {
        final_goodmatches.push_back(validGoodMatches[inliers.ptr<int>(l)[0]]);
    }

    cv::Mat finalMatchesImg;

    cv::drawMatches(frame1.rgb, frame1.kps, frame2.rgb, frame2.kps, final_goodmatches, finalMatchesImg);

    cv::imshow("经RANSAC过滤后的匹配结果", finalMatchesImg);
    cv::waitKey(0);
    
    RESULT_OF_PNP res;
    res.rvec = rvec;
    res.tvec = tvec;
    res.inlier_nums = inliers.rows;
    
    return res;
}

Eigen::Isometry3d getIsometry(cv::Mat& rvec, cv::Mat& tvec) {
    cv::Mat cv_rotation;
    Eigen::Matrix3d eigen_rotation;

    cv::Rodrigues(rvec, cv_rotation);
    cv::cv2eigen(cv_rotation, eigen_rotation);

    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
    T.rotate(eigen_rotation);
    T(0, 3) = tvec.at<double>(0, 0);
    T(1, 3) = tvec.at<double>(1, 0);
    T(2, 3) = tvec.at<double>(2, 0);

    return T;
}

PointCloud::Ptr jointPointCloud(PointCloud::Ptr original, FRAME& newFrame, Eigen::Isometry3d T, CAMERA_INTRINSIC_PARAMS& camera) {
    PointCloud::Ptr sumCloud(new PointCloud());
    PointCloud::Ptr newCloud = img2PointCloud(newFrame.rgb, newFrame.depth, camera);
    pcl::transformPointCloud(*original, *sumCloud, T.matrix());
    *sumCloud += *newCloud;

    return sumCloud;
}

CAMERA_INTRINSIC_PARAMS getCamera() {
    ParameterReader pd;
    CAMERA_INTRINSIC_PARAMS camera;
    camera.cx = atof(pd.getData("camera.cx").c_str());
    camera.cy = atof(pd.getData("camera.cy").c_str());
    camera.fx = atof(pd.getData("camera.fx").c_str());
    camera.fy = atof(pd.getData("camera.fy").c_str());
    camera.scale = atof(pd.getData("camera.scale").c_str());

    return camera;
}

FRAME readFrame(int index, ParameterReader pd) {
    FRAME frame;
    stringstream ss;
    string rgbDir = pd.getData("rgb_dir");
    string rgbExt = pd.getData("rgb_extension");
    string depthDir = pd.getData("depth_dir");
    string depthExt = pd.getData("depth_extension");

    string filename;
    ss<<rgbDir<<index<<rgbExt;
    ss>>filename;

    frame.rgb = cv::imread(filename);

    ss.clear();
    filename.clear();

    ss<<depthDir<<index<<depthExt;
    ss>>filename;

    frame.depth = cv::imread(filename, -1);


    return frame;
}