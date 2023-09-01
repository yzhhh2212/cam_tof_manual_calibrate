#ifndef _CAMERA_H_
#define _CAMERA_H_
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
// #include <Eigen/Core>

class camera
{
public:
    camera();
    bool EstimateBoardPose(cv::Mat image);
    void ProjectBoard();
    pcl::PointCloud<pcl::PointXYZ>::Ptr ProjectChessboardToCameraSpace();


private:
    // chaseboard para
    int _col;
    int _row;
    float _square_size;
    std::vector<cv::Point3f> _p3ds;
    std::vector<cv::Point2f> _p2ds;

    Eigen::Matrix4d _Tcb;
    Eigen::Matrix4d _Tbc;
    cv::Mat intrinsics_;
    cv::Mat distCoeffs_;
};


#endif
