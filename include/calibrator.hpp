#include <opencv2/opencv.hpp>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include "camera.hpp"
#include <memory>
#include <pcl/visualization/cloud_viewer.h>

class calibrator
{
public:
    calibrator();
    void Run(cv::Mat &image, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud);
    void VisualizationCallback(pcl::visualization::PCLVisualizer &viz);
    void ShowImageAndReadKeyboardThread();

private:
    std::mutex _cloud_mutex; // 用于保护点云数据的互斥锁
    std::mutex _image_mutex; // 用于保护点云数据的互斥锁
    pcl::PointCloud<pcl::PointXYZ>::Ptr _boardcloud;
    pcl::PointCloud<pcl::PointXYZ>::Ptr _cloud;
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> _green;
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> _red;
    cv::Mat _image;
    cv::Mat _image_raw;
};