#include <opencv2/opencv.hpp>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include "camera.hpp"
#include <memory>
#include <pcl/visualization/cloud_viewer.h>
int load_image()
{
}
std::mutex cloud_mutex; // 用于保护点云数据的互斥锁
pcl::PointCloud<pcl::PointXYZ>::Ptr boardcloud(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

void run_viewer(boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer)
{
    while (!viewer->wasStopped())
    {
        std::lock_guard<std::mutex> lock(cloud_mutex); // 加锁
        viewer->spinOnce(100);
    }
}
void VisualizationCallback(pcl::visualization::PCLVisualizer &viz)
{
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> green(cloud, 0, 255, 0);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> red(cloud, 255, 0, 0);
    std::unique_lock<std::mutex> lockOnCallBack(cloud_mutex);
    {
        if (!viz.updatePointCloud(cloud, red, "tof_cloud"))
        {
            viz.addPointCloud(cloud, red, "tof_cloud");
        }

        if (!viz.updatePointCloud(boardcloud, green, "board_cloud"))
        {
            viz.addPointCloud(boardcloud, green, "board_cloud");
        }
    }
}

int main()
{
    std::string folderPath = "/usr/local/project/data/"; // 例如 "C:/images/"
    std::string fileExtension = ".jpg";                  // 图片的扩展名
    std::string pointcloudExtension = ".pcd";            // 图片的扩展名
    // boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    pcl::visualization::CloudViewer viewer("Cloud Viewer");
    std::shared_ptr<camera> frame(new camera());
    // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    // pcl::PointCloud<pcl::PointXYZ>::Ptr boardcloud(new pcl::PointCloud<pcl::PointXYZ>);
    // pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> green(cloud, 0, 255, 0);
    // pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> red(cloud, 255, 0, 0);
    // auto PclViewerCallBack = std::bind(VisualizationCallback, std::placeholders::_1, cloud, boardcloud);
    // viewer.runOnVisualizationThread(PclViewerCallBack);
    viewer.runOnVisualizationThread(VisualizationCallback);
    for (int i = 0; i <= 174; ++i)
    {
        std::stringstream ss;
        std::stringstream sspc;
        ss << folderPath << i << fileExtension; // 拼接完整的文件路径
        sspc << folderPath << i << pointcloudExtension;
        std::string filePath = ss.str();
        std::string pointcloudPath = sspc.str();

        cv::Mat gray;

        cv::Mat image = cv::imread(filePath, cv::IMREAD_COLOR); // 读取图片
        cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
        if (!frame->EstimateBoardPose(image))
            continue;
        cv::imshow("image", image);

        // 更新点云数据
        {
            std::unique_lock<std::mutex> lockOnMain(cloud_mutex);
            if (!cloud->empty())
                cloud->clear();
            if (pcl::io::loadPCDFile<pcl::PointXYZ>(pointcloudPath, *cloud) == -1)
            {
                std::cout << "无法打开或找到PCD文件: " << pointcloudPath << std::endl;
                return -1;
            }
            if (!cloud->empty())
                boardcloud->clear();

            boardcloud = frame->ProjectChessboardToCameraSpace();
            // viewer->removePointCloud("tof cloud");
            // viewer->removePointCloud("board cloud");
            // viewer->addPointCloud<pcl::PointXYZ>(cloud, red, "tof cloud");
            // viewer->addPointCloud<pcl::PointXYZ>(boardcloud, green, "board cloud");
            // 刷新视图
            // viewer->spinOnce(100);
        }
        // 检查用户是否按下了“q”键
        // if (viewer->wasStopped())
        // {
        //     break;
        // }

        int key = cv::waitKey(2000); // 等待500毫秒（0.5秒）

        if (key == 27)
        { // 按下ESC键退出
            break;
        }
    }
}
