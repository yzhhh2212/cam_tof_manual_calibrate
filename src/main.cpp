#include <opencv2/opencv.hpp>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include "camera.hpp"
#include <memory>
#include <pcl/visualization/cloud_viewer.h>

std::mutex cloud_mutex; // 用于保护点云数据的互斥锁
pcl::PointCloud<pcl::PointXYZ>::Ptr boardcloud(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

double delta_angle = 0.001;            // 欧拉角调整量
double delta_trans = 0.001;            // 平移调整量
Eigen::Vector3d euler_angles(0, 0, 0); // 欧拉角
Eigen::Vector3d translation(0, 0, 0);  // 平移向量

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

bool readYaml(Eigen::Matrix3d &Rcl, Eigen::Vector3d &tcl)
{
    std::ifstream f("/usr/local/project/keystar/cam_tof_manual/estimate_pose.yaml");
    if (f.good())
    {
        f.close();
        cv::FileStorage fs("/usr/local/project/keystar/cam_tof_manual/estimate_pose.yaml", cv::FileStorage::READ);
        // 你的其他代码

        if (!fs.isOpened())
        {
            std::cerr << "Failed to open the YAML file!" << std::endl;
            return false;
        }

        // 读取旋转四元数
        cv::FileNode rotationNode = fs["rotation"];
        double w = (double)rotationNode["w"];
        double x = (double)rotationNode["x"];
        double y = (double)rotationNode["y"];
        double z = (double)rotationNode["z"];

        // 读取平移向量
        cv::FileNode translationNode = fs["translation"];
        double tx = (double)translationNode["x"];
        double ty = (double)translationNode["y"];
        double tz = (double)translationNode["z"];

        // 将四元数转换为旋转矩阵（使用 Eigen 库）
        Eigen::Quaterniond quat(w, x, y, z);
        Rcl = quat.toRotationMatrix();

        // 创建平移向量（使用 Eigen 库）
        tcl << tx, ty, tz;

        // 输出读取到的值
        std::cout << "Rotation Matrix: \n"
                  << Rcl << std::endl;
        std::cout << "Translation Vector: \n"
                  << tcl << std::endl;

        return true;
    }
    else
    {
        std::cout << "文件不存在或无法打开" << std::endl;
    }
    // 创建一个 FileStorage 对象，用于读取 YAML 文件
}

int main()
{
    std::string folderPath = "/usr/local/project/data/"; // 例如 "C:/images/"
    std::string fileExtension = ".jpg";                  // 图片的扩展名
    std::string pointcloudExtension = ".pcd";            // 图片的扩展名
    pcl::visualization::CloudViewer viewer("Cloud Viewer");
    viewer.runOnVisualizationThread(VisualizationCallback);
    double w = 0.999334;
    double x = 0.00683661;
    double y = 0.0270457;
    double z = -0.0235022;

    camera::_Rcl_Estimate = Eigen::Quaterniond(w, x, y, z).toRotationMatrix();
    camera::_tcl_Estimate[0] = -0.0227738;
    camera::_tcl_Estimate[1] = -0.0660415;
    camera::_tcl_Estimate[2] = 0.0939607;
    camera::_Tcl_Estimate.setIdentity();
    camera::_Tcl_Estimate.block<3, 3>(0, 0) = camera::_Rcl_Estimate; // 设置旋转部分
    camera::_Tcl_Estimate.block<3, 1>(0, 3) = camera::_tcl_Estimate; // 设置平移部分
    camera::_Tlc_Estimate = camera::_Tcl_Estimate.inverse();
    camera::_Tcl_manual = camera::_Tcl_Estimate;
    camera::_Rcl_manual = camera::_Rcl_Estimate;
    camera::_tcl_manual = camera::_tcl_Estimate;
    camera::_Tlc_manual = camera::_Tlc_Estimate;

    for (int i = 0; i <= 174; ++i)
    {
        std::shared_ptr<camera> frame(new camera());
        std::stringstream ss;
        std::stringstream sspc;
        ss << folderPath << i << fileExtension; // 拼接完整的文件路径
        sspc << folderPath << i << pointcloudExtension;
        std::string filePath = ss.str();
        std::string pointcloudPath = sspc.str();

        cv::Mat gray;

        cv::Mat image = cv::imread(filePath, cv::IMREAD_COLOR);     // 读取图片
        cv::Mat raw_image = cv::imread(filePath, cv::IMREAD_COLOR); // 读取图片
        cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
        if (!frame->EstimateBoardPose(image))
            continue;

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
        }
        frame->Project3DTo2D(image, cloud);
        cv::imshow("image", image);
        int key = cv::waitKey(500); // 等待500毫秒（0.5秒）
        std::cout << "Do you wanna use image " << i << " to calibrate? Press [Y] / [N] " << std::endl;
        int newKey = cv::waitKey(0);
        if (newKey == 27)
        { // 按下ESC键退出
            break;
        }
        else if (newKey == 110 || newKey == 78)
        {
            continue;
        }
        else if (newKey == 121 || newKey == 89)
        {
            {
                std::unique_lock<std::mutex> lockOnCallBack(cloud_mutex);
                boardcloud = frame->ChessboardFromCam2TofEstimate();
            }
            cv::Mat image_tmp = raw_image.clone();
            frame->UpdatePose(camera::_Tcl_manual, image_tmp, cloud);
            while (1)
            {
                int newkey2 = cv::waitKey(0);
                if (newkey2 == 110 || newkey2 == 78)
                {
                    std::cout << "按下了 N 或 n，退出循环。" << std::endl;
                    break;
                }
                if (newkey2 == 'q')
                {
                    euler_angles(0) = delta_angle;
                    std::cout << "按下了 Q，X 轴的欧拉角增加了 " << delta_angle << " 度。" << std::endl;
                }
                if (newkey2 == 'a')
                {
                    euler_angles(0) = -delta_angle;
                    std::cout << "按下了 A，X 轴的欧拉角减少了 " << delta_angle << " 度。" << std::endl;
                }
                if (newkey2 == 'w')
                {
                    euler_angles(1) = delta_angle;
                    std::cout << "按下了 W，Y 轴的欧拉角增加了 " << delta_angle << " 度。" << std::endl;
                }
                if (newkey2 == 's')
                {
                    euler_angles(1) = -delta_angle;
                    std::cout << "按下了 S，Y 轴的欧拉角减少了 " << delta_angle << " 度。" << std::endl;
                }
                if (newkey2 == 'e')
                {
                    euler_angles(2) = delta_angle;
                    std::cout << "按下了 E，Z 轴的欧拉角增加了 " << delta_angle << " 度。" << std::endl;
                }
                if (newkey2 == 'd')
                {
                    euler_angles(2) = -delta_angle;
                    std::cout << "按下了 D，Z 轴的欧拉角减少了 " << delta_angle << " 度。" << std::endl;
                }
                if (newkey2 == 'r')
                {
                    translation(0) = delta_trans;
                    std::cout << "按下了 R，X 轴的平移量增加了 " << delta_trans << " 米。" << std::endl;
                }
                if (newkey2 == 'f')
                {
                    translation(0) = -delta_trans;
                    std::cout << "按下了 F，X 轴的平移量减少了 " << delta_trans << " 米。" << std::endl;
                }
                if (newkey2 == 't')
                {
                    translation(1) = delta_trans;
                    std::cout << "按下了 T，Y 轴的平移量增加了 " << delta_trans << " 米。" << std::endl;
                }
                if (newkey2 == 'g')
                {
                    translation(1) = -delta_trans;
                    std::cout << "按下了 G，Y 轴的平移量减少了 " << delta_trans << " 米。" << std::endl;
                }
                if (newkey2 == 'y')
                {
                    translation(2) = delta_trans;
                    std::cout << "按下了 Y，Z 轴的平移量增加了 " << delta_trans << " 米。" << std::endl;
                }
                if (newkey2 == 'h')
                {
                    translation(2) = -delta_trans;
                    std::cout << "按下了 H，Z 轴的平移量减少了 " << delta_trans << " 米。" << std::endl;
                }
                if (newkey2 == 'z')
                {
                    std::cout << "RESET !!!!!!!!!!!!!!" << std::endl;
                    camera::_Tlc_Estimate = camera::_Tcl_Estimate.inverse();
                    camera::_Tcl_manual = camera::_Tcl_Estimate;
                    camera::_Rcl_manual = camera::_Rcl_Estimate;
                    camera::_tcl_manual = camera::_tcl_Estimate;
                    camera::_Tlc_manual = camera::_Tlc_Estimate;
                    translation.setZero();
                    euler_angles.setZero();
                }

                Eigen::Matrix3d rotation_matrix = Eigen::AngleAxisd(euler_angles(0), Eigen::Vector3d::UnitX()) * Eigen::AngleAxisd(euler_angles(1), Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(euler_angles(2), Eigen::Vector3d::UnitZ()).toRotationMatrix();
                camera::_Rcl_manual = camera::_Rcl_manual * rotation_matrix;
                camera::_tcl_manual = camera::_tcl_manual + translation;
                camera::_Tcl_manual.block<3, 3>(0, 0) = camera::_Rcl_manual;
                camera::_Tcl_manual.block<3, 1>(0, 3) = camera::_tcl_manual;
                cv::Mat image_tmp = raw_image.clone();
                frame->UpdatePose(camera::_Tcl_manual, image_tmp, cloud);

                std::unique_lock<std::mutex> lockOnCallBack(cloud_mutex);
                {
                    boardcloud = frame->Update3dPose(camera::_Tcl_manual);
                }
                std::cout << "当前的旋转矩阵 Rct：\n"
                          << camera::_Rcl_manual << std::endl;
                std::cout << "当前的平移向量 tct：\n"
                          << camera::_tcl_manual << std::endl;
            }
        }
    }
}
