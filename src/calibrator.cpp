#include "calibrator.hpp"

void calibrator::VisualizationCallback(pcl::visualization::PCLVisualizer &viz)
{
    std::unique_lock<std::mutex> lockOnCallBack(_cloud_mutex);
    {
        if (!viz.updatePointCloud(_cloud, _red, "tof_cloud"))
        {
            viz.addPointCloud(_cloud, _red, "tof_cloud");
        }

        if (!viz.updatePointCloud(_boardcloud, _green, "board_cloud"))
        {
            viz.addPointCloud(_boardcloud, _green, "board_cloud");
        }
    }
}

void calibrator::ShowImageAndReadKeyboardThread()
{
    double delta_angle = 0.001;            // 欧拉角调整量
    double delta_trans = 0.001;            // 平移调整量
    Eigen::Vector3d euler_angles(0, 0, 0); // 欧拉角
    Eigen::Vector3d translation(0, 0, 0);  // 平移向量
    while (1)
    {
        {
            std::unique_lock<std::mutex> image_lock(_image_mutex);
            cv::imshow("image", _image);
        }
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

void calibrator::Run(cv::Mat &image, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud)
{
    _image = image.clone();
    _image_raw = image.clone();
    std::shared_ptr<camera> frame(new camera());
    cv::Mat gray;
    cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);

    frame->Project3DTo2D(_image, cloud);
    frame->UpdatePose(camera::_Tcl_manual)
}