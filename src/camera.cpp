#include "camera.hpp"

camera::camera()
{
    _col = 5;
    _row = 8;
    _square_size = 0.0275;
    distCoeffs_ = cv::Mat::zeros(1, 5, CV_64F);
    distCoeffs_.at<double>(0, 0) = -0.311801;
    distCoeffs_.at<double>(0, 1) = 0.073565;
    distCoeffs_.at<double>(0, 2) = 0.002223;
    distCoeffs_.at<double>(0, 3) = 0.001517;
    distCoeffs_.at<double>(0, 4) = 0;

    intrinsics_ = cv::Mat::eye(3, 3, CV_64F);
    intrinsics_.at<double>(0, 0) = 700.225972;
    intrinsics_.at<double>(1, 1) = 681.416116;
    intrinsics_.at<double>(0, 2) = 631.705476;
    intrinsics_.at<double>(1, 2) = 346.033095;
}

bool camera::EstimateBoardPose(cv::Mat image)
{
    cv::Size pattern_size(_col, _row);
    cv::Mat gray;
    std::vector<cv::Point2f> corners;
    cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);

    if (cv::findChessboardCorners(image, pattern_size, corners))
    {
        // cv::cornerSubPix(
        //     image, corners, cv::Size(11, 11), cv::Size(-1, -1),
        //     cv::TermCriteria(2 + 1, 30, 0.1));
        cv::cornerSubPix(gray, corners, cv::Size(11, 11), cv::Size(-1, -1),
                         cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.1));
        if (corners.size() == _col * _row)
        {
            int count = 0;
            for (int i = 0; i < _row; i++)
            {
                for (int j = 0; j < _col; j++)
                {
                    // Todo: change 3d-coordinate
                    _p3ds.emplace_back(
                        cv::Point3f(j * _square_size, i * _square_size, 0.0));
                    _p2ds.emplace_back(cv::Point2f(corners[count].x, corners[count].y));
                    count++;
                }
            }
            cv::drawChessboardCorners(image, pattern_size, cv::Mat(corners), true);
        }
        else
        {
            std::cout << "Chessboard config is not correct with image\n";
        }
    }
    else
    {
        std::cout << "No chessboard detected in image\n";
        return false;
    }

    if (_p3ds.size() != _p2ds.size() || _p3ds.size() < 4)
        return false;
    // use PnP to calculate the pose
    cv::Mat r, t;
    cv::solvePnP(_p3ds, _p2ds, intrinsics_, distCoeffs_, r, t);
    std::cout << "solvePnP" << std::endl;
    // type transform
    cv::Mat cvR;
    cv::Rodrigues(r, cvR);

    Eigen::Matrix3d Rcb;
    cv::cv2eigen(cvR, Rcb);
    Eigen::Vector3d tcb;
    cv::cv2eigen(t, tcb);

    _Tcb.setIdentity();
    _Tcb.block<3, 3>(0, 0) = Rcb; // 设置旋转部分
    _Tcb.block<3, 1>(0, 3) = tcb; // 设置平移部分

    // transform matrix
    _Tbc.setIdentity();
    _Tbc.block<3, 3>(0, 0) = Rcb.inverse();
    _Tbc.block<3, 1>(0, 3) = -Rcb.inverse() * tcb;

    std::cout << "pnp求位姿成功" << std::endl;
    std::cout << "The transformation matrix _Tcb is: \n"
              << _Tcb << std::endl;
    return true;
    // visualize coordinate
    // Eigen::Matrix3d K;
    // cv::cv2eigen(intrinsics_, K);
    // Coordinate coor(Rcb, tcb);

    // std::cout << "before DrawOnImg" << std::endl;
    // coor.DrawOnImg(K, imgUndistort, imgUndistort);
    // std::cout << "after DrawOnImg" << std::endl;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr camera::ProjectChessboardToCameraSpace()
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    std::vector<cv::Point3f> transformed_p3ds;
    for (const auto &point : _p3ds)
    {
        Eigen::Vector4d point_homo(point.x, point.y, point.z, 1.0); // 齐次坐标
        Eigen::Vector4d transformed_point_homo = _Tcb * point_homo;
        // transformed_p3ds.emplace_back(
        //     cv::Point3f(transformed_point_homo(0), transformed_point_homo(1), transformed_point_homo(2)));
        cloud->push_back(pcl::PointXYZ(transformed_point_homo(0), transformed_point_homo(1), transformed_point_homo(2)));
    }
    return cloud;
}