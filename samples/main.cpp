#include <gflags/gflags.h>
#include "class_camera_calibrator.hpp"


int main()
{
	FLAGS_log_dir = "./";
	FLAGS_colorlogtostderr = true;
	google::InitGoogleLogging("calibrator");
	google::LogToStderr();
	std::vector<cv::String> images;
	cv::glob("../../images/*.jpg", images);
	std::vector<cv::Mat> vec_mat;
	for (const auto &path:images)
	{
		cv::Mat img = cv::imread(path, cv::IMREAD_GRAYSCALE);
		vec_mat.push_back(img);
	}

	CameraCalibrator m;
	Eigen::Matrix3d camera_matrix;
	Eigen::VectorXd k;
	std::vector<Eigen::MatrixXd> vec_extrinsics;

	m.set_input(vec_mat, cv::Size{ 9,6 });
	m.get_result(camera_matrix,k,vec_extrinsics);

	std::cout << "camera_matrix:\n" << camera_matrix << std::endl;
	std::cout << "k:\n" << k << std::endl;
	//for (int i=0;i<vec_extrinsics.size();++i)
	//{
	//	LOG(INFO) << "vec_extrinsics["<<i<<"]:\n" << vec_extrinsics[i] << std::endl;
	//}
	Eigen::Matrix3d opencv_camera_matrix;
	opencv_camera_matrix << 532.79536563, 0., 342.4582516,
		0, 532.91928339, 233.90060514,
		0, 0, 1;
	std::cout << "opencv calibrateCamera api result:\n" << opencv_camera_matrix << std::endl; 
	std::cin.get();
}