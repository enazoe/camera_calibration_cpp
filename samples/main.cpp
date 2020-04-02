
#include "class_camera_calibrator.hpp"


int main()
{
	std::vector<cv::String> images;
	cv::glob("./data/*.jpg", images);
	std::vector<cv::Mat> vec_mat;
	for (const auto &path:images)
	{
		cv::Mat img = cv::imread(path, cv::IMREAD_UNCHANGED);
		vec_mat.push_back(img);
	}

	CameraCalibrator m;
	Eigen::Matrix3d camera_matrix;
	Eigen::VectorXd k;
	std::vector<Eigen::MatrixXd> vec_extrinsics;

	m.set_input(vec_mat, cv::Size{ 9,6 });
	m.get_result(camera_matrix,k,vec_extrinsics);

	DLOG(INFO) << "camera_matrix:\n" << camera_matrix << std::endl;
	DLOG(INFO) << "k:\n" << k << std::endl;
	for (int i=0;i<vec_extrinsics.size();++i)
	{
		DLOG(INFO) << "vec_extrinsics["<<i<<"]:\n" << vec_extrinsics[i] << std::endl;
	}
	std::cin.get();
}