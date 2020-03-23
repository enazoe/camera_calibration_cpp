
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
	m.set_input(vec_mat, cv::Size{ 9,6 });
}