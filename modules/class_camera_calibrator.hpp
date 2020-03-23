#ifndef CLASS_CAMERA_CALIBRATOR_HPP_
#define CLASS_CAMERA_CALIBRATOR_HPP_

#include <vector>
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>


class CameraCalibrator
{
public:
	void set_input(const std::vector<cv::Mat> &vec_mat_,
		           const cv::Size  &chessboard_size_)
	{
		std::cout << vec_mat_.size() << std::endl;
		for (const auto &img : vec_mat_)
		{
			cv::Mat mat_gray;// = cv::Mat::zeros(img.size(), CV_8UC1);
			std::vector<cv::Point2f> corner_pts;
			if (3 == img.channels())
			{
				cv::cvtColor(img, mat_gray, cv::COLOR_BGR2GRAY);
			}
			else
			{
				img.copyTo(mat_gray);
			}
			int found = cv::findChessboardCorners(mat_gray, chessboard_size_, corner_pts,
				cv::CALIB_CB_ADAPTIVE_THRESH|cv::CALIB_CB_FAST_CHECK | cv::CALIB_CB_NORMALIZE_IMAGE);
			if (!found)
			{
				continue;				
			}
			cv::TermCriteria criteria(2, 30, 0.001);
			cv::cornerSubPix(mat_gray, corner_pts, chessboard_size_, cv::Size(-1, -1), criteria);
			//images coordinate points
			_vec_vec_img_pts.push_back(corner_pts);

			//chessboard coordinate points
			std::vector<cv::Point2f> vec_points;
			for (int r=0;r<chessboard_size_.height;++r)
			{
				for (int c=0;c<chessboard_size_.width;++c)
				{
					vec_points.emplace_back(r, c);
				}
			}
			_vec_vec_board_pts.push_back(vec_points);

			if (!_b_disp_corners)
			{
				continue;
			}

			cv::Mat frame;
			if (1 == img.channels())
			{
				cv::cvtColor(img, frame, cv::COLOR_GRAY2BGR);
			}
			else
			{
				img.copyTo(frame);
			}
						cv::drawChessboardCorners(frame, chessboard_size_, corner_pts, found);
			cv::imshow("Image", frame);
			cv::waitKey(0);
		}
	}

	void get_result()
	{

	}

private:

	std::vector<std::vector<cv::Point2f>> _vec_vec_img_pts;
	std::vector<std::vector<cv::Point2f>> _vec_vec_board_pts;

	bool _b_disp_corners =true;
};


#endif // CLASS_CAMERA_CALIBRATOR_HPP_
