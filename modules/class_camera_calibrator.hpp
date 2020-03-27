#ifndef CLASS_CAMERA_CALIBRATOR_HPP_
#define CLASS_CAMERA_CALIBRATOR_HPP_

#include <vector>
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <Eigen/Dense>

#include <ceres/ceres.h>
class CameraCalibrator
{
public:
	void set_input(const std::vector<cv::Mat> &vec_mat_,
		           const cv::Size  &chessboard_size_)
	{
		_boards_pts.clear();
		_imgs_pts.clear();
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
			_imgs_pts.push_back(corner_pts);

			//chessboard coordinate points
			std::vector<cv::Point2f> vec_points;
			for (int r=0;r<chessboard_size_.height;++r)
			{
				for (int c=0;c<chessboard_size_.width;++c)
				{
					vec_points.emplace_back(c,r);
				}
			}
			_boards_pts.push_back(vec_points);

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
		std::vector<Eigen::Matrix3d> vec_h;
		this->get_homography(vec_h);
	}

private:

	void get_homography(std::vector<Eigen::Matrix3d> &vec_h_)
	{
		vec_h_.clear();
		for (int i=0;i<_imgs_pts.size();++i)
		{
			Eigen::Matrix3d ini_H,refined_H;
			this->estimate_H(_imgs_pts[i], _boards_pts[i], ini_H);
			this->refine_H(_imgs_pts[i], _boards_pts[i], ini_H, refined_H);
			vec_h_.push_back(refined_H);
		}
	}

	void estimate_H(const std::vector<cv::Point2f> &img_pts_,
		const std::vector<cv::Point2f> &board_pts_,
		Eigen::Matrix3d &matrix_H_)
	{
		Eigen::Matrix3d matrix_normalize_img_pts;
		Eigen::Matrix3d matrix_normalize_board_pts;
		int N = img_pts_.size();
		this->get_normalization_matrix(img_pts_, matrix_normalize_img_pts);
		this->get_normalization_matrix(board_pts_, matrix_normalize_board_pts);
		Eigen::MatrixXd M(2 * N, 9);
		M.setZero();

		for (int i = 0; i < N; ++i)
		{
			Eigen::Vector3d norm_img_p = matrix_normalize_img_pts
				* Eigen::Vector3d(img_pts_[i].x, img_pts_[i].y, 1);
			Eigen::Vector3d norm_board_p = matrix_normalize_board_pts
				* Eigen::Vector3d(board_pts_[i].x, board_pts_[i].y, 1);
			//M
			M(2 * i, 0) = -norm_board_p(0);
			M(2 * i, 1) = -norm_board_p(1);
			M(2 * i, 2) = -1;
			M(2 * i, 6) = norm_img_p(0)*norm_board_p(0);
			M(2 * i, 7) = norm_img_p(0)*norm_board_p(1);
			M(2 * i, 8) = norm_img_p(0);

			M(2 * i + 1, 3) = -norm_board_p(0);
			M(2 * i + 1, 4) = -norm_board_p(1);
			M(2 * i + 1, 5) = -1;
			M(2 * i + 1, 6) = norm_img_p(1)*norm_board_p(0);
			M(2 * i + 1, 7) = norm_img_p(1)*norm_board_p(1);
			M(2 * i + 1, 8) = norm_img_p(1);

		}
		//svd solve M*h=0
		Eigen::JacobiSVD<Eigen::MatrixXd> svd(M, Eigen::ComputeFullV);
		Eigen::VectorXd V = svd.matrixV().col(8);
		matrix_H_ << V(0), V(1), V(2), V(3), V(4), V(5), V(6), V(7), V(8);
		matrix_H_ = matrix_normalize_img_pts.inverse()*matrix_H_*matrix_normalize_board_pts;
		matrix_H_ /= matrix_H_(2, 2);
	}

	//¹éÒ»»¯¾ØÕó£¬
	void get_normalization_matrix(const std::vector<cv::Point2f> &pts_,Eigen::Matrix3d &matrix_trans_)
	{
		double sum_x = 0, sum_y = 0;
		std::for_each(std::begin(pts_), std::end(pts_),
			[&](const cv::Point2f &p)
			{
			sum_x += p.x;
			sum_y += p.y;
			});
		double mean_x = sum_x / pts_.size();
		double mean_y = sum_y / pts_.size();

		double accmx = 0, accmy = 0;
		std::for_each(std::begin(pts_), std::end(pts_),
			[&](const cv::Point2f &p)
			{
			accmx += (p.x - mean_x)*(p.x - mean_x);
			accmy += (p.y - mean_y)*(p.y - mean_y);
			});
		double stdx = std::sqrt(accmx / double(pts_.size() - 1));
		double stdy = std::sqrt(accmy / double(pts_.size() - 1));

		double sx = std::sqrt(2.) / stdx;
		double sy = std::sqrt(2.) / stdy;

		matrix_trans_ << sx, 0, -sx*mean_x,
			0, sy, -sy*mean_y,
			0, 0, 1;
	}

	void refine_H(const std::vector<cv::Point2f> &img_pts_,
				  const std::vector<cv::Point2f> &board_pts_,
				  const Eigen::Matrix3d &matrix_H_,
				  Eigen::Matrix3d &refined_H_)
	{
		
	}
private:

	std::vector<std::vector<cv::Point2f>> _imgs_pts;
	std::vector<std::vector<cv::Point2f>> _boards_pts;

	bool _b_disp_corners = false;
};


#endif // CLASS_CAMERA_CALIBRATOR_HPP_
