#ifndef CLASS_CAMERA_CALIBRATOR_HPP_
#define CLASS_CAMERA_CALIBRATOR_HPP_

#include <vector>
#include <opencv2/opencv.hpp>
#include <gflags/gflags.h>
#include <glog/logging.h>
#include <ceres/ceres.h>

#include "class_nonlinear_optimizer.hpp"

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
			CHECK(1 == img.channels()) << "images must be gray";
			std::vector<cv::Point2f> corner_pts;
			int found = cv::findChessboardCorners(img, chessboard_size_, corner_pts,
				cv::CALIB_CB_ADAPTIVE_THRESH|cv::CALIB_CB_FAST_CHECK | cv::CALIB_CB_NORMALIZE_IMAGE);
			if (!found)
			{
				continue;				
			}
			cv::TermCriteria criteria(2, 30, 0.001);
			cv::cornerSubPix(img, corner_pts, chessboard_size_, cv::Size(-1, -1), criteria);
			//images coordinate points
			_imgs_pts.push_back(corner_pts);
			//chessboard coordinate points
			this->make_board_points(chessboard_size_);
			this->disp_corners(img, chessboard_size_, corner_pts, found);
		}
	}

	void get_result(Eigen::Matrix3d &camera_matrix_,
		Eigen::VectorXd &k_,
		std::vector<Eigen::MatrixXd> &vec_extrinsics_)
	{
		std::vector<Eigen::Matrix3d> vec_h;
		this->get_homography(vec_h);

		this->get_camera_instrinsics(vec_h, camera_matrix_);

		this->get_extrinsics(vec_h, camera_matrix_, vec_extrinsics_);

		this->get_distortion(camera_matrix_, vec_extrinsics_, k_);

		this->refine_all(camera_matrix_, k_, vec_extrinsics_);
	}

private:

	void make_board_points(const cv::Size  &chessboard_size_)
	{
		std::vector<cv::Point2f> vec_points;
		for (int r = 0; r < chessboard_size_.height; ++r)
		{
			for (int c = 0; c < chessboard_size_.width; ++c)
			{
				vec_points.emplace_back(c, r);
			}
		}
		_boards_pts.push_back(vec_points);
	}

	void disp_corners(const cv::Mat &img,
		const cv::Size  &chessboard_size_,
		std::vector<cv::Point2f> corner_pts,
		int found)
	{
		if (!_b_disp_corners)
		{
			return;
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

	void refine_all(Eigen::Matrix3d &camera_matrix_,
		Eigen::VectorXd &k_,
		std::vector<Eigen::MatrixXd> &vec_extrinsics_)
	{
		Params params,params_refined;
		params.camera_matrix = camera_matrix_;
		params.k=k_;
		params.vec_rt=vec_extrinsics_;
		optimier.refine_all_camera_params(params, _imgs_pts,
			_boards_pts, params_refined);
		camera_matrix_ = params_refined.camera_matrix;
		k_ = params_refined.k;
		vec_extrinsics_ = params_refined.vec_rt;
	}

	void get_distortion(const Eigen::Matrix3d &camera_matrix_,
		const std::vector<Eigen::MatrixXd> &vec_extrinsics_,
		Eigen::VectorXd &k_)
	{
		Eigen::MatrixXd D;
		Eigen::VectorXd d;
		double uc = camera_matrix_(0, 2);
		double vc = camera_matrix_(1, 2);
		for (int i = 0; i <_imgs_pts.size(); ++i)
		{
			for (int j = 0; j < _imgs_pts[i].size(); ++j)
			{
				Eigen::Vector4d houm_coor(_boards_pts[i][j].x, _boards_pts[i][j].y, 0, 1);
				Eigen::Vector3d uv = camera_matrix_*vec_extrinsics_[i] * houm_coor;
				Eigen::Vector2d uv_estim(uv(0) / uv(2), uv(1) / uv(2));

				Eigen::Vector3d coor_norm = vec_extrinsics_[i] * houm_coor;
				coor_norm /= coor_norm(2);
				Eigen::Vector2d v_r(coor_norm(0), coor_norm(1));
				double r = v_r.norm();

				Eigen::RowVector2d vu((uv_estim(0) - uc)*r*r, (uv_estim(0) - uc)*r*r*r*r);
				D.conservativeResize(D.rows() + 1, 2);
				D.row(D.rows() - 1) = vu;
				Eigen::RowVector2d vv((uv_estim(1) - vc)*r*r, (uv_estim(1) - vc)*r*r*r*r);
				D.conservativeResize(D.rows() + 1, 2);
				D.row(D.rows() - 1) = vv;
				
				d.conservativeResize(d.size() + 1);
				d(d.size() - 1) = _imgs_pts[i][j].x - uv_estim(0);
				d.conservativeResize(d.size() + 1);
				d(d.size() - 1) = _imgs_pts[i][j].y - uv_estim(1);
			}
		}
		Eigen::MatrixXd DTD = D.transpose()*D;
		Eigen::MatrixXd temp = (DTD.inverse())*D.transpose();
		k_ = temp*d;
	}

	void get_extrinsics(const std::vector<Eigen::Matrix3d> &vec_h_,
						const Eigen::Matrix3d &camera_matrix_,
						std::vector<Eigen::MatrixXd> &vec_extrinsics_)
	{
		vec_extrinsics_.clear();
		Eigen::Matrix3d inv_camera_matrix = camera_matrix_.inverse();
		for (int i=0;i<vec_h_.size();++i)
		{
			Eigen::Vector3d s = inv_camera_matrix*vec_h_[i].col(0);
			double scalar_factor = 1 / s.norm();

			Eigen::Vector3d r0 = scalar_factor * inv_camera_matrix*vec_h_[i].col(0);
			Eigen::Vector3d r1 = scalar_factor * inv_camera_matrix*vec_h_[i].col(1);
			Eigen::Vector3d t  = scalar_factor * inv_camera_matrix*vec_h_[i].col(2);
			Eigen::Vector3d r2 = r0.cross(r1);

			Eigen::MatrixXd RT(3,4);
			RT.block<3,1>(0,0) = r0;
			RT.block<3,1>(0,1) = r1;
			RT.block<3,1>(0,2) = r2;
			RT.block<3,1>(0,3) = t;
			vec_extrinsics_.push_back(RT);
		}
	}

	//
	void create_v(const Eigen::Matrix3d &h_,
				  const int p,
				  const int q,
				  Eigen::RowVectorXd &row_v_)
	{
		row_v_ << h_(0, p) * h_(0, q),
			h_(0, p) * h_(1, q) + h_(1, p) * h_(0, q),
			h_(1, p) * h_(1, q),
			h_(2, p) * h_(0, q) + h_(0, p) * h_(2, q),
			h_(2, p) * h_(1, q) + h_(1, p) * h_(2, q),
			h_(2, p) * h_(2, q);
		
	}

	//获取内参
	void get_camera_instrinsics(const std::vector<Eigen::Matrix3d> &vec_h_,
								Eigen::Matrix3d &camera_matrix_)
	{
		int N = vec_h_.size();
		Eigen::MatrixXd V(2 * N, 6);
		V.setZero();

		for (int n = 0; n < N; ++n)
		{
			Eigen::RowVectorXd v01(6),v00(6), v11(6);
			create_v(vec_h_[n], 0, 1, v01);
			V.row(2*n) = v01;
			create_v(vec_h_[n], 0, 0, v00);
			create_v(vec_h_[n], 1, 1, v11);
			V.row(2*n + 1) = v00 - v11;
		}
		Eigen::JacobiSVD<Eigen::MatrixXd> svd(V, Eigen::ComputeFullV);
		Eigen::VectorXd b = svd.matrixV().col(5);
		//求取相机内参
		double	w = b[0] * b[2] * b[5] - b[1] * b[1] * b[5] - b[0] * b[4] * b[4] + 2 * b[1] * b[3] * b[4] - b[2] * b[3] * b[3];
		double	d = b[0] * b[2] - b[1] * b[1];

		double	alpha = std::sqrt(w / (d * b[0]));
		double	beta = std::sqrt(w / (d*d) * b[0]);
		double	gamma = std::sqrt(w / (d*d * b[0])) * b[1];
		double	uc = (b[1] * b[4] - b[2] * b[3]) / d;
		double	vc = (b[1] * b[3] - b[0] * b[4]) / d;

		camera_matrix_ << alpha, gamma, uc,
			0, beta, vc,
			0, 0, 1;
		DLOG(INFO) << "rough camera_matrix :\n" << camera_matrix_ << std::endl;
	}

	void get_homography(std::vector<Eigen::Matrix3d> &vec_h_)
	{
		vec_h_.clear();
		for (int i=0;i<_imgs_pts.size();++i)
		{
			Eigen::Matrix3d ini_H,refined_H;
			this->estimate_H(_imgs_pts[i], _boards_pts[i], ini_H);
			optimier.refine_H(_imgs_pts[i], _boards_pts[i], ini_H,refined_H);
			vec_h_.push_back(refined_H);
		}
	}

	//normalized DLT algorithm
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

	//归一化矩阵，
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

private:

	bool _b_disp_corners = false;
	std::vector<std::vector<cv::Point2f>> _imgs_pts;
	std::vector<std::vector<cv::Point2f>> _boards_pts;

	NonlinearOptimizer optimier;
};


#endif // CLASS_CAMERA_CALIBRATOR_HPP_
