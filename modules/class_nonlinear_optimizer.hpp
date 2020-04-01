#ifndef CLASS_NONLINEAR_OPTIMIZER_HPP_
#define CLASS_NONLINEAR_OPTIMIZER_HPP_

#include <opencv2/opencv.hpp>
#include <ceres/ceres.h>
#include <ceres/rotation.h>

struct Params
{
	Eigen::Matrix3d camera_matrix;
	Eigen::VectorXd k;
	std::vector<Eigen::MatrixXd> vec_rt;
};
struct ReprojectionError
{
	ReprojectionError(const Eigen::Vector2d &img_pts_, const Eigen::Vector2d &board_pts_)
		:_img_pts(img_pts_), _board_pts(board_pts_)
	{
	}

	template<typename T>
	bool operator()(const T* const instrinsics_,
		const T* const k_,
		const T* const rt_,//6 : angle axis and translation
		T* residuls)
	{
		Eigen::Vector3d hom_w(_board_pts(0), _board_pts(1), T(1.));
		T hom_w_trans[3];
		ceres::AngleAxisRotatePoint(rt_, hom_w, hom_w_trans);
		hom_w_trans[0] += rt[3];
		hom_w_trans[1] += rt[4];
		hom_w_trans[2] += rt[5];

		Eigen::Vector2d c_p;
		c_p(0) = hom_w_trans(0) / hom_w_trans[2];
		c_p(1) = hom_w_trans(1) / hom_w_trans[2];

		//distortion
		T r2 = c_p(0)*c_p(0) + c_p(1)*c_p(1);
		T r4 = r2*r2;
		T r_coeff = (T(1) + k_[0] * r2 + k_[1] * r4);
		T xd = c_p(0)*r_coeff;
		T yd = c_p(1)*r_coeff;

		//camera coord => image coord
		T predict_x = instrinsics_[0] * xd + instrinsics_[1] * yd + instrinsics_[2];
		T predict_y = instrinsics_[4] * yd + instrinsics_[5];

		//residus

		residuls[0] = _img_pts(0) - predict_x;
		residuls[1] = _img_pts(1) - predict_y;
	}
	Eigen::Vector2d _img_pts;
	Eigen::Vector2d _board_pts;
};
class NonlinearOptimizer
{
public:
	NonlinearOptimizer()
	{
	}
	~NonlinearOptimizer()
	{
	}

	void refine_all_camera_params(const Params &params_,
		const std::vector<std::vector<cv::Point2f>> &imgs_pts_,
		const std::vector<std::vector<cv::Point2f>> &bords_pts_,
		Params &refined_params_)
	{
		ceres::Problem problem;
		Eigen::Matrix3d camera_matrix = params_.camera_matrix;
		Eigen::VectorXd k = params_.k;
		std::vector<Eigen::MatrixXd> vec_rt = params_.vec_rt;
		for (int n = 0; n < params_.vec_rt.size(); ++n)
		{
			Eigen::MatrixXd x1(2, bords_pts_[n].size());
			Eigen::MatrixXd x2(2, imgs_pts_[n].size());

			for (int i = 0; i < bords_pts_[n].size(); ++i)
			{
				x1(0, i) = bords_pts_[n][i].x;
				x1(1, i) = bords_pts_[n][i].y;
			}
			for (int i = 0; i < imgs_pts_[n].size(); ++i)
			{
				x2(0, i) = imgs_pts_[n][i].x;
				x2(1, i) = imgs_pts_[n][i].y;
			}

			for (int i = 0; i < x1.cols(); i++) 
			{
				ReprojectionError *cost_function =
					new ReprojectionError(x2.col(i),x1.col(i));

				Eigen::AngleAxisd r(vec_rt[n].block<3, 3>(0, 0));
				Eigen::VectorXd vr(r.axis());
				Eigen::VectorXd rt(vr(0), vr(1), vr(2),
					vec_rt[n](0,3), vec_rt[n](1, 3), vec_rt[n](2, 3));
				problem.AddResidualBlock(
					new ceres::AutoDiffCostFunction<
					ReprojectionError,
					2,  // num_residuals
					7>(cost_function),
					NULL,
					camera_matrix.data(),
					k.data(),
					rt.data());
			}
		}
		//// Configure the solver.
		//ceres::Solver::Options options;
		//options.use_nonmonotonic_steps = true;
		//options.preconditioner_type = ceres::SCHUR_JACOBI;
		//options.linear_solver_type = ceres::ITERATIVE_SCHUR;
		//options.use_inner_iterations = true;
		//options.max_num_iterations = 100;
		//options.minimizer_progress_to_stdout = true;

		//// Solve!
		//ceres::Solver::Summary summary;
		//ceres::Solve(options, &problem, &summary);

		//std::cout << "Final report:\n" << summary.FullReport();
		//std::cin.get();
	}
private:


};

#endif
