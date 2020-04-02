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

template <typename T>
void SymmetricGeometricDistanceTerms(const Eigen::Matrix<T, 3, 3> &H,
	const Eigen::Matrix<T, 2, 1> &x1,
	const Eigen::Matrix<T, 2, 1> &x2,
	T forward_error[2],
	T backward_error[2])
{
	typedef Eigen::Matrix<T, 3, 1> Vec3;
	Vec3 x(x1(0), x1(1), T(1.0));
	Vec3 y(x2(0), x2(1), T(1.0));

	Vec3 H_x = H * x;
	Vec3 Hinv_y = H.inverse() * y;

	H_x /= H_x(2);
	Hinv_y /= Hinv_y(2);

	forward_error[0] = H_x(0) - y(0);
	forward_error[1] = H_x(1) - y(1);
	backward_error[0] = Hinv_y(0) - x(0);
	backward_error[1] = Hinv_y(1) - x(1);
}
double SymmetricGeometricDistance(const Eigen::Matrix3d &H,
	const Eigen::Vector2d &x1,
	const Eigen::Vector2d &x2)
{
	Eigen::Vector2d forward_error, backward_error;
	SymmetricGeometricDistanceTerms<double>(H,
		x1,
		x2,
		forward_error.data(),
		backward_error.data());
	return forward_error.squaredNorm() +
		backward_error.squaredNorm();
}


struct EstimateHomographyOptions {
	// Default settings for homography estimation which should be suitable
	// for a wide range of use cases.
	EstimateHomographyOptions()
		: max_num_iterations(50),
		expected_average_symmetric_distance(1e-16) {}

	int max_num_iterations;
	double expected_average_symmetric_distance;
};

// Termination checking callback. This is needed to finish the
// optimization when an absolute error threshold is met, as opposed
// to Ceres's function_tolerance, which provides for finishing when
// successful steps reduce the cost function by a fractional amount.
// In this case, the callback checks for the absolute average reprojection
// error and terminates when it's below a threshold (for example all
// points < 0.5px error).
class TerminationCheckingCallback : public ceres::IterationCallback {
public:
	TerminationCheckingCallback(const Eigen::MatrixXd &x1, const Eigen::MatrixXd &x2,
		const EstimateHomographyOptions &options,
		Eigen::Matrix3d *H)
		: options_(options), x1_(x1), x2_(x2), H_(H) {}

	virtual ceres::CallbackReturnType operator()(
		const ceres::IterationSummary& summary) {
		// If the step wasn't successful, there's nothing to do.
		if (!summary.step_is_successful) {
			return ceres::SOLVER_CONTINUE;
		}

		// Calculate average of symmetric geometric distance.
		double average_distance = 0.0;
		for (int i = 0; i < x1_.cols(); i++) {
			average_distance += SymmetricGeometricDistance(*H_,
				x1_.col(i),
				x2_.col(i));
		}
		average_distance /= x1_.cols();

		if (average_distance <= options_.expected_average_symmetric_distance) {
			return ceres::SOLVER_TERMINATE_SUCCESSFULLY;
		}
		return ceres::SOLVER_CONTINUE;
	}

private:
	const EstimateHomographyOptions &options_;
	const Eigen::MatrixXd &x1_;
	const Eigen::MatrixXd &x2_;
	Eigen::Matrix3d *H_;
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

	void refine_H(const std::vector<cv::Point2f> &img_pts_,
		const std::vector<cv::Point2f> &board_pts_,
		const Eigen::Matrix3d &matrix_H_,
		Eigen::Matrix3d &refined_H_)
	{
		Eigen::MatrixXd x1(2, board_pts_.size());
		Eigen::MatrixXd x2(2, img_pts_.size());

		for (int i = 0; i < board_pts_.size(); ++i)
		{
			x1(0, i) = board_pts_[i].x;
			x1(1, i) = board_pts_[i].y;
		}
		for (int i = 0; i < img_pts_.size(); ++i)
		{
			x2(0, i) = img_pts_[i].x;
			x2(1, i) = img_pts_[i].y;
		}


		Eigen::Matrix3d H = matrix_H_;
		//std::cout << "H:" << H<< std::endl;
		// Step 2: Refine matrix using Ceres minimizer.
		ceres::Problem problem;
		for (int i = 0; i < x1.cols(); i++) {
			HomographySymmetricGeometricCostFunctor
				*homography_symmetric_geometric_cost_function =
				new HomographySymmetricGeometricCostFunctor(x1.col(i),
					x2.col(i));

			problem.AddResidualBlock(
				new ceres::AutoDiffCostFunction<
				HomographySymmetricGeometricCostFunctor,
				4,  // num_residuals
				9>(homography_symmetric_geometric_cost_function),
				NULL,
				H.data());
		}
		EstimateHomographyOptions options;
		options.expected_average_symmetric_distance = 0.02;
		// Configure the solve.
		ceres::Solver::Options solver_options;
		solver_options.linear_solver_type = ceres::DENSE_QR;
		solver_options.max_num_iterations = options.max_num_iterations;
		solver_options.update_state_every_iteration = true;

		// Terminate if the average symmetric distance is good enough.
		TerminationCheckingCallback callback(x1, x2, options, &H);
		solver_options.callbacks.push_back(&callback);

		// Run the solve.
		ceres::Solver::Summary summary;
		ceres::Solve(solver_options, &problem, &summary);

		refined_H_ = H / H(2, 2);
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
		Eigen::VectorXd v_camera_matrix(5);
		v_camera_matrix << camera_matrix(0,0),
			camera_matrix(0, 1),
			camera_matrix(0, 2),
			camera_matrix(1, 1),
			camera_matrix(1, 2);
		double *p_camera = v_camera_matrix.data();
		double *p_k = k.data();
		
		//package all rt
		std::vector<Eigen::VectorXd> packet_rt;
		for (int n = 0; n < params_.vec_rt.size(); ++n)
		{
			Eigen::AngleAxisd r(vec_rt[n].block<3, 3>(0, 0));
			Eigen::VectorXd rot_vec(r.axis()*r.angle());
			Eigen::VectorXd rt(6);
			rt << rot_vec(0), rot_vec(1), rot_vec(2),
				vec_rt[n](0, 3), vec_rt[n](1, 3), vec_rt[n](2, 3);
			packet_rt.push_back(rt);
		}
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
			
			double *p_rt = &packet_rt[n](0);
			for (int i = 0; i < x1.cols(); i++) 
			{
				ReprojectionError *cost_function =
					new ReprojectionError(x2.col(i),x1.col(i));

					problem.AddResidualBlock(
					new ceres::AutoDiffCostFunction<
					ReprojectionError,
					2,  // num_residuals
					5,2,6>(cost_function),
					NULL,
					p_camera,
					p_k,
					p_rt);
			}
		}
		// Configure the solver.
		ceres::Solver::Options options;
		options.linear_solver_type = ceres::DENSE_SCHUR;
		options.minimizer_progress_to_stdout = true;

		// Solve!
		ceres::Solver::Summary summary;
		ceres::Solve(options, &problem, &summary);

		DLOG(INFO)<< "Final Brief Report:\n" << summary.BriefReport()<<std::endl;
		this->formate_data(v_camera_matrix, k, packet_rt, refined_params_);
	}
private:
	
	void formate_data(const Eigen::VectorXd &v_camera_matrix_,
		const Eigen::VectorXd &v_dist_,
		const std::vector<Eigen::VectorXd> &v_rt_,
		Params &params_)
	{
		params_.camera_matrix << v_camera_matrix_(0), v_camera_matrix_(1), v_camera_matrix_(2),
			0., v_camera_matrix_(3), v_camera_matrix_(4),
			0, 0, 1.;
		params_.k = v_dist_;
		params_.vec_rt.clear();
		for (const auto &rt:v_rt_)
		{
			Eigen::Vector3d rv(rt(0), rt(1), rt(2));
			Eigen::AngleAxisd r_v(rv.norm(), rv/rv.norm());
			Eigen::Matrix<double, 3, 4> rt;
			rt.block<3, 3>(0, 0) = r_v.toRotationMatrix();
			rt.block<3, 1>(0, 3) = Eigen::Vector3d(rt(3), rt(4), rt(5));
			params_.vec_rt.push_back(rt);
		}
	}

	// Cost functor which computes symmetric geometric distance
	// used for homography matrix refinement.
	struct HomographySymmetricGeometricCostFunctor
	{
		HomographySymmetricGeometricCostFunctor(const Eigen::Vector2d &x,
			const Eigen::Vector2d &y)
			: x_(x), y_(y) { }

		template<typename T>
		bool operator()(const T* homography_parameters, T* residuals) const {
			typedef Eigen::Matrix<T, 3, 3> Mat3;
			typedef Eigen::Matrix<T, 2, 1> Vec2;

			Mat3 H(homography_parameters);
			Vec2 x(T(x_(0)), T(x_(1)));
			Vec2 y(T(y_(0)), T(y_(1)));

			SymmetricGeometricDistanceTerms<T>(H,
				x,
				y,
				&residuals[0],
				&residuals[2]);
			return true;
		}

		const Eigen::Vector2d x_;
		const Eigen::Vector2d y_;
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
			T* residuls)const
		{
			//	Eigen::Vector3d hom_w(_board_pts(0), _board_pts(1), T(1.));
			T hom_w_t[3];
			hom_w_t[0] = T(_board_pts(0));
			hom_w_t[1] = T(_board_pts(1));
			hom_w_t[2] = T(1.);
			T hom_w_trans[3];
			ceres::AngleAxisRotatePoint(rt_, hom_w_t, hom_w_trans);
			hom_w_trans[0] += rt_[3];
			hom_w_trans[1] += rt_[4];
			hom_w_trans[2] += rt_[5];

			T c_x = hom_w_trans[0] / hom_w_trans[2];
			T c_y = hom_w_trans[1] / hom_w_trans[2];

			//distortion
			T r2 = c_x*c_x + c_y*c_y;
			T r4 = r2*r2;
			T r_coeff = (T(1) + k_[0] * r2 + k_[1] * r4);
			T xd = c_x*r_coeff;
			T yd = c_y*r_coeff;

			//camera coord => image coord
			T predict_x = instrinsics_[0] * xd + instrinsics_[1] * yd + instrinsics_[2];
			T predict_y = instrinsics_[3] * yd + instrinsics_[4];

			//residus

			residuls[0] = _img_pts(0) - predict_x;
			residuls[1] = _img_pts(1) - predict_y;
			return true;
		}
		const Eigen::Vector2d _img_pts;
		const Eigen::Vector2d _board_pts;
	};

};

#endif
