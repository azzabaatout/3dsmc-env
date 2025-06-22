#include "utils/io.h"
#include "utils/points.h"

#include "ceres/ceres.h"
#include <math.h>


// Done implementing: Implement the cost function (check gaussian.cpp for reference)

struct RegistrationCostFunction
{
	RegistrationCostFunction(const Point2D& p1, const Point2D& p2, double weight)
		: point1_(p1), point2_(p2), weight_(weight) {}

	template<typename T>
	bool operator()(const T* const angle, const T* const tx, const T* const ty, T* residual) const
	{
		// Transform point1 by rotation and translation
		T cos_angle = ceres::cos(*angle);
		T sin_angle = ceres::sin(*angle);

		T transformed_x = cos_angle * T(point1_.x) - sin_angle * T(point1_.y) + *tx;
		T transformed_y = sin_angle * T(point1_.x) + cos_angle * T(point1_.y) + *ty;

		// Calculate weighted residuals
		residual[0] = weight_ * (transformed_x - T(point2_.x));
		residual[1] = weight_ * (transformed_y - T(point2_.y));

		return true;
	}

private:
	const Point2D point1_;
	const Point2D point2_;
	const double weight_;
};

int main(int argc, char** argv)
{
	google::InitGoogleLogging(argv[0]);

	// Read data points and the weights, and define the parameters of the problem
	const std::string file_path_1 = "../../Data/points_dragon_1.txt";
	const auto points1 = read_points_from_file<Point2D>(file_path_1);
	
	const std::string file_path_2 = "../../Data/points_dragon_2.txt";
	const auto points2 = read_points_from_file<Point2D>(file_path_2);
	
	const std::string file_path_weights = "../../Data/weights_dragon.txt";
	const auto weights = read_points_from_file<Weight>(file_path_weights);
	
	const double angle_initial = 0.0;
	const double tx_initial = 0.0;
	const double ty_initial = 0.0;
	
	double angle = angle_initial;
	double tx = tx_initial;
	double ty = ty_initial;

	ceres::Problem problem;

	// TODO: For each weighted correspondence create one residual block (check gaussian.cpp for reference)

	for (size_t i = 0; i < points1.size(); ++i) {
		problem.AddResidualBlock(
			new ceres::AutoDiffCostFunction<RegistrationCostFunction, 2, 1, 1, 1>(
				new RegistrationCostFunction(points1[i], points2[i], weights[i].w)
			),
			nullptr,
			&angle,
			&tx,
			&ty
		);
	}

	ceres::Solver::Options options;
	options.max_num_iterations = 25;
	options.linear_solver_type = ceres::DENSE_QR;
	options.minimizer_progress_to_stdout = true;

	ceres::Solver::Summary summary;
	ceres::Solve(options, &problem, &summary);

	std::cout << summary.BriefReport() << std::endl;

	// Output the final values of the translation and rotation (in degree)
	std::cout << "Initial angle: " << angle_initial << "\ttx: " << tx_initial << "\tty: " << ty_initial << std::endl;
	std::cout << "Final angle: " << std::fmod(angle * 180 / M_PI, 360.0) << "\ttx: " << tx << "\tty: " << ty << std::endl;

	system("pause");
	return 0;
}
