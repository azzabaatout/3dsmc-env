#include "utils/io.h"
#include "utils/points.h"

#include "ceres/ceres.h"
#include <math.h>


// Done: Implement the cost function (check gaussian.cpp for reference)
// Implemented -
struct SurfaceCostFunction
{
	SurfaceCostFunction(const Point3D& point_)
		: point(point_)
	{
	}

	template<typename T>
	bool operator()(const T* const a, const T* const b, const T* const c, T* residual) const
	{
		T lhs = c[0] * T(point.z);
		T rhs = a[0] * T(point.x) * T(point.x) + b[0] * T(point.y) * T(point.y);
		residual[0] = c[0] * T(point.z) - a[0] * T(point.x) * T(point.x) - b[0] * T(point.y) * T(point.y);
		return true;
	}

private:
	const Point3D point;
};


int main(int argc, char** argv)
{
	google::InitGoogleLogging(argv[0]);

	// Read 3D surface data points and define the parameters of the problem
	const std::string file_path = "../../Data/points_surface.txt";
	const auto points = read_points_from_file<Point3D>(file_path);
	
	const double a_initial = 1.0;
	const double b_initial = 1.0;
	const double c_initial = 1.0;
	
	double a = a_initial;
	double b = b_initial;
	double c = c_initial;
	
	ceres::Problem problem;

	// Done: For each data point create one residual block (check gaussian.cpp for reference) Read in the input data, create a new cost function and define the parameters for the problem.

	// DOne: For each data point create one residual block exactly like in the gaussian example
	for (auto& point : points)
	{
		problem.AddResidualBlock(
			new ceres::AutoDiffCostFunction<SurfaceCostFunction, 1, 1, 1, 1>(
				new SurfaceCostFunction(point)
			),
			nullptr, &a, &b, &c
		);
	}


	ceres::Solver::Options options;
	options.max_num_iterations = 100;
	options.linear_solver_type = ceres::DENSE_QR;
	options.minimizer_progress_to_stdout = true;

	ceres::Solver::Summary summary;
	ceres::Solve(options, &problem, &summary);

	std::cout << summary.BriefReport() << std::endl;
	
	// Output the final values of the parameters
	std::cout << "Initial a: " << a_initial << "\tb: " << b_initial << "\tc: " << c_initial << std::endl;
	std::cout << "Final a: " << a << "\tb: " << b << "\tc: " << c << std::endl;

	system("pause");
	return 0;
}
