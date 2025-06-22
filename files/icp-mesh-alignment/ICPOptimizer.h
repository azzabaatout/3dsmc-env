#pragma once

// The Google logging library (GLOG), used in Ceres, has a conflict with Windows defined constants. This definitions prevents GLOG to use the same constants
#define GLOG_NO_ABBREVIATED_SEVERITIES

#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <flann/flann.hpp>

#include "SimpleMesh.h"
#include "NearestNeighbor.h"
#include "PointCloud.h"
#include "ProcrustesAligner.h"


/**
 * Helper methods for writing Ceres cost functions.
 */
template <typename T>
static inline void fillVector(const Vector3f& input, T* output) {
    output[0] = T(input[0]);
    output[1] = T(input[1]);
    output[2] = T(input[2]);
}


/**
 * Pose increment is only an interface to the underlying array (in constructor, no copy
 * of the input array is made).
 * Important: Input array needs to have a size of at least 6.
 */
template <typename T>
class PoseIncrement {
public:
    explicit PoseIncrement(T* const array) : m_array{ array } { }

    void setZero() {
        for (int i = 0; i < 6; ++i)
            m_array[i] = T(0);
    }

    T* getData() const {
        return m_array;
    }

    /**
     * Applies the pose increment onto the input point and produces transformed output point.
     * Important: The memory for both 3D points (input and output) needs to be reserved (i.e. on the stack)
     * beforehand).
     */
    void apply(T* inputPoint, T* outputPoint) const {
        // pose[0,1,2] is angle-axis rotation.
        // pose[3,4,5] is translation.
        const T* rotation = m_array;
        const T* translation = m_array + 3;

        T temp[3];
        ceres::AngleAxisRotatePoint(rotation, inputPoint, temp);

        outputPoint[0] = temp[0] + translation[0];
        outputPoint[1] = temp[1] + translation[1];
        outputPoint[2] = temp[2] + translation[2];
    }

    /**
     * Converts the pose increment with rotation in SO3 notation and translation as 3D vector into
     * transformation 4x4 matrix.
     */
    static Matrix4f convertToMatrix(const PoseIncrement<double>& poseIncrement) {
        // pose[0,1,2] is angle-axis rotation.
        // pose[3,4,5] is translation.
        double* pose = poseIncrement.getData();
        double* rotation = pose;
        double* translation = pose + 3;

        // Convert the rotation from SO3 to matrix notation (with column-major storage).
        double rotationMatrix[9];
        ceres::AngleAxisToRotationMatrix(rotation, rotationMatrix);

        // Create the 4x4 transformation matrix.
        Matrix4f matrix;
        matrix.setIdentity();
        matrix(0, 0) = float(rotationMatrix[0]);	matrix(0, 1) = float(rotationMatrix[3]);	matrix(0, 2) = float(rotationMatrix[6]);	matrix(0, 3) = float(translation[0]);
        matrix(1, 0) = float(rotationMatrix[1]);	matrix(1, 1) = float(rotationMatrix[4]);	matrix(1, 2) = float(rotationMatrix[7]);	matrix(1, 3) = float(translation[1]);
        matrix(2, 0) = float(rotationMatrix[2]);	matrix(2, 1) = float(rotationMatrix[5]);	matrix(2, 2) = float(rotationMatrix[8]);	matrix(2, 3) = float(translation[2]);

        return matrix;
    }

private:
    T* m_array;
};


/**
 * Optimization constraints.
 */
class PointToPointConstraint {
public:
    PointToPointConstraint(const Vector3f& sourcePoint, const Vector3f& targetPoint, const float weight) :
        m_sourcePoint{ sourcePoint },
        m_targetPoint{ targetPoint },
        m_weight{ weight }
    { }

    template <typename T>
    bool operator()(const T* const pose, T* residuals) const {
        // Done: Implemented the point-to-point cost function.
        // The resulting 3D residual should be stored in residuals array. To apply the pose 
        // increment (pose parameters) to the source point, you can use the PoseIncrement
        // class.
        // Important: Ceres automatically squares the cost function.

        // transforming the source point using the pose increment
        T sourcePoint[3];
        fillVector(m_sourcePoint, sourcePoint);

        T transformedPoint[3];

        PoseIncrement<T> poseIncrement = PoseIncrement<T>(const_cast<T*>(pose));
        poseIncrement.apply(sourcePoint, transformedPoint);


        // computing the residuals and scaling by the weight
        // here we are not squaring the residuals since Ceres does that automatically
        residuals[0] = T(m_weight) * (transformedPoint[0] - T(m_targetPoint[0]));
        residuals[1] = T(m_weight) * (transformedPoint[1] - T(m_targetPoint[1]));
        residuals[2] = T(m_weight) * (transformedPoint[2] - T(m_targetPoint[2]));

        return true;
    }

    static ceres::CostFunction* create(const Vector3f& sourcePoint, const Vector3f& targetPoint, const float weight) {
        return new ceres::AutoDiffCostFunction<PointToPointConstraint, 3, 6>(
            new PointToPointConstraint(sourcePoint, targetPoint, weight)
            );
    }

protected:
    const Vector3f m_sourcePoint;
    const Vector3f m_targetPoint;
    const float m_weight;
    const float LAMBDA = 0.1f;
};

class PointToPlaneConstraint {
public:
    PointToPlaneConstraint(const Vector3f& sourcePoint, const Vector3f& targetPoint, const Vector3f& targetNormal, const float weight) :
        m_sourcePoint{ sourcePoint },
        m_targetPoint{ targetPoint },
        m_targetNormal{ targetNormal },
        m_weight{ weight }
    { }

    template <typename T>
    bool operator()(const T* const pose, T* residuals) const {
        // Done: Implemented the point-to-plane cost function.
        // The resulting 1D residual should be stored in residuals array. To apply the pose 
        // increment (pose parameters) to the source point, you can use the PoseIncrement
        // class.
        // Important: Ceres automatically squares the cost function.

        // transform the source point using the current pose increment
        T sourcePoint[3];
        fillVector(m_sourcePoint, sourcePoint);

        T transformedPoint[3];
        PoseIncrement<T> poseIncrement = PoseIncrement<T>(const_cast<T*>(pose));
        poseIncrement.apply(sourcePoint, transformedPoint);

        T targetPoint[3];
        fillVector(m_targetPoint, targetPoint);

        // poin-to-plane distance: dot product of point difference and normal
        residuals[0] = T(m_weight) * (
            (transformedPoint[0] - targetPoint[0]) * T(m_targetNormal[0]) +
            (transformedPoint[1] - targetPoint[1]) * T(m_targetNormal[1]) +
            (transformedPoint[2] - targetPoint[2]) * T(m_targetNormal[2])
        );

        return true;
    }


    static ceres::CostFunction* create(const Vector3f& sourcePoint, const Vector3f& targetPoint, const Vector3f& targetNormal, const float weight) {
        return new ceres::AutoDiffCostFunction<PointToPlaneConstraint, 1, 6>(
            new PointToPlaneConstraint(sourcePoint, targetPoint, targetNormal, weight)
            );
    }

protected:
    const Vector3f m_sourcePoint;
    const Vector3f m_targetPoint;
    const Vector3f m_targetNormal;
    const float m_weight;
    const float LAMBDA = 1.0f;
};


/**
 * ICP optimizer - Abstract Base Class
 */
class ICPOptimizer {
public:
    ICPOptimizer() :
        m_bUsePointToPlaneConstraints{ false },
        m_nIterations{ 20 },
        m_nearestNeighborSearch{ std::make_unique<NearestNeighborSearchFlann>() }
    { }

    void setMatchingMaxDistance(float maxDistance) {
        m_nearestNeighborSearch->setMatchingMaxDistance(maxDistance);
    }

    void usePointToPlaneConstraints(bool bUsePointToPlaneConstraints) {
        m_bUsePointToPlaneConstraints = bUsePointToPlaneConstraints;
    }

    void setNbOfIterations(unsigned nIterations) {
        m_nIterations = nIterations;
    }

    virtual void estimatePose(const PointCloud& source, const PointCloud& target, Matrix4f& initialPose) = 0;

protected:
    bool m_bUsePointToPlaneConstraints;
    unsigned m_nIterations;
    std::unique_ptr<NearestNeighborSearch> m_nearestNeighborSearch;

    std::vector<Vector3f> transformPoints(const std::vector<Vector3f>& sourcePoints, const Matrix4f& pose) {
        std::vector<Vector3f> transformedPoints;
        transformedPoints.reserve(sourcePoints.size());

        const auto rotation = pose.block(0, 0, 3, 3);
        const auto translation = pose.block(0, 3, 3, 1);

        for (const auto& point : sourcePoints) {
            transformedPoints.push_back(rotation * point + translation);
        }

        return transformedPoints;
    }

    std::vector<Vector3f> transformNormals(const std::vector<Vector3f>& sourceNormals, const Matrix4f& pose) {
        std::vector<Vector3f> transformedNormals;
        transformedNormals.reserve(sourceNormals.size());

        const auto rotation = pose.block(0, 0, 3, 3);

        for (const auto& normal : sourceNormals) {
            transformedNormals.push_back(rotation.inverse().transpose() * normal);
        }

        return transformedNormals;
    }

    void pruneCorrespondences(const std::vector<Vector3f>& sourceNormals, const std::vector<Vector3f>& targetNormals, std::vector<Match>& matches) {
        const unsigned nPoints = sourceNormals.size();
        // cos of 60 degrees
        const float threshold = cos(60.0f * M_PI / 180.0f);

        for (unsigned i = 0; i < nPoints; i++) {
            Match& match = matches[i];
            if (match.idx >= 0) {
                const auto& sourceNormal = sourceNormals[i];
                const auto& targetNormal = targetNormals[match.idx];

                // Done: Invalidate the match (set it to -1) if the angle between the normals is greater than 60
                // we calc the dot product between normals
                float dotProduct = sourceNormal.dot(targetNormal);

                // if angle is greater than 60 degrees (dot product < cos(60))
                // invalidate the match
                if (dotProduct < threshold) {
                    match.idx = -1;
                }
            }
        }
    }
};


/**
 * ICP optimizer - using Ceres for optimization.
 */
class CeresICPOptimizer : public ICPOptimizer {
public:
    CeresICPOptimizer() {}

    virtual void estimatePose(const PointCloud& source, const PointCloud& target, Matrix4f& initialPose) override {
        // Build the index of the FLANN tree (for fast nearest neighbor lookup).
        m_nearestNeighborSearch->buildIndex(target.getPoints());

        // The initial estimate can be given as an argument.
        Matrix4f estimatedPose = initialPose;

        // We optimize on the transformation in SE3 notation: 3 parameters for the axis-angle vector of the rotation (its length presents
        // the rotation angle) and 3 parameters for the translation vector. 
        double incrementArray[6];
        auto poseIncrement = PoseIncrement<double>(incrementArray);
        poseIncrement.setZero();

        for (int i = 0; i < m_nIterations; ++i) {
            // Compute the matches.
            std::cout << "Matching points ..." << std::endl;
            clock_t begin = clock();

            auto transformedPoints = transformPoints(source.getPoints(), estimatedPose);
            auto transformedNormals = transformNormals(source.getNormals(), estimatedPose);

            auto matches = m_nearestNeighborSearch->queryMatches(transformedPoints);
            pruneCorrespondences(transformedNormals, target.getNormals(), matches);

            clock_t end = clock();
            double elapsedSecs = double(end - begin) / CLOCKS_PER_SEC;
            std::cout << "Completed in " << elapsedSecs << " seconds." << std::endl;

            // Prepare point-to-point and point-to-plane constraints.
            ceres::Problem problem;
            prepareConstraints(transformedPoints, target.getPoints(), target.getNormals(), matches, poseIncrement, problem);

            // Configure options for the solver.
            ceres::Solver::Options options;
            configureSolver(options);

            // Run the solver (for one iteration).
            ceres::Solver::Summary summary;
            ceres::Solve(options, &problem, &summary);
            std::cout << summary.BriefReport() << std::endl;
            //std::cout << summary.FullReport() << std::endl;

            // Update the current pose estimate (we always update the pose from the left, using left-increment notation).
            Matrix4f matrix = PoseIncrement<double>::convertToMatrix(poseIncrement);
            estimatedPose = PoseIncrement<double>::convertToMatrix(poseIncrement) * estimatedPose;
            poseIncrement.setZero();

            std::cout << "Optimization iteration done." << std::endl;
        }

        // Store result
        initialPose = estimatedPose;
    }


private:
    void configureSolver(ceres::Solver::Options& options) {
        // Ceres options.
        options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
        options.use_nonmonotonic_steps = false;
        options.linear_solver_type = ceres::DENSE_QR;
        options.minimizer_progress_to_stdout = 1;
        options.max_num_iterations = 1;
        options.num_threads = 8;
    }

    void prepareConstraints(const std::vector<Vector3f>& sourcePoints, const std::vector<Vector3f>& targetPoints, const std::vector<Vector3f>& targetNormals, const std::vector<Match> matches, const PoseIncrement<double>& poseIncrement, ceres::Problem& problem) const {
        const unsigned nPoints = sourcePoints.size();

        for (unsigned i = 0; i < nPoints; ++i) {
            const auto match = matches[i];
            if (match.idx >= 0) {
                const auto& sourcePoint = sourcePoints[i];
                const auto& targetPoint = targetPoints[match.idx];

                if (!sourcePoint.allFinite() || !targetPoint.allFinite())
                    continue;

                // Done: Create a new point-to-point cost function and add it as constraint (i.e. residual block)
                // to the Ceres problem.
				// 3 is the number of residuals (3D point), 6 is the number of parameters being optimized (pose increment)

				/*ceres::CostFunction* pointToPointCost =
                new ceres::AutoDiffCostFunction<PointToPointCostFunctor, 3, 6>(
                    new PointToPointCostFunctor(sourcePoint, targetPoint));
				*/
				ceres::CostFunction* pointToPointCost = PointToPointConstraint::create(sourcePoint, targetPoint, 1.0f);

				problem.AddResidualBlock(
                    pointToPointCost,
                    nullptr, // squared loss
                    poseIncrement.getData() // try it with data() if this does not work
                );


                if (m_bUsePointToPlaneConstraints) {
                    const auto& targetNormal = targetNormals[match.idx];

                    if (!targetNormal.allFinite())
                        continue;

                    // Done: Create a new point-to-plane cost function and add it as constraint (i.e. residual block)
                    // to the Ceres problem.


					// 1 is the number of residuals (1D point-to-plane distance), 6 is the number of parameters being optimized (pose increment)
                    // Note: The point-to-plane cost function is defined as the distance from the transformed source point to the target point along the target normal
					/*ceres::CostFunction* pointToPlaneCost =
                    new ceres::AutoDiffCostFunction<PointToPlaneCostFunctor, 1, 6>(
                        new PointToPlaneCostFunctor(sourcePoint, targetPoint, targetNormal));

					*/
					ceres::CostFunction* pointToPlaneCost = PointToPlaneConstraint::create(sourcePoint, targetPoint, targetNormal, 1.0f);
    				problem.AddResidualBlock(pointToPlaneCost, nullptr, poseIncrement.getData());
                }
            }
        }
    }
};


/**
 * ICP optimizer - using linear least-squares for optimization.
 */
class LinearICPOptimizer : public ICPOptimizer {
public:
    LinearICPOptimizer() {}

    virtual void estimatePose(const PointCloud& source, const PointCloud& target, Matrix4f& initialPose) override {
        // Build the index of the FLANN tree (for fast nearest neighbor lookup).
        m_nearestNeighborSearch->buildIndex(target.getPoints());

        // The initial estimate can be given as an argument.
        Matrix4f estimatedPose = initialPose;

        for (int i = 0; i < m_nIterations; ++i) {
            // Compute the matches.
            std::cout << "Matching points ..." << std::endl;
            clock_t begin = clock();

            auto transformedPoints = transformPoints(source.getPoints(), estimatedPose);
            auto transformedNormals = transformNormals(source.getNormals(), estimatedPose);

            auto matches = m_nearestNeighborSearch->queryMatches(transformedPoints);
            pruneCorrespondences(transformedNormals, target.getNormals(), matches);

            clock_t end = clock();
            double elapsedSecs = double(end - begin) / CLOCKS_PER_SEC;
            std::cout << "Completed in " << elapsedSecs << " seconds." << std::endl;

            std::vector<Vector3f> sourcePoints;
            std::vector<Vector3f> targetPoints;

            // Add all matches to the sourcePoints and targetPoints vector,
            // so that the sourcePoints[i] matches targetPoints[i]. For every source point,
            // the matches vector holds the index of the matching target point.
            for (int j = 0; j < transformedPoints.size(); j++) {
                const auto& match = matches[j];
                if (match.idx >= 0) {
                    sourcePoints.push_back(transformedPoints[j]);
                    targetPoints.push_back(target.getPoints()[match.idx]);
                }
            }

            // Estimate the new pose
            if (m_bUsePointToPlaneConstraints) {
                estimatedPose = estimatePosePointToPlane(sourcePoints, targetPoints, target.getNormals()) * estimatedPose;
            }
            else {
                estimatedPose = estimatePosePointToPoint(sourcePoints, targetPoints) * estimatedPose;
            }

            std::cout << "Optimization iteration done." << std::endl;
        }

        // Store result
        initialPose = estimatedPose;
    }

private:
    Matrix4f estimatePosePointToPoint(const std::vector<Vector3f>& sourcePoints, const std::vector<Vector3f>& targetPoints) {
        ProcrustesAligner procrustAligner;
        Matrix4f estimatedPose = procrustAligner.estimatePose(sourcePoints, targetPoints);

        return estimatedPose;
    }

    Matrix4f estimatePosePointToPlane(const std::vector<Vector3f>& sourcePoints, const std::vector<Vector3f>& targetPoints, const std::vector<Vector3f>& targetNormals) {
        const unsigned nPoints = sourcePoints.size();

        // Build the system Ax = b
        MatrixXf A = MatrixXf::Zero(4 * nPoints, 6);
        VectorXf b = VectorXf::Zero(4 * nPoints);

        for (unsigned i = 0; i < nPoints; i++) {
            const auto& s = sourcePoints[i];
            const auto& d = targetPoints[i];
            const auto& n = targetNormals[i];

			if (!s.allFinite() || !d.allFinite() || !n.allFinite())
            continue;

            // Done: Add the point-to-plane constraints to the system (1 row)
			Vector3f cross = s.cross(n); // 3D: [s]_x(hoch t) * n
        	float dot = n.dot(s - d);


			A.row(4 * i).head<3>() = cross.transpose();   // rotation theta
        	A.row(4 * i).tail<3>() = n.transpose();       // translation t
        	b(4 * i) = -dot;

			//Done: Add the point-to-point constraints to the system (3 rows)
			Vector3f diff = s - d;
        	Matrix3f skew;
        	skew <<     0, -s.z(),  s.y(),
                 s.z(),     0, -s.x(),
                -s.y(),  s.x(),     0;

        	A.block<3,3>(4 * i + 1, 0) = skew;        // rotation
        	A.block<3,3>(4 * i + 1, 3) = Matrix3f::Identity();  // translation
        	b.segment<3>(4 * i + 1) = -diff;

            //Done: Optionally, apply a higher weight to point-to-plane correspondences
			//weight point-to-plane residual more
			A.row(4 * i) *= 2.0f;
        	b(4 * i) *= 2.0f;

        }

        // Done: Solve the system -> solve the least squares problem Ax = b using the normal equations.
        VectorXf x = (A.transpose() * A).ldlt().solve(A.transpose() * b);


        float alpha = x(0), beta = x(1), gamma = x(2);

        // Build the pose matrix - euler angle order is XYZ
        Matrix3f rotation = AngleAxisf(alpha, Vector3f::UnitX()).toRotationMatrix() *
            AngleAxisf(beta, Vector3f::UnitY()).toRotationMatrix() *
            AngleAxisf(gamma, Vector3f::UnitZ()).toRotationMatrix();

        Vector3f translation = x.tail(3);

        // Done: Build the pose matrix using the rotation and translation matrices
        Matrix4f estimatedPose = Matrix4f::Identity();
		estimatedPose.block<3,3>(0,0) = rotation;
    	estimatedPose.block<3,1>(0,3) = translation;

        return estimatedPose;
    }
};
