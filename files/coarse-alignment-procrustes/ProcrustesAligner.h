#pragma once
#include "SimpleMesh.h"

class ProcrustesAligner {
public:
	Matrix4f estimatePose(const std::vector<Vector3f>& sourcePoints, const std::vector<Vector3f>& targetPoints) {
		ASSERT(sourcePoints.size() == targetPoints.size() && "The number of source and target points should be the same, since every source point is matched with corresponding target point.");

		// We estimate the pose between source and target points using Procrustes algorithm.
		// Our shapes have the same scale, therefore we don't estimate scale. We estimated rotation and translation
		// from source points to target points.

		auto sourceMean = computeMean(sourcePoints);
		auto targetMean = computeMean(targetPoints);
		
		Matrix3f rotation = estimateRotation(sourcePoints, sourceMean, targetPoints, targetMean);
		Vector3f translation = computeTranslation(sourceMean, targetMean, rotation);

		// TODO: Compute the transformation matrix by using the computed rotation and translation.
		// You can access parts of the matrix with .block(start_row, start_col, num_rows, num_cols) = elements
		
		Matrix4f estimatedPose = Matrix4f::Identity();
		return estimatedPose;
	}

private:
	Vector3f computeMean(const std::vector<Vector3f>& points) {
		// Done: Compute the mean of input points.
		// Hint: You can use the .size() method to get the length of a vector.

		Vector3f mean = Vector3f::Zero();
		if (points.empty()) {
			return mean;
		}

		for (const Vector3f& point : points) {
			mean += point;
		}

		mean /= static_cast<float>(points.size());
		return mean;
	}

	/*Matrix3f estimateRotation(const std::vector<Vector3f>& sourcePoints, const Vector3f& sourceMean, const std::vector<Vector3f>& targetPoints, const Vector3f& targetMean) {
		// TODO: Estimate the rotation from source to target points, following the Procrustes algorithm.
		// To compute the singular value decomposition you can use JacobiSVD() from Eigen.
		// Hint: You can initialize an Eigen matrix with "MatrixXf m(num_rows,num_cols);" and access/modify parts of it using the .block() method (see above).

		Matrix3f rotation = Matrix3f::Identity(); 
        return rotation;
	}
*/

	Matrix3f estimateRotation(const std::vector<Vector3f>& sourcePoints, const Vector3f& sourceMean, const std::vector<Vector3f>& targetPoints, const Vector3f& targetMean) {
		// Compute the centered vectors
		const size_t numPoints = sourcePoints.size();
		MatrixXf sourceCentered(3, numPoints);
		MatrixXf targetCentered(3, numPoints);

		for (size_t i = 0; i < numPoints; ++i) {
			sourceCentered.col(i) = sourcePoints[i] - sourceMean;
			targetCentered.col(i) = targetPoints[i] - targetMean;
		}

		// Compute the covariance matrix
		Matrix3f covariance = sourceCentered * targetCentered.transpose();

		// Compute SVD of the covariance matrix
		JacobiSVD<Matrix3f> svd(covariance, ComputeFullU | ComputeFullV);
		Matrix3f U = svd.matrixU();
		Matrix3f V = svd.matrixV();

		// Compute the rotation matrix
		Matrix3f rotation = V * U.transpose();

		// Handle reflection case
		if (rotation.determinant() < 0) {
			V.col(2) = -V.col(2);
			rotation = V * U.transpose();
		}

		return rotation;
	}

	Vector3f computeTranslation(const Vector3f& sourceMean, const Vector3f& targetMean, const Matrix3f& rotation) {
		// Done: Compute the translation vector from source to target points.
		Vector3f translation = Vector3f::Zero();
		// translation = targetMean - R * sourceMean
		translation = targetMean - rotation * sourceMean;
        return translation;
	}
};
