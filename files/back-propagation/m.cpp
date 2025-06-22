#include <iostream>
#include <fstream>
#include <array>

#include "Eigen.h"
#include "VirtualSensor.h"

struct Vertex
{
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	// position stored as 4 floats (4th component is supposed to be 1.0)
	Vector4f position;
	// color stored as 4 unsigned char
	Vector4uc color;
};

bool WriteMesh(Vertex* vertices, unsigned int width, unsigned int height, const std::string& filename)
{
	float edgeThreshold = 0.01f; // 1cm

	// TODO 2: use the OFF file format to save the vertices grid (http://www.geomview.org/docs/html/OFF.html)
	// - have a look at the "off_sample.off" file to see how to store the vertices and triangles
	// - for debugging we recommend to first only write out the vertices (set the number of faces to zero)
	// - for simplicity write every vertex to file, even if it is not valid (position.x() == MINF) (note that all vertices in the off file have to be valid, thus, if a point is not valid write out a dummy point like (0,0,0))
	// - use a simple triangulation exploiting the grid structure (neighboring vertices build a triangle, two triangles per grid cell)
	// - you can use an arbitrary triangulation of the cells, but make sure that the triangles are consistently oriented
	// - only write triangles with valid vertices and an edge length smaller then edgeThreshold

	// TODO: Get number of vertices
	unsigned int nVertices  = width * height;

	// TODO: Determine number of valid faces
	// Count valid faces by checking triangles in the grid
	unsigned nFaces = 0;
	for (unsigned int y = 0; y < height - 1; y++) {
        for (unsigned int x = 0; x < width - 1; x++) {
            unsigned int idx0 = y * width + x;
            unsigned int idx1 = y * width + (x + 1);
            unsigned int idx2 = (y + 1) * width + x;
            unsigned int idx3 = (y + 1) * width + (x + 1);

            // Check if vertices are valid
            bool valid0 = vertices[idx0].position[0] != MINF;
            bool valid1 = vertices[idx1].position[0] != MINF;
            bool valid2 = vertices[idx2].position[0] != MINF;
            bool valid3 = vertices[idx3].position[0] != MINF;

            // Check first triangle (0,1,2)
            if (valid0 && valid1 && valid2) {
                Vector4f edge1 = vertices[idx1].position - vertices[idx0].position;
                Vector4f edge2 = vertices[idx2].position - vertices[idx0].position;
                if (edge1.head<3>().norm() < edgeThreshold && edge2.head<3>().norm() < edgeThreshold) {
                    nFaces++;
                }
            }

            // Check second triangle (1,3,2)
            if (valid1 && valid3 && valid2) {
                Vector4f edge1 = vertices[idx3].position - vertices[idx1].position;
                Vector4f edge2 = vertices[idx2].position - vertices[idx1].position;
                if (edge1.head<3>().norm() < edgeThreshold && edge2.head<3>().norm() < edgeThreshold) {
                    nFaces++;
                }
            }
        }
    }

	// Write off file
	std::ofstream outFile(filename);
	if (!outFile.is_open()) return false;

	// write header
	outFile << "COFF" << std::endl;

	outFile << "# numVertices numFaces numEdges" << std::endl;

	outFile << nVertices << " " << nFaces << " 0" << std::endl;

	// TODO: save vertices
	for (unsigned int i = 0; i < nVertices; i++) {
        if (vertices[i].position[0] == MINF) {
            outFile << "0 0 0 " << (int)vertices[i].color[0] << " "
                   << (int)vertices[i].color[1] << " "
                   << (int)vertices[i].color[2] << " "
                   << (int)vertices[i].color[3] << std::endl;
        } else {
            outFile << vertices[i].position[0] << " "
                   << vertices[i].position[1] << " "
                   << vertices[i].position[2] << " "
                   << (int)vertices[i].color[0] << " "
                   << (int)vertices[i].color[1] << " "
                   << (int)vertices[i].color[2] << " "
                   << (int)vertices[i].color[3] << std::endl;
        }
    }

	// TODO: save valid faces
	std::cout << "# list of faces" << std::endl;
	std::cout << "# nVerticesPerFace idx0 idx1 idx2 ..." << std::endl;
	for (unsigned int y = 0; y < height - 1; y++) {
        for (unsigned int x = 0; x < width - 1; x++) {
            unsigned int idx0 = y * width + x;
            unsigned int idx1 = y * width + (x + 1);
            unsigned int idx2 = (y + 1) * width + x;
            unsigned int idx3 = (y + 1) * width + (x + 1);

            bool valid0 = vertices[idx0].position[0] != MINF;
            bool valid1 = vertices[idx1].position[0] != MINF;
            bool valid2 = vertices[idx2].position[0] != MINF;
            bool valid3 = vertices[idx3].position[0] != MINF;

            // Write first triangle
            if (valid0 && valid1 && valid2) {
                Vector4f edge1 = vertices[idx1].position - vertices[idx0].position;
                Vector4f edge2 = vertices[idx2].position - vertices[idx0].position;
                if (edge1.head<3>().norm() < edgeThreshold && edge2.head<3>().norm() < edgeThreshold) {
                    outFile << "3 " << idx0 << " " << idx1 << " " << idx2 << std::endl;
                }
            }

            // Write second triangle
            if (valid1 && valid3 && valid2) {
                Vector4f edge1 = vertices[idx3].position - vertices[idx1].position;
                Vector4f edge2 = vertices[idx2].position - vertices[idx1].position;
                if (edge1.head<3>().norm() < edgeThreshold && edge2.head<3>().norm() < edgeThreshold) {
                    outFile << "3 " << idx1 << " " << idx3 << " " << idx2 << std::endl;
                }
            }
        }
    }

	// close file
	outFile.close();

	return true;
}


int main()
{
	// Make sure this path points to the data folder
	std::string filenameIn = "../Data/rgbd_dataset_freiburg1_xyz/";
	std::string filenameBaseOut = "mesh_";

	// load video
	std::cout << "Initialize virtual sensor..." << std::endl;
	VirtualSensor sensor;
	if (!sensor.Init(filenameIn))
	{
		std::cout << "Failed to initialize the sensor!\nCheck file path!" << std::endl;
		return -1;
	}

	// convert video to meshes_result
	while (sensor.ProcessNextFrame())
	{
		// get ptr to the current depth frame
		// depth is stored in row major (get dimensions via sensor.GetDepthImageWidth() / GetDepthImageHeight())
		float* depthMap = sensor.GetDepth();
		// get ptr to the current color frame
		// color is stored as RGBX in row major (4 byte values per pixel, get dimensions via sensor.GetColorImageWidth() / GetColorImageHeight())
		BYTE* colorMap = sensor.GetColorRGBX();

		// get depth intrinsics
		Matrix3f depthIntrinsics = sensor.GetDepthIntrinsics();
		Matrix3f depthIntrinsicsInv = depthIntrinsics.inverse();

		float fX = depthIntrinsics(0, 0);
		float fY = depthIntrinsics(1, 1);
		float cX = depthIntrinsics(0, 2);
		float cY = depthIntrinsics(1, 2);

		// compute inverse depth extrinsics
		Matrix4f depthExtrinsicsInv = sensor.GetDepthExtrinsics().inverse();

		Matrix4f trajectory = sensor.GetTrajectory();
		Matrix4f trajectoryInv = sensor.GetTrajectory().inverse();

		// TODO 1: back-projection
		// write result to the vertices array below, keep pixel ordering!
		// if the depth value at idx is invalid (MINF) write the following values to the vertices array
		// vertices[idx].position = Vector4f(MINF, MINF, MINF, MINF);
		// vertices[idx].color = Vector4uc(0,0,0,0);
		// otherwise apply back-projection and transform the vertex to world space, use the corresponding color from the colormap

		Vertex* vertices = new Vertex[sensor.GetDepthImageWidth() * sensor.GetDepthImageHeight()];

		// Back-projection for each pixel
		for (unsigned int y = 0; y < sensor.GetDepthImageHeight(); ++y) {
			for (unsigned int x = 0; x < sensor.GetDepthImageWidth(); ++x) {
				unsigned int idx = y * sensor.GetDepthImageWidth() + x;
				float depth = depthMap[idx];

				if (depth == MINF) {
					vertices[idx].position = Vector4f(MINF, MINF, MINF, MINF);
					vertices[idx].color = Vector4uc(0, 0, 0, 0);
				} else {
					// Back-project from image space to camera space
					float camX = (x - cX) * depth / fX;
					float camY = (y - cY) * depth / fY;
					float camZ = depth;

					// transform vertex position from camera to world space
					Vector4f camPoint(camX, camY, camZ, 1.0f);
					Vector4f worldPoint = trajectory * depthExtrinsicsInv * camPoint;

					// store the world position
					vertices[idx].position = worldPoint;

					// get color from colorMap (RGBX format)
					vertices[idx].color = Vector4uc(
						colorMap[4 * idx],     // R
						colorMap[4 * idx + 1], // G
						colorMap[4 * idx + 2], // B
						255                    // A (set to fully opaque)
					);
				}
			}
		}


		// write mesh file
		std::stringstream ss;
		ss << filenameBaseOut << sensor.GetCurrentFrameCnt() << "file.off";
		if (!WriteMesh(vertices, sensor.GetDepthImageWidth(), sensor.GetDepthImageHeight(), ss.str()))
		{
			std::cout << "Failed to write mesh!\nCheck file path!" << std::endl;
			return -1;
		}

		// free mem
		delete[] vertices;
	}

	return 0;
}