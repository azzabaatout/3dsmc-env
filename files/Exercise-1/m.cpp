bool WriteMesh(Vertex* vertices, unsigned int width, unsigned int height, const std::string& filename)
{
    float edgeThreshold = 0.01f; // 1cm

    // Get number of vertices (width * height)
    unsigned int nVertices = width * height;

    // Count valid faces by checking triangles in the grid
    unsigned int nFaces = 0;
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

    // Write header
    outFile << "COFF" << std::endl;
    outFile << "# numVertices numFaces numEdges" << std::endl;
    outFile << nVertices << " " << nFaces << " 0" << std::endl;

    // Save vertices
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

    // Save valid faces
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

    outFile.close();
    return true;
}