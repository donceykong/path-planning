#include <octomap/octomap.h>
#include <iostream>
#include <fstream>
#include <vector>

// Function to read .bin file and parse point cloud data
std::vector<octomap::point3d> readPointCloudFromBin(const std::string& filename) {
    std::vector<octomap::point3d> points;

    std::ifstream input_file(filename, std::ios::binary);
    if (!input_file.is_open()) {
        std::cerr << "Could not open the file!" << std::endl;
        return points;
    }

    // Assuming the file contains float values in x, y, z order
    float x, y, z;
    while (input_file.read(reinterpret_cast<char*>(&x), sizeof(float)) &&
           input_file.read(reinterpret_cast<char*>(&y), sizeof(float)) &&
           input_file.read(reinterpret_cast<char*>(&z), sizeof(float))) {
        points.emplace_back(x, y, z);
    }

    input_file.close();
    return points;
}

int main() {
    // File path to the .bin file containing point cloud data
    std::string filename = "pointcloud.bin";

    // Read point cloud data from the .bin file
    std::vector<octomap::point3d> point_cloud = readPointCloudFromBin(filename);

    // Create an OctoMap object with a resolution of 0.1 meters
    octomap::OcTree tree(0.1);

    // Insert the point cloud data into the octree
    octomap::point3d sensor_origin(0, 0, 0); // Assume the sensor is at the origin
    for (const auto& point : point_cloud) {
        tree.updateNode(point, true); // Mark the node as occupied
    }

    // Query the occupancy status of a voxel (example query)
    octomap::OcTreeNode* node = tree.search(1.0, 1.0, 1.0);
    if (node != nullptr) {
        if (node->getOccupancy() > 0.5) {
            std::cout << "The voxel is occupied." << std::endl;
        } else {
            std::cout << "The voxel is free." << std::endl;
        }
    } else {
        std::cout << "The voxel is unknown." << std::endl;
    }

    // Save the OctoMap to a file
    tree.writeBinary("octomap.bt");

    return 0;
}

