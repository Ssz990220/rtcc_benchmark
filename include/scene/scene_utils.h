#pragma once


#include <geometric_shapes/mesh_operations.h>
#include <geometric_shapes/mesh_operations.h>
#include <moveit/planning_scene/planning_scene.h>



void add_mesh(collision_detection::CollisionEnvPtr planning_scene, std::string path, std::string name, Eigen::Isometry3d &pose)
{
    auto mesh = std::shared_ptr<shapes::Mesh>(shapes::createMeshFromResource(path));
    planning_scene->getWorld()->addToObject(name, mesh, pose);
}

void add_cube(collision_detection::CollisionEnvPtr planning_scene, std::array<double, 6> &cubeInfo, std::string name)
{

    // Create a shared pointer to a new box shape
    shapes::ShapePtr cube = std::make_shared<shapes::Box>(cubeInfo[0], cubeInfo[1], cubeInfo[2]);
    // Define the pose of the cube
    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    pose.translation() = Eigen::Vector3d(cubeInfo[3], cubeInfo[4], cubeInfo[5]);

    // Add the cube to the planning scene
    planning_scene->getWorld()->addToObject(name, cube, pose);
}