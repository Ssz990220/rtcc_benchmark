#pragma once

#include <geometric_shapes/mesh_operations.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

void add_mesh(std::shared_ptr<planning_scene::PlanningScene> &planning_scene, std::string path, std::string name, Eigen::Isometry3d &pose)
{
    auto mesh = std::shared_ptr<shapes::Mesh>(shapes::createMeshFromResource(path));
    planning_scene->getWorldNonConst()->addToObject(name, mesh, pose);
}

void add_cube(std::shared_ptr<planning_scene::PlanningScene> &planning_scene, std::array<double, 6> &cubeInfo, std::string name)
{

    // Create a shared pointer to a new box shape
    shapes::ShapePtr cube = std::make_shared<shapes::Box>(cubeInfo[0], cubeInfo[1], cubeInfo[2]);
    // Define the pose of the cube
    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    pose.translation() = Eigen::Vector3d(cubeInfo[3], cubeInfo[4], cubeInfo[5]);

    // Add the cube to the planning scene
    planning_scene->getWorldNonConst()->addToObject(name, cube, pose);
}


void add_mesh(std::shared_ptr<planning_scene::PlanningScene> &planning_scene, std::string path, std::string name, Eigen::Isometry3d &pose, visualization_msgs::MarkerArray &g_collision_objs)
{
  auto mesh = std::shared_ptr<shapes::Mesh>(shapes::createMeshFromResource(path));
  planning_scene->getWorldNonConst()->addToObject(name, mesh, pose);

  visualization_msgs::Marker g_mesh;
  g_mesh.header.frame_id = "base";
  g_mesh.header.stamp = ros::Time::now();
  g_mesh.ns = name;
  g_mesh.type = visualization_msgs::Marker::MESH_RESOURCE;
  g_mesh.action = visualization_msgs::Marker::ADD;
  g_mesh.pose.position.x = pose.translation().x();
  g_mesh.pose.position.y = pose.translation().y();
  g_mesh.pose.position.z = pose.translation().z();
  Eigen::Quaterniond quat(pose.rotation());
  g_mesh.pose.orientation.x = quat.x();
  g_mesh.pose.orientation.y = quat.y();
  g_mesh.pose.orientation.z = quat.z();
  g_mesh.pose.orientation.w = quat.w();
  g_mesh.scale.x = 1;
  g_mesh.scale.y = 1;
  g_mesh.scale.z = 1;
  g_mesh.color.a = 0.8; // Don't forget to set the alpha!
  g_mesh.color.r = 0.0;
  g_mesh.color.g = 1.0;
  g_mesh.color.b = 0.0;
  g_mesh.lifetime = ros::Duration();
  g_mesh.mesh_resource = path;
  g_collision_objs.markers.push_back(g_mesh);
}

void add_cube(std::shared_ptr<planning_scene::PlanningScene> &planning_scene, std::array<double, 6> &cubeInfo, std::string name, visualization_msgs::MarkerArray &g_collision_objs)
{

  // Create a shared pointer to a new box shape
  shapes::ShapePtr cube = std::make_shared<shapes::Box>(cubeInfo[0], cubeInfo[1], cubeInfo[2]);
  // Define the pose of the cube
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  pose.translation() = Eigen::Vector3d(cubeInfo[3], cubeInfo[4], cubeInfo[5]);

  // Add the cube to the planning scene
  planning_scene->getWorldNonConst()->addToObject(name, cube, pose);

  // Add to marker array
  visualization_msgs::Marker marker;
  marker.header.frame_id = "base";
  marker.header.stamp = ros::Time::now();
  marker.ns = name;
  marker.type = visualization_msgs::Marker::CUBE;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position.x = cubeInfo[3];
  marker.pose.position.y = cubeInfo[4];
  marker.pose.position.z = cubeInfo[5];
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = cubeInfo[0];
  marker.scale.y = cubeInfo[1];
  marker.scale.z = cubeInfo[2];
  marker.color.a = 0.8; // Don't forget to set the alpha!
  marker.color.r = 0.0;
  marker.color.g = 1.0;
  marker.color.b = 0.0;
  marker.lifetime = ros::Duration();
  g_collision_objs.markers.push_back(marker);
}
