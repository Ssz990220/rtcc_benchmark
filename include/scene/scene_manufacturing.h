#pragma once

#include "scene_utils.h"
#include <ros/package.h>
#include <geometric_shapes/mesh_operations.h>
#include <moveit/planning_scene/planning_scene.h>

void setCollisionScene(std::shared_ptr<planning_scene::PlanningScene> &planning_scene)
{
  std::array<double, 6> cubeInfo = {0.6, 1.2, 0.74, 0, -0.45, -0.45};
  add_cube(planning_scene, cubeInfo, "table1");
  std::array<double, 6> cubeInfo2 = {0.51, 0.52, 1, 0.46, 0.76, -0.24};
  add_cube(planning_scene, cubeInfo2, "table2");
  std::array<double, 6> cubeInfo3 = {0.5, 0.7, 0.8, -1.77, -0.1, -0.34};
  add_cube(planning_scene, cubeInfo3, "table3");
  std::array<double, 6> cubeInfo4 = {0.1, 1.2, 0.8, -0.25, -0.45, -0.4};
  add_cube(planning_scene, cubeInfo4, "table4");
  std::array<double, 6> cubeInfo5 = {0.4, 1.0, 0.8, 0, -0.6, -0.4};
  add_cube(planning_scene, cubeInfo5, "table5");
  std::array<double, 6> cubeInfo6 = {0.4, 0.75, 0.85, 0, -0.825, -0.315};
  add_cube(planning_scene, cubeInfo6, "table6");
  std::array<double, 6> cubeInfo7 = {0.1, 1.2, 0.8, 0.25, -0.45, -0.4};
  add_cube(planning_scene, cubeInfo7, "table7");

  // std::array<double, 6> cubeInfo8 = {3,0.1,2.5, -0.75, 1, 0.5};
  // add_cube(cubeInfo8, "wall1");
  // std::array<double, 6> cubeInfo9 = {0.1,2.6,2.5, 0.84, -0.3, 0.5};
  // add_cube(cubeInfo9, "wall2");
  // std::array<double, 6> cubeInfo10 = {3,2.9,0.1, -0.75,-0.45,-0.79};
  // add_cube(cubeInfo10, "floor");
  // std::array<double, 6> cubeInfo11 = {3,2.9,0.1, -0.75,-0.45,1.5};
  // add_cube(cubeInfo11, "ceiling");
  // std::array<double, 6> cubeInfo12 = {0.9,0.4,2.5, 0.39,-1.8,0.5};
  // add_cube(cubeInfo12, "wall3");
  // std::array<double, 6> cubeInfo13 = {3,0.1,2.5, -0.75,-1.9,0.5};
  // add_cube(cubeInfo13, "wall4");

  std::string meshPath = "file://" + ros::package::getPath("scene_102") + "/models/PipeLong.obj";
  Eigen::Isometry3d meshPose = Eigen::Isometry3d::Identity();

  constexpr double x = 0.0;
  constexpr double y = 0.0;
  constexpr double z = 0.866025;
  constexpr double w = 0.5;
  Eigen::Quaterniond quat(w, x, y, z); // w is the scalar part, and x, y, z are the vector part of the quaternion

  // Set the translation part
  meshPose.translation() << -0.23, -0.37, 0.35;
  meshPose.rotate(quat);

  add_mesh(planning_scene, meshPath, "pipe", meshPose);
}

void setCollisionScene(std::shared_ptr<planning_scene::PlanningScene> &planning_scene, visualization_msgs::MarkerArray &g_collision_objs, std::unique_ptr<ros::Publisher> &g_col_marker_publisher)
{
  std::array<double, 6> cubeInfo = {0.6, 1.2, 0.74, 0, -0.45, -0.45};
  add_cube(planning_scene, cubeInfo, "table1", g_collision_objs);
  std::array<double, 6> cubeInfo2 = {0.51, 0.52, 1, 0.46, 0.76, -0.24};
  add_cube(planning_scene, cubeInfo2, "table2", g_collision_objs);
  std::array<double, 6> cubeInfo3 = {0.5, 0.7, 0.8, -1.77, -0.1, -0.34};
  add_cube(planning_scene, cubeInfo3, "table3", g_collision_objs);
  std::array<double, 6> cubeInfo4 = {0.1, 1.2, 0.8, -0.25, -0.45, -0.4};
  add_cube(planning_scene, cubeInfo4, "table4", g_collision_objs);
  std::array<double, 6> cubeInfo5 = {0.4, 1.0, 0.8, 0, -0.6, -0.4};
  add_cube(planning_scene, cubeInfo5, "table5"), g_collision_objs;
  std::array<double, 6> cubeInfo6 = {0.4, 0.75, 0.85, 0, -0.825, -0.315};
  add_cube(planning_scene, cubeInfo6, "table6", g_collision_objs);
  std::array<double, 6> cubeInfo7 = {0.1, 1.2, 0.8, 0.25, -0.45, -0.4};
  add_cube(planning_scene, cubeInfo7, "table7", g_collision_objs);

  // std::array<double, 6> cubeInfo8 = {3,0.1,2.5, -0.75, 1, 0.5};
  // add_cube(cubeInfo8, "wall1");
  // std::array<double, 6> cubeInfo9 = {0.1,2.6,2.5, 0.84, -0.3, 0.5};
  // add_cube(cubeInfo9, "wall2");
  // std::array<double, 6> cubeInfo10 = {3,2.9,0.1, -0.75,-0.45,-0.79};
  // add_cube(cubeInfo10, "floor");
  // std::array<double, 6> cubeInfo11 = {3,2.9,0.1, -0.75,-0.45,1.5};
  // add_cube(cubeInfo11, "ceiling");
  // std::array<double, 6> cubeInfo12 = {0.9,0.4,2.5, 0.39,-1.8,0.5};
  // add_cube(cubeInfo12, "wall3");
  // std::array<double, 6> cubeInfo13 = {3,0.1,2.5, -0.75,-1.9,0.5};
  // add_cube(cubeInfo13, "wall4");

  std::string meshPath = "file://" + ros::package::getPath("scene_102") + "/models/PipeLong.obj";
  Eigen::Isometry3d meshPose = Eigen::Isometry3d::Identity();

  constexpr double x = 0.0;
  constexpr double y = 0.0;
  constexpr double z = 0.866025;
  constexpr double w = 0.5;
  Eigen::Quaterniond quat(w, x, y, z); // w is the scalar part, and x, y, z are the vector part of the quaternion

  // Set the translation part
  meshPose.translation() << -0.23, -0.37, 0.35;
  meshPose.rotate(quat);

  add_mesh(planning_scene, meshPath, "pipe", meshPose, g_collision_objs);

  g_col_marker_publisher->publish(g_collision_objs);
}