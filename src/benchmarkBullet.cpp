#include <moveit/utils/robot_model_test_utils.h>
#include <geometric_shapes/mesh_operations.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_state/conversions.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/collision_detection_bullet/collision_env_bullet.h>
#include <moveit/collision_detection_bullet/collision_detector_allocator_bullet.h>
#include "benchmarkCommon.h"

inline void setToHome(moveit::core::RobotState &panda_state)
{
  panda_state.setToDefaultValues();
  double joint2 = -0.785;
  double joint4 = -2.356;
  double joint6 = 1.571;
  double joint7 = 0.785;
  panda_state.setJointPositions("panda_joint2", &joint2);
  panda_state.setJointPositions("panda_joint4", &joint4);
  panda_state.setJointPositions("panda_joint6", &joint6);
  panda_state.setJointPositions("panda_joint7", &joint7);
  panda_state.update();
}

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

void setCollisionScene(collision_detection::CollisionEnvPtr planning_scene)
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

  std::string meshPath = "file://" + ros::package::getPath("scene_102") + "/models/PipeLong.STL";
  Eigen::Isometry3d meshPose = Eigen::Isometry3d::Identity();

  constexpr double x = 0.35355339059327384;
  constexpr double y = 0.6123724356957945;
  constexpr double z = 0.6123724356957945;
  constexpr double w = 0.35355339059327384;
  Eigen::Quaterniond quat(w, x, y, z); // w is the scalar part, and x, y, z are the vector part of the quaternion

  // Set the translation part
  meshPose.translation() << -0.23, -0.37, 0.35;
  meshPose.rotate(quat);

  add_mesh(planning_scene, meshPath, "pipe", meshPose);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "benchmarkBullet");
  std::shared_ptr<collision_detection::CollisionDetectorAllocatorBullet> value_ = std::make_shared<collision_detection::CollisionDetectorAllocatorBullet>();
  robot_model_loader::RobotModelLoader rm_loader_("robot_description");
  robot_model::RobotModelPtr robot_model = rm_loader_.getModel();
  collision_detection::CollisionEnvPtr cenv_ = value_->allocateEnv(robot_model);
  moveit::core::RobotStatePtr robot_state_ = std::make_shared<moveit::core::RobotState>(robot_model);
  // setToHome(*robot_state_);

  setCollisionScene(cenv_);

  std::vector<moveit::core::RobotState> states;
  std::vector<std::pair<moveit::core::RobotState, moveit::core::RobotState>> statePairs;

  loadPoses(states, statePairs, *robot_state_);
  std::cout << "Load " << states.size() << " poses from file" << std::endl;

  std::cout << "Found " << cenv_->getWorld()->getObjectIds().size() << " objects in the planning scene" << std::endl;
  auto world = cenv_->getWorld()->getObjectIds();
  for (auto it = world.begin(); it != world.end(); it++)
  {
    std::cout << "Object " << *it << std::endl;
    auto shape = cenv_->getWorld()->getObject(*it)->shapes_.front();
    std::cout << "Shape " << shape->type << std::endl;
    shape->print(std::cout);
  }
  // Start Collision Detection
  collision_detection::CollisionRequest req;
  req.group_name = "arm";
  collision_detection::CollisionResult res;
  // Static Poses First
  auto start = std::chrono::high_resolution_clock::now();
  for (auto &state : states)
  {
    cenv_->checkRobotCollision(req, res, state);
  }

  auto end = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double, std::milli> elapsed = end - start;
  std::cout << "Time for trajectory collision detection: " << elapsed.count() << " ms" << std::endl;
}