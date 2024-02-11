#include "benchmarkCommon.h"
#include "scene/scene_manufacuting.h"
#include <moveit/collision_detection_bullet/collision_env_bullet.h>
#include <moveit/collision_detection_bullet/collision_detector_allocator_bullet.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "benchmarkBullet");
  std::shared_ptr<collision_detection::CollisionDetectorAllocatorBullet> value_ = std::make_shared<collision_detection::CollisionDetectorAllocatorBullet>();
  robot_model_loader::RobotModelLoader rm_loader_("robot_description");
  robot_model::RobotModelPtr robot_model = rm_loader_.getModel();
  collision_detection::CollisionEnvPtr cenv_ = value_->allocateEnv(robot_model);
  moveit::core::RobotStatePtr robot_state_ = std::make_shared<moveit::core::RobotState>(robot_model);

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

  // Start Trajectory Collision Detection
  start = std::chrono::high_resolution_clock::now();
  for (auto &statePair : statePairs)
  {
    collision_detection::CollisionResult c_res;
    cenv_->checkRobotCollision(req, c_res, statePair.first, statePair.second);
  }
  end = std::chrono::high_resolution_clock::now();
  elapsed = end - start;
  std::cout << "Time for trajectory collision detection: " << elapsed.count() << " ms" << std::endl;
}