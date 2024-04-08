#include "benchmarkCommon.h"
#include "scene/scene_manufacuting.h"
#include "robot/jaka.h"
#include <moveit/collision_detection_bullet/collision_detector_allocator_bullet.h>
#define SAVE_RESULT

int main(int argc, char **argv)
{
  ros::init(argc, argv, "benchmarkBullet");
  std::shared_ptr<collision_detection::CollisionDetectorAllocatorBullet> value_ = std::make_shared<collision_detection::CollisionDetectorAllocatorBullet>();
  robot_model_loader::RobotModelLoader rm_loader_("robot_description");
  robot_model::RobotModelPtr robot_model = rm_loader_.getModel();
    auto planning_scene = std::make_shared<planning_scene::PlanningScene>(robot_model);
    moveit::core::RobotState robot_state_ = planning_scene->getCurrentStateNonConst();

  setCollisionScene(planning_scene);

  std::vector<moveit::core::RobotState> states;
  std::vector<std::pair<moveit::core::RobotState, moveit::core::RobotState>> statePairs;

  BenchmarkRobot::loadPoses(states, statePairs, robot_state_);
  // std::cout << "Load " << states.size() << " poses from file" << std::endl;

  // std::cout << "Found " << cenv_->getWorld()->getObjectIds().size() << " objects in the planning scene" << std::endl;
  // auto world = cenv_->getWorld()->getObjectIds();
  // for (auto it = world.begin(); it != world.end(); it++)
  // {
  //   std::cout << "Object " << *it << std::endl;
  //   auto shape = cenv_->getWorld()->getObject(*it)->shapes_.front();
  //   std::cout << "Shape " << shape->type << std::endl;
  //   shape->print(std::cout);
  // }

#if defined(SAVE_RESULT)
    std::vector<int> results(states.size());
    std::vector<int> resultsTraj(statePairs.size());
#endif
  // Start Collision Detection
  collision_detection::CollisionRequest req;
  req.group_name = moveGroupName;
  collision_detection::CollisionResult res;
  // Static Poses First
  auto start = std::chrono::high_resolution_clock::now();
  for (auto &state : states)
  {
    // state.updateCollisionBodyTransforms();
    planning_scene->checkCollision(req, res, state);
#if defined(SAVE_RESULT)
    results.push_back(res.collision);
#endif
  }

  auto end = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double, std::milli> elapsed = end - start;
  std::cout << "Time for trajectory collision detection: " << elapsed.count() << " ms" << std::endl;

#if defined(SAVE_RESULT)
    std::ofstream file(ros::package::getPath("rtcc_benchmark") + "/result/bullet_static_results.txt");
    for (auto &result : results)
    {
        file << result << std::endl;
    }
    file.close();
    std::cout << "Result Saved" << std::endl;
#endif
  // Start Trajectory Collision Detection
  start = std::chrono::high_resolution_clock::now();
  for (auto &statePair : statePairs)
  {
    collision_detection::CollisionResult c_res;
    planning_scene->getCollisionEnv()->checkRobotCollision(req, c_res, statePair.first, statePair.second);
  }
  end = std::chrono::high_resolution_clock::now();
  elapsed = end - start;
  std::cout << "Time for trajectory collision detection: " << elapsed.count() << " ms" << std::endl;
}