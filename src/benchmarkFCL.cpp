#include "benchmarkCommon.h"
#include "robot/jaka.h"
#include "scene/scene_manufacuting.h"
#include <moveit/collision_detection_fcl/collision_detector_allocator_fcl.h>
#define SAVE_RESULT

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



int main(int argc, char **argv)
{
    ros::init(argc, argv, "benchmarkFCL");
    std::shared_ptr<collision_detection::CollisionDetectorAllocatorFCL> value_ = std::make_shared<collision_detection::CollisionDetectorAllocatorFCL>();
    robot_model_loader::RobotModelLoader rm_loader_("robot_description");
    robot_model::RobotModelPtr robot_model = rm_loader_.getModel();
    auto planning_scene = std::make_shared<planning_scene::PlanningScene>(robot_model);
    moveit::core::RobotState robot_state_ = planning_scene->getCurrentStateNonConst();
    // setToHome(*robot_state_);

    setCollisionScene(planning_scene);

    std::vector<moveit::core::RobotState> states;
    std::vector<std::pair<moveit::core::RobotState, moveit::core::RobotState>> statePairs;

    BenchmarkRobot::loadPoses(states, statePairs, robot_state_);
    std::cout << "Load " << states.size() << " poses from file" << std::endl;

    std::cout << "Found " << planning_scene->getWorld()->getObjectIds().size() << " objects in the planning scene" << std::endl;
    auto world = planning_scene->getWorld()->getObjectIds();
    for (auto it = world.begin(); it != world.end(); it++)
    {
        std::cout << "Object " << *it << std::endl;
        auto shape = planning_scene->getWorld()->getObject(*it)->shapes_.front();
        std::cout << "Shape " << shape->type << std::endl;
        shape->print(std::cout);
    }

#if defined(SAVE_RESULT)
    std::vector<int> results(states.size());
    std::vector<int> resultsTraj(statePairs.size());
#endif
    // Start Collision Detection
    collision_detection::CollisionRequest req;
    req.group_name = "arm";
    collision_detection::CollisionResult res;
    // Static Poses First
    auto start = std::chrono::high_resolution_clock::now();
    for (auto &state : states)
    {
        planning_scene->checkCollision(req, res, state);
#if defined(SAVE_RESULT)
        results.push_back((int)res.collision);
#endif
    }

    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> elapsed = end - start;
    std::cout << "Time for trajectory collision detection: " << elapsed.count() << " ms" << std::endl;

#if defined(SAVE_RESULT)
    std::ofstream file(ros::package::getPath("rtcc_benchmark") + "/result/fcl_static_results.txt");
    for (auto &result : results)
    {
        file << result << std::endl;
    }
    file.close();
    std::cout << "Result Saved" << std::endl;
#endif
}