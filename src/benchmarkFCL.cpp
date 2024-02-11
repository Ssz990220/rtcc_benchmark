#include <moveit/utils/robot_model_test_utils.h>
#include <geometric_shapes/mesh_operations.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_state/conversions.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/collision_detection_fcl/collision_detector_allocator_fcl.h>
#include "benchmarkCommon.h"
#include "scene/scene_manufacuting.h"

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