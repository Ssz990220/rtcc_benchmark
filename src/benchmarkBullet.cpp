#include <vector>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/collision_detection_bullet/collision_env_bullet.h>
#include <moveit/collision_detection_bullet/collision_detector_allocator_bullet.h>
#include <moveit/collision_detection/collision_tools.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene/planning_scene.h>

auto g_planning_scene = std::unique_ptr<planning_scene::PlanningScene>();

int main(int argc, char** argv){
    ros::init(argc, argv, "benchmarkBullet");
    ros::NodeHandle node_handle;

    ros::AsyncSpinner spinner(1);
    spinner.start();
    


    robot_model_loader::RobotModelLoader rm_loader_("robot_description");
    moveit::core::RobotModelPtr robot_model_ = rm_loader_.getModel();
    if (!robot_model_)
    {
        ROS_ERROR("Could not load robot description");
    }
    moveit::core::RobotStatePtr robot_state_;
    
    g_planning_scene = std::make_unique<planning_scene::PlanningScene>(robot_model_);
    g_planning_scene->setActiveCollisionDetector(collision_detection::CollisionDetectorAllocatorBullet::create(),
                                                 true /* exclusive */);
}