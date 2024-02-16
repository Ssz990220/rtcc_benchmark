#pragma once
#include "utils.h"

using moveit::core::RobotState;
using namespace BenchmarkRobot;

constexpr int DOF = 6;
constexpr char moveGroupName[] = "arm";
constexpr char baseName[] = "base";
constexpr std::array<const char*, DOF> jointNames = {"jaka_joint_1", "jaka_joint_2", "jaka_joint_3", "jaka_joint_4", "jaka_joint_5", "jaka_joint_6"};
namespace BenchmarkRobot{
    void setState(RobotState &state, const std::array<double, DOF> &pose)
    {
        for (int i = 0; i < DOF; i++)
        {
            state.setJointPositions(jointNames[i], &pose[i]);
        }
        state.update();
    }


    void loadPoses(std::vector<RobotState> &states, std::vector<std::pair<RobotState, RobotState>> &statePairs, RobotState &templateState)
    {
        std::string path = ros::package::getPath("rtcc_benchmark");
        std::string dataDir = path + "/data/poses/jaka/";
        std::string posesFile = dataDir + "poses.bin";
        std::string trajFile = dataDir + "trajPoses.bin";

        loadStatesFromFile<DOF>(posesFile, states, jointNames, templateState);

        loadTrajFromFile<DOF>(trajFile, statePairs, jointNames, templateState);
    }

    void validState(std::shared_ptr<planning_scene::PlanningScene> &planning_scene, moveit::core::RobotState &state, collision_detection::CollisionRequest& c_req)
    {
        collision_detection::CollisionResult c_res;
        
        planning_scene->checkCollision(c_req, c_res, state);

        if (c_res.collision)
        {
            ROS_INFO_STREAM("In Collision");
        }
    }

    void validState(std::shared_ptr<planning_scene::PlanningScene> &planning_scene, const std::array<double, DOF> &jointValues, moveit::core::RobotState &state, collision_detection::CollisionRequest& c_req)
    {
        setState(state, jointValues);
        state.update();

        validState(planning_scene, state, c_req);
    }

}