#pragma once

#include <array>
#include <vector>
#include <chrono>
#include <ros/package.h>

#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/collision_detection/collision_common.h>
#include <moveit/collision_detection/collision_env.h>
#include <moveit/collision_detection/collision_detector_allocator.h>
#include <moveit/collision_detection/collision_tools.h>

using moveit::core::RobotState;

static void poseToState(const std::array<double, 6> &pose, RobotState &state)
{
    state.setJointPositions("jaka_joint_1", &pose[0]);
    state.setJointPositions("jaka_joint_2", &pose[1]);
    state.setJointPositions("jaka_joint_3", &pose[2]);
    state.setJointPositions("jaka_joint_4", &pose[3]);
    state.setJointPositions("jaka_joint_5", &pose[4]);
    state.setJointPositions("jaka_joint_6", &pose[5]);
    state.update();
}

void loadPoses(std::vector<RobotState> &states, std::vector<std::pair<RobotState, RobotState>> &statePairs, RobotState &templateState)
{
    std::vector<std::array<double, 6>> poses;
    // Read poses from bin file as vector of arrays
    std::string path = ros::package::getPath("rtcc_benchmark");
    std::string dataDir = path + "/data/";

    std::ifstream file(dataDir + "poses.bin", std::ios::binary);
    if (file.is_open())
    {
        while (true)
        {
            std::array<float, 6> posef;
            std::array<double, 6> pose;
            file.read(reinterpret_cast<char *>(posef.data()), sizeof(float) * 6);
            if (file.eof())
            {
                break;
            }

            std::transform(posef.begin(), posef.end(), pose.begin(),
                           [](float f)
                           { return static_cast<double>(f); });
            poses.push_back(pose);
        }
    }
    file.close();
    states.clear();
    states.reserve(poses.size()); // Reserve space to avoid reallocations

    // Define the lambda function outside std::transform
    auto poseToRobotState = [&templateState](const std::array<double, 6> &pose) -> RobotState
    {
        RobotState state(templateState);
        poseToState(pose, state);
        return state;
    };

    std::transform(poses.begin(), poses.end(), std::back_inserter(states), poseToRobotState);

    std::vector<std::array<double, 6>> trajPoses;
    file.open(dataDir + "trajPoses.bin", std::ios::binary);
    if (file.is_open())
    {
        while (true)
        {
            std::array<float, 6> posef;
            std::array<double, 6> pose;
            file.read(reinterpret_cast<char *>(pose.data()), sizeof(float) * 6);
            if (file.eof())
            {
                break;
            }
            std::transform(posef.begin(), posef.end(), pose.begin(),
                           [](float f)
                           { return static_cast<double>(f); });
            trajPoses.push_back(pose);
        }
    }
    file.close();

    size_t trajSize = trajPoses.size() / 2;
    statePairs.clear();
    statePairs.reserve(trajSize); // Reserve space to avoid reallocations

    for (int i = 0; i < trajSize; i++)
    {
        RobotState startState = poseToRobotState(trajPoses[i]);
        RobotState endState = poseToRobotState(trajPoses[i + trajSize]);
        statePairs.emplace_back(std::make_pair(std::move(startState), std::move(endState)));
    }
}