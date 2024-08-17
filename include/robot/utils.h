#pragma once
#include <array>
#include <fstream>
#include <ros/package.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/planning_scene/planning_scene.h>

using moveit::core::RobotState;

namespace BenchmarkRobot{
    template<int DOF>
    static void poseToState(const std::array<double, DOF> &pose, std::array<const char*, DOF> jointNames, RobotState &state)
    {
        for (int i = 0; i < DOF; i++)
        {
            state.setJointPositions(jointNames[i], &pose[i]);
        }
        state.update();
    }

    template <int DOF>
    struct poseToRobotState{
        RobotState templateState;
        std::array<const char*, DOF> jointNames;

        // poseToRobotState(const RobotState& ts, ) : templateState(ts) {}
        poseToRobotState(const std::array<const char*, DOF> names, const RobotState& ts) : templateState(ts), jointNames(names) {}

        RobotState operator()(const std::array<double, DOF>& pose){
            RobotState state(templateState);
            poseToState<DOF>(pose, jointNames, state);
            return state;
        }
    };

    template <int DOF>
    void loadPosesFromFile(std::string& filePath, std::vector<std::array<double, DOF>> &poses){
        std::ifstream file(filePath, std::ios::binary);
        if (file.is_open())
        {
            while (true)
            {
                std::array<float, DOF> posef;
                std::array<double, DOF> pose;
                file.read(reinterpret_cast<char *>(posef.data()), sizeof(float) * DOF);
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
    }


    template <int DOF>
    void loadStatesFromFile(std::string& filePath, std::vector<RobotState> &states, std::array<const char*, DOF> jointNames, RobotState &templateState){
        std::vector<std::array<double, DOF>> poses;
        loadPosesFromFile<DOF>(filePath, poses);
        states.clear();
        states.reserve(poses.size()); // Reserve space to avoid reallocations
        poseToRobotState<DOF> transformer(jointNames, templateState);
        std::transform(poses.begin(), poses.end(), std::back_inserter(states), transformer);
    }

    template <int DOF>
    void loadTrajFromFile(std::string& filePath, std::vector<std::pair<RobotState, RobotState>> &statePairs, std::array<const char*, DOF> jointNames, RobotState &templateState){
        std::vector<std::array<double, DOF>> poses;
        loadPosesFromFile<DOF>(filePath, poses);

        size_t trajSize = poses.size() / 2;
        statePairs.clear();
        statePairs.reserve(trajSize); // Reserve space to avoid reallocations
        poseToRobotState<DOF> transformer(jointNames, templateState);

        for (int i = 0; i < trajSize; i++)
        {
            RobotState startState = transformer(poses[i]);
            RobotState endState = transformer(poses[i + trajSize]);
            statePairs.emplace_back(std::make_pair(std::move(startState), std::move(endState)));
        }
    }
}