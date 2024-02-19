#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/collision_detection_fcl/collision_detector_allocator_fcl.h>
#include <moveit/robot_state/conversions.h>
#include <geometric_shapes/mesh_operations.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include "interactive_robot.h"

#include "benchmarkCommon.h"
#include "robot/panda.h"
#include "scene/scene_shelf.h"
// auto g_planning_scene = std::unique_ptr<planning_scene::PlanningScene>();

// Temp
auto g_marker_array_publisher = std::unique_ptr<ros::Publisher>();
auto g_col_marker_publisher = std::unique_ptr<ros::Publisher>();
shapes::ShapePtr g_world_cube_shape;
visualization_msgs::MarkerArray g_collision_points;
visualization_msgs::MarkerArray g_collision_objs;
static int colObjCount = 0;
const double BOX_SIZE = 0.1;


int main(int argc, char **argv)
{
  ros::init(argc, argv, "benchmarkFCLViz");
  ros::NodeHandle node_handle;

  ros::AsyncSpinner spinner(1);
  spinner.start();

  robot_model_loader::RobotModelLoader rm_loader_("robot_description");

  robot_model::RobotModelPtr robot_model = rm_loader_.getModel();
  ros::Publisher robot_state_publisher_(node_handle.advertise<moveit_msgs::DisplayRobotState>("benchmarkBulletViz/interactive_robot_state", 1));
  auto planning_scene = std::make_shared<planning_scene::PlanningScene>(robot_model);
  planning_scene->setActiveCollisionDetector(collision_detection::CollisionDetectorAllocatorFCL::create(),
                                             /* exclusive = */ true);
  // Temp
  moveit_visual_tools::MoveItVisualTools visual_tools("base");
  ros::Publisher marker_publisher(node_handle.advertise<visualization_msgs::Marker>("interactive_robot_markers", 1));
  visual_tools.setRobotStateTopic("interactive_robot_state");

  g_marker_array_publisher = std::make_unique<ros::Publisher>(
      node_handle.advertise<visualization_msgs::MarkerArray>("interactive_robot_marray", 100));

  g_col_marker_publisher = std::make_unique<ros::Publisher>(
      node_handle.advertise<visualization_msgs::MarkerArray>("col_scene_marray", 100));

  visual_tools.prompt(
      "Press 'next' in the RvizVisualToolsGui window to start the continuous collision detection demo.");
  ROS_INFO("Shutting down the interactive interactive_robot...");

  setCollisionScene(planning_scene, g_collision_objs, g_col_marker_publisher);

  moveit::core::RobotState state = planning_scene->getCurrentStateNonConst();
  state.setToDefaultValues();

  // Load Poses
  std::vector<moveit::core::RobotState> states;
  std::vector<std::pair<moveit::core::RobotState, moveit::core::RobotState>> statePairs;
  BenchmarkRobot::loadPoses(states, statePairs, state);
  ROS_INFO_STREAM("Loaded " << states.size() << " Poses from file");

  visual_tools.prompt(
      "Press 'next' in the RvizVisualToolsGui window to start the benchmark on static poses validation...");

  // Time the collision detection
  std::vector<collision_detection::CollisionRequest> c_req(states.size());
  std::vector<collision_detection::CollisionResult> c_res(states.size());
  std::vector<int> result(states.size());
  auto start = std::chrono::high_resolution_clock::now();
//   for (auto const &state : states)
  for (int i = 0; i < states.size(); ++i)
  {
    // computeCollisionContactPoints(planning_scene, states[i]);
    // visual_tools.publishRobotState(states[i]);
    planning_scene->checkCollision(c_req[i], c_res[i], states[i]);
    // visual_tools.publishRobotState(states[i]);
    if (c_res[i].collision){
        // std::cout << "In Collision" << std::endl;
        result[i] = 1;
    }
    // visual_tools.prompt(
    //     "Press 'next' for next pose");
  }

  auto end = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double, std::milli> elapsed = end - start;
  std::cout << "Time for static poses collision detection: " << elapsed.count() << " ms" << std::endl;
// Print the first 10 elemtns in result
    for (int i = 0; i < 100; ++i){
        std::cout << result[i] << " ";
    }
    std::cout << std::endl;

  visual_tools.prompt(
      "Press 'next' in the RvizVisualToolsGui window to start the benchmark on trajectory validation...");
  ROS_INFO_STREAM("Loaded " << statePairs.size() << " Trajs from file");

  std::string path = ros::package::getPath("rtcc_benchmark");
  std::string dataDir = path + "/data/poses/panda/";
  {
    std::string disTraj4 = dataDir + "pandasDisTraj4.bin";

    loadStatesFromFile<DOF>(disTraj4, states, jointNames, state);

    std::cout << "Loaded " << states.size() << " states from file" << std::endl;
    c_req.resize(states.size());
    c_res.resize(states.size());

    start = std::chrono::high_resolution_clock::now();
    for (int i = 0; i < states.size(); i++)
    {
      planning_scene->checkCollision(c_req[i], c_res[i], states[i]);
    }
    end = std::chrono::high_resolution_clock::now();
    elapsed = end - start;
    std::cout << "Time for trajectory collision detection -- 4 discretized poses: " << elapsed.count() << " ms" << std::endl;
  }
  {
    std::string disTraj8 = dataDir + "pandasDisTraj8.bin";

    loadStatesFromFile<DOF>(disTraj8, states, jointNames, state);

    std::cout << "Loaded " << states.size() << " states from file" << std::endl;
    c_req.resize(states.size());
    c_res.resize(states.size());

    start = std::chrono::high_resolution_clock::now();
    for (int i = 0; i < states.size(); i++)
    {
      planning_scene->checkCollision(c_req[i], c_res[i], states[i]);
    }
    end = std::chrono::high_resolution_clock::now();
    elapsed = end - start;
    std::cout << "Time for trajectory collision detection -- 8 discretized poses: " << elapsed.count() << " ms" << std::endl;
  }

  {
    std::string disTraj16 = dataDir + "pandasDisTraj16.bin";

    loadStatesFromFile<DOF>(disTraj16, states, jointNames, state);

    std::cout << "Loaded " << states.size() << " states from file" << std::endl;
    c_req.resize(states.size());
    c_res.resize(states.size());

    start = std::chrono::high_resolution_clock::now();
    for (int i = 0; i < states.size(); i++)
    {
      planning_scene->checkCollision(c_req[i], c_res[i], states[i]);
    }
    end = std::chrono::high_resolution_clock::now();
    elapsed = end - start;
    std::cout << "Time for trajectory collision detection -- 16 discretized poses: " << elapsed.count() << " ms" << std::endl;
  }

  moveit_msgs::DisplayRobotState msg;
  robot_state::robotStateToRobotStateMsg(state, msg.state);
  visual_tools.publishRobotState(state);

  for (auto& s : states){

    computeCollisionContactPoints(planning_scene, s, g_marker_array_publisher, g_collision_points);
    visual_tools.publishRobotState(s);
    visual_tools.prompt(
        "Press 'next' for next pose");
  }

  visual_tools.prompt(
      "Press 'next' in the RvizVisualToolsGui window to start the continuous collision detection demo.");
  ROS_INFO("Shutting down the interactive interactive_robot...");

    // remove all collision markers
  if (!g_collision_points.markers.empty())
  {
    for (auto &marker : g_collision_points.markers)
      marker.action = visualization_msgs::Marker::DELETE;

    g_marker_array_publisher->publish(g_collision_points);
  }

  visual_tools.deleteAllMarkers();
}