#include <ros/ros.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

int main(int argc, char** argv){
    ros::init(argc, argv, "ArmTeleopLogic");
    ros::AsyncSpinner spin(1);
    spin.start();
    ros::NodeHandle np("~");

    robot_model_loader::RobotModelLoader modelLoader("robot_description");
    const robot_model::RobotModelPtr model = modelLoader.getModel();
    moveit::core::RobotStatePtr armState(new moveit::core::RobotState(model));
    const moveit::core::JointModelGroup* joint_model_group = armState->getJointModelGroup("arm");

    planning_scene::PlanningScenePtr scene(new planning_scene::PlanningScene(model));
    scene->getCurrentStateNonConst().setToDefaultValues(joint_model_group, "ready");

    planning_interface::MotionPlanRequest req;
    planning_interface::MotionPlanResponse res;
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = "link1";
    pose.pose.position.x = 3;
    pose.pose.position.y = 2;
    pose.pose.position.z = 1;
    pose.pose.orientation.w = 0;
    moveit_msgs::Constraints poseGoal = kinematic_constraints::constructGoalConstraints("link6", pose);
    req.goal_constraints.clear();
    req.goal_constraints.push_back(poseGoal);

    planning_interface::PlannerManagerPtr planner;

    planning_interface::PlanningContextPtr context = planner->getPlanningContext(scene, req, res.error_code_);
    context->solve(res);
    if(res.error_code_.val != res.error_code_.SUCCESS){
        ROS_ERROR("Could not compute plan successfully");
        return 0;

    }

    ros::Publisher display_publisher = np.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
    moveit_msgs::DisplayTrajectory display_trajectory;
    moveit_msgs::MotionPlanResponse response;
    res.getMessage(response);
    display_trajectory.trajectory_start = response.trajectory_start;
    display_trajectory.trajectory.push_back(response.trajectory);
    display_publisher.publish(display_trajectory);

    moveit_visual_tools::MoveItVisualTools vTools("link1");
    vTools.deleteAllMarkers();
    vTools.publishTrajectoryLine(display_trajectory.trajectory.back(), joint_model_group);
    vTools.trigger();


    return 0;
}