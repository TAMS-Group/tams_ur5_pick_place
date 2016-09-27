#include <ros/ros.h>
#include <moveit_msgs/PlanGrasps.h>

#include <moveit/move_group_interface/move_group.h>

void jointValuesToJointTrajectory(std::map<std::string, double> target_values, ros::Duration duration, 
        trajectory_msgs::JointTrajectory &grasp_pose)
{
  grasp_pose.joint_names.reserve(target_values.size());
  grasp_pose.points.resize(1);
  grasp_pose.points[0].positions.reserve(target_values.size());

  for(std::map<std::string, double>::iterator it = target_values.begin(); it != target_values.end(); ++it){
    grasp_pose.joint_names.push_back(it->first);
    grasp_pose.points[0].positions.push_back(it->second);
  }
  grasp_pose.points[0].time_from_start = duration;
}

bool serviceCB(moveit_msgs::PlanGrasps::Request &req, moveit_msgs::PlanGrasps::Response &res)
{
  moveit::planning_interface::MoveGroup arm(req.arm_name);
  moveit::planning_interface::MoveGroup gripper(arm.getRobotModel()->getEndEffectors()[0]->getName());

  moveit_msgs::Grasp grasp;
  grasp.id = "grasp";

  jointValuesToJointTrajectory(gripper.getNamedTargetValues("open"), ros::Duration(1.0), grasp.pre_grasp_posture);
  jointValuesToJointTrajectory(gripper.getNamedTargetValues("closed"), ros::Duration(2.0), grasp.grasp_posture);

  geometry_msgs::PoseStamped pose;
  pose.header.frame_id = req.target.id;
  pose.pose.orientation.x = 0.5;
  pose.pose.orientation.y = 0.5;
  pose.pose.orientation.z = -0.5;
  pose.pose.orientation.w = 0.5;
  pose.pose.position.z = req.target.primitives[0].dimensions[0]/2;
  grasp.grasp_pose = pose;

  grasp.pre_grasp_approach.min_distance = 0.08;
  grasp.pre_grasp_approach.desired_distance = 0.1;
  grasp.pre_grasp_approach.direction.header.frame_id = "tool0";
  grasp.pre_grasp_approach.direction.vector.x = 1.0;

  grasp.post_grasp_retreat.min_distance = 0.08;
  grasp.post_grasp_retreat.desired_distance = 0.1;
  grasp.post_grasp_retreat.direction.header.frame_id = arm.getPlanningFrame();
  grasp.post_grasp_retreat.direction.vector.z = 1.0;
  res.grasps.push_back(grasp);

  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "plan_grasps_server");
  ros::NodeHandle n;

  ros::ServiceServer service = n.advertiseService("plan_grasps", serviceCB);
  ros::spin();

  return 0;
}

