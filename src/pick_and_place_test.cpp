#include <iostream>
#include <stdio.h>

#include <moveit/move_group_interface/move_group.h>

#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/ApplyPlanningScene.h>
#include <moveit_msgs/Grasp.h>

#include <manipulation_msgs/GraspPlanning.h>

class PickAndPlaceTest{

protected:
    ros::NodeHandle node_handle;
    ros::ServiceClient planning_scene_diff_client;
    ros::ServiceClient grasp_planning_service;

public:
    PickAndPlaceTest(){
        planning_scene_diff_client = node_handle.serviceClient<moveit_msgs::ApplyPlanningScene>("apply_planning_scene");
        planning_scene_diff_client.waitForExistence();
    }

    ~PickAndPlaceTest(){
    }

    void executePick(){

        moveit::planning_interface::MoveGroup arm("arm");
        moveit::planning_interface::MoveGroup gripper("gripper");
        
        moveit_msgs::Grasp grasp;
        grasp.id = "grasp";

        jointValuesToJointTrajectory(gripper.getNamedTargetValues("open"), grasp.pre_grasp_posture);
        jointValuesToJointTrajectory(gripper.getNamedTargetValues("closed"), grasp.grasp_posture);

        geometry_msgs::PoseStamped pose;
        pose.header.frame_id = "object";
        pose.pose.orientation.x = 0.5;
        pose.pose.orientation.y = 0.5;
        pose.pose.orientation.z = -0.5;
        pose.pose.orientation.w = 0.5;
        pose.pose.position.z = 0.1;
        grasp.grasp_pose = pose;

        grasp.pre_grasp_approach.min_distance = 0.02;
        grasp.pre_grasp_approach.desired_distance = 0.1;
        grasp.pre_grasp_approach.direction.header.frame_id = "tool0";
        grasp.pre_grasp_approach.direction.vector.x = 1.0;

        grasp.post_grasp_retreat.min_distance = 0.02;
        grasp.post_grasp_retreat.desired_distance = 0.1;
        grasp.post_grasp_retreat.direction.header.frame_id = "table_top";
        grasp.post_grasp_retreat.direction.vector.z = 1.0;

        arm.setSupportSurfaceName("table");

        arm.pick("object");
    }

    void executePlace(){

        moveit::planning_interface::MoveGroup arm("arm");
        moveit::planning_interface::MoveGroup gripper("gripper");

        std::vector<moveit_msgs::PlaceLocation> location;

        moveit_msgs::PlaceLocation place_location;
        place_location.id = "place_location";

        jointValuesToJointTrajectory(gripper.getNamedTargetValues("open"), place_location.post_place_posture);

        geometry_msgs::PoseStamped pose;
        pose.header.frame_id = "table_top";
        pose.pose.orientation.w = 1;
        pose.pose.position.x = -0.03;
        pose.pose.position.y = 0.2;
        pose.pose.position.z = 0.1;
        place_location.place_pose = pose;

        place_location.pre_place_approach.min_distance = 0.02;
        place_location.pre_place_approach.desired_distance = 0.1;
        place_location.pre_place_approach.direction.header.frame_id = "tool0";
        place_location.pre_place_approach.direction.vector.x = 1.0;

        place_location.post_place_retreat.min_distance = 0.02;
        place_location.post_place_retreat.desired_distance = 0.1;
        place_location.post_place_retreat.direction.header.frame_id = "table_top";
        place_location.post_place_retreat.direction.vector.z = 1.0;

        location.push_back(place_location);

        arm.setSupportSurfaceName("table");
        arm.place("object", location);
    }

    void spawnObject(){
        moveit_msgs::ApplyPlanningScene srv;
        moveit_msgs::PlanningScene planning_scene;
        planning_scene.is_diff = true;

        moveit_msgs::CollisionObject object;

        object.header.frame_id = "table_top";
        object.id = "object";

        shape_msgs::SolidPrimitive primitive;
        primitive.type = primitive.BOX;
        primitive.dimensions.resize(3);
        primitive.dimensions[0] = 0.05;
        primitive.dimensions[1] = 0.05;
        primitive.dimensions[2] = 0.20;

        geometry_msgs::Pose pose;
        pose.orientation.w = 1;
        pose.position.x = -0.03;
        pose.position.y = 0.2;
        pose.position.z = primitive.dimensions[2]/2 + 0.0;

        object.primitives.push_back(primitive);
        object.primitive_poses.push_back(pose);
        object.operation = object.ADD;
        planning_scene.world.collision_objects.push_back(object);
        
        srv.request.scene = planning_scene;
        planning_scene_diff_client.call(srv);
    }

    void jointValuesToJointTrajectory(std::map<std::string, double> target_values, trajectory_msgs::JointTrajectory &grasp_pose){
       grasp_pose.joint_names.reserve(target_values.size());
       grasp_pose.points.resize(1);
       grasp_pose.points[0].positions.reserve(target_values.size());

       for(std::map<std::string, double>::iterator it = target_values.begin(); it != target_values.end(); ++it){
           grasp_pose.joint_names.push_back(it->first);
           grasp_pose.points[0].positions.push_back(it->second);
       }
    }

    void testPose(){

        moveit::planning_interface::MoveGroup arm("arm");

        geometry_msgs::PoseStamped pose;

        pose.header.frame_id = "table_top";
        pose.pose.orientation.x = 0.5;
        pose.pose.orientation.y = 0.5;
        pose.pose.orientation.z = -0.5;
        pose.pose.orientation.w = 0.5;
        pose.pose.position.x = 0.05;
        pose.pose.position.y = 0.2;
        pose.pose.position.z = 0.344;

        arm.setPoseTarget(pose, "tool0");
        arm.move();
    }

};

int main(int argc, char** argv){
    ros::init(argc, argv, "PaPTest");
    ros::AsyncSpinner spinner(1);
    spinner.start();

    PickAndPlaceTest testClass;
    testClass.spawnObject();
    testClass.executePick();
    ROS_ERROR("PLACE");
    ros::WallDuration(1.0).sleep();
    testClass.executePlace();
    //testClass.testPose();
    return 0;
}
