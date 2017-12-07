#include <iostream>
#include <stdio.h>

#include <moveit/move_group_interface/move_group_interface.h>

#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/ApplyPlanningScene.h>
#include <moveit_msgs/Grasp.h>

#include <manipulation_msgs/GraspPlanning.h>

class PickAndPlaceTest{

protected:
    ros::NodeHandle node_handle;
    ros::ServiceClient planning_scene_diff_client;
    ros::ServiceClient grasp_planning_service;

public:
    moveit::planning_interface::MoveGroupInterface arm;
    PickAndPlaceTest() :
        arm("arm")
    {
        planning_scene_diff_client = node_handle.serviceClient<moveit_msgs::ApplyPlanningScene>("apply_planning_scene");
        planning_scene_diff_client.waitForExistence();
    }

    ~PickAndPlaceTest(){
    }

    bool executePick(){
        arm.setPlanningTime(20.0);
        arm.setSupportSurfaceName("table");
        return bool(arm.planGraspsAndPick("object"));
    }

    bool executePick(moveit_msgs::CollisionObject &object){
        arm.setPlanningTime(20.0);
        arm.setSupportSurfaceName("table");
        return bool(arm.planGraspsAndPick(object));
    }

    void executePlace(){

        moveit::planning_interface::MoveGroupInterface gripper("gripper");

        std::vector<moveit_msgs::PlaceLocation> location;

        moveit_msgs::PlaceLocation place_location;
        place_location.id = "place_location";

        jointValuesToJointTrajectory(gripper.getNamedTargetValues("open"), place_location.post_place_posture);

        geometry_msgs::PoseStamped pose;
        pose.header.frame_id = "table_top";
        pose.pose.orientation.w = 1;
        pose.pose.position.x = -0.08;
        pose.pose.position.y = 0.2;
        pose.pose.position.z = 0.1;
        place_location.place_pose = pose;

        place_location.pre_place_approach.min_distance = 0.02;
        place_location.pre_place_approach.desired_distance = 0.1;
        place_location.pre_place_approach.direction.header.frame_id = "s_model_tool0";
        place_location.pre_place_approach.direction.vector.x = 1.0;

        place_location.post_place_retreat.min_distance = 0.02;
        place_location.post_place_retreat.desired_distance = 0.1;
        place_location.post_place_retreat.direction.header.frame_id = "table_top";
        place_location.post_place_retreat.direction.vector.z = 1.0;

        location.push_back(place_location);

        arm.setSupportSurfaceName("table");
        arm.place("object", location);
    }

    moveit_msgs::CollisionObject spawnObject(){
        moveit_msgs::ApplyPlanningScene srv;
        moveit_msgs::PlanningScene planning_scene;
        planning_scene.is_diff = true;
        planning_scene.robot_state.is_diff = true;

        moveit_msgs::CollisionObject object;

        object.header.frame_id = "table_top";
        object.id = "object";

        shape_msgs::SolidPrimitive primitive;
        primitive.type = primitive.CYLINDER;
        primitive.dimensions.push_back(0.2);
        primitive.dimensions.push_back(0.04);

        geometry_msgs::Pose pose;
        pose.orientation.w = 1;
        pose.position.x = -0.03;
        pose.position.y = 0.2;
        pose.position.z = primitive.dimensions[0]/2 + 0.0;

        object.primitives.push_back(primitive);
        object.primitive_poses.push_back(pose);

        // add object to scene
        object.operation = object.ADD;
        planning_scene.world.collision_objects.push_back(object);

        // remove attached object in case it is attached
        moveit_msgs::AttachedCollisionObject aco;
        object.operation = object.REMOVE;
        aco.object = object;
        planning_scene.robot_state.attached_collision_objects.push_back(aco);

        srv.request.scene = planning_scene;
        planning_scene_diff_client.call(srv);

        return object;
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

    ROS_INFO("Starting Test");
    PickAndPlaceTest testClass;
    ROS_INFO("Spawn Object");

    moveit_msgs::CollisionObject object = testClass.spawnObject();

    while(ros::ok())
    {
      ROS_INFO("Pick Object");
      testClass.executePick();
      //testClass.executePick(object);

      ROS_INFO("Place Object");
      testClass.executePlace();

      ROS_INFO("Move to home");
      testClass.arm.setNamedTarget("home");
      testClass.arm.move();
    }
    return 0;
}
