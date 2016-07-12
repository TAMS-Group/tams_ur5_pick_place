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
        spawnObject();

        moveit_msgs::Grasp grasp;
        grasp.id = "grasp";

        grasp.pre_grasp_posture.joint_names.resize(1, "s_model_finger_1_link_0");
        grasp.pre_grasp_posture.points.resize(1);
        grasp.pre_grasp_posture.points[0].positions.resize(1);
        grasp.pre_grasp_posture.points[0].positions[0] = 1;

        grasp.grasp_posture.joint_names.resize(1, "s_model_finger_1_link_0");
        grasp.grasp_posture.points.resize(1);
        grasp.grasp_posture.points[0].positions.resize(1);
        grasp.grasp_posture.points[0].positions[0] = -1;

        
        geometry_msgs::PoseStamped pose;
        pose.header.frame_id = "object";
        pose.pose.position.x = 0.0;
        pose.pose.position.y = 0.0;
        pose.pose.position.z = 0.1;
        pose.pose.orientation.w = 1.0;
        grasp.grasp_pose = pose;

        grasp.pre_grasp_approach.min_distance = 0.2;
        grasp.pre_grasp_approach.desired_distance = 0.4;
        grasp.pre_grasp_approach.direction.header.frame_id = "ft_fts_toolside";
        grasp.pre_grasp_approach.direction.vector.z = 1.0;

        grasp.post_grasp_retreat.min_distance = 0.2;
        grasp.post_grasp_retreat.desired_distance = 0.4;
        grasp.post_grasp_retreat.direction.header.frame_id = "ft_fts_toolside";
        grasp.post_grasp_retreat.direction.vector.z = -1.0;


        arm.pick("object", grasp);
    }

    void spawnObject(){
        moveit_msgs::ApplyPlanningScene srv;
        moveit_msgs::PlanningScene planning_scene;
        planning_scene.is_diff = true;

        moveit_msgs::CollisionObject object;

        object.header.frame_id = "/table_top";
        object.id = "object";

        shape_msgs::SolidPrimitive primitive;
        primitive.type = primitive.BOX;
        primitive.dimensions.resize(3);
        primitive.dimensions[0] = 0.05;
        primitive.dimensions[1] = 0.05;
        primitive.dimensions[2] = 0.20;

        geometry_msgs::Pose pose;
        pose.orientation.w = 1;
        pose.position.x = -0.3;
        pose.position.y = 0.2;
        pose.position.z = primitive.dimensions[2]/2;

        object.primitives.push_back(primitive);
        object.primitive_poses.push_back(pose);
        object.operation = object.ADD;
        planning_scene.world.collision_objects.push_back(object);
        
        srv.request.scene = planning_scene;
        planning_scene_diff_client.call(srv);
    }
};

int main(int argc, char** argv){
    ros::init(argc, argv, "PaPTest");
    PickAndPlaceTest testClass;
    testClass.executePick();
    ros::spin();
    return 0;
}
