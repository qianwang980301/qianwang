#include <ros/ros.h>
#include <grabsth/arm_motion_lib.h>
#include <math.h>
#include <string>
#include <gazebo_msgs/GetJointProperties.h>
#include <gazebo_msgs/GetModelState.h>
#include <geometry_msgs/Point.h>
#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <Eigen/Geometry>

   

int main(int argc, char** argv) {
    ros::init(argc, argv, "grabsth_usage");
    ros::NodeHandle nh;
    ArmMotionCommander arm_motion_commander(&nh);
    
    ros::ServiceClient get_model_state_client = nh.serviceClient<gazebo_msgs::GetModelState>(
        "/gazebo/get_model_state");
    gazebo_msgs::GetModelState get_model_state_srv_msg;

    // parameters for flow control, time assignment
    double lift_height = 0.2; // height to lift the beer up
    double beer_height = 0.23;// 0.23 is real height, use 0.30 to lift the grasp point up a little
    double table_height = 1.0; // the height of the table
    double torso_height = 0.832;
    double time_delay = 1.5; 
    //double gripper_open = 0.24; // distance of two paddles when gripper is open
    //double gripper_close = 0.08; // distance of two paddles when grasping the beer
    int rtn_val;

    // eigen stuff
    Eigen::Vector3d xvec_des,yvec_des,zvec_des,origin_des;
    Eigen::Affine3d Affine_des_gripper;
    Eigen::Matrix3d Rmat;

    // rotation
    xvec_des << -1,0,0;
    yvec_des << 0,1,0;
    zvec_des << 0,0,-1;
    Rmat.col(0) = xvec_des;
    Rmat.col(1) = yvec_des;
    Rmat.col(2) = zvec_des;
    Affine_des_gripper.linear() = Rmat;
    // translation
    origin_des << 0.4, 0.3, 0.1;
    Affine_des_gripper.translation() = origin_des;

    rtn_val=arm_motion_commander.plan_move_to_pre_pose();
    rtn_val=arm_motion_commander.rt_arm_execute_planned_path();

    ///////////////////////////////////////
    // 1.move the gripper to the safe point
    ///////////////////////////////////////
    geometry_msgs::PoseStamped rt_tool_pose;
    // rt_tool_pose.pose.position.x=0.4;
    // rt_tool_pose.pose.position.y=0.3;
    // rt_tool_pose.pose.position.z=0.1;
    // rtn_val = arm_motion_commander.rt_arm_plan_path_current_to_goal_pose(rt_tool_pose);
    // rtn_val = arm_motion_commander.rt_arm_execute_planned_path();
    ROS_INFO("task 1: move the gripper to the top area of the beer.");

    
    // get the position of the beer from gazebo
    // the position from gazebo is the bottom center of the beer
    // geometry_msgs::PoseStamped beer_pos; // {float64 x, float64 y, float z}
    geometry_msgs::Point beer_pos;
    get_model_state_srv_msg.request.model_name = "beer";
    get_model_state_srv_msg.request.relative_entity_name = "link";
    // "link" is the entity name when I add a beer in gazebo
    get_model_state_client.call(get_model_state_srv_msg);
    beer_pos = get_model_state_srv_msg.response.pose.position;
    ROS_INFO_STREAM("beer, x: " << beer_pos.x <<
        "; y: " << beer_pos.y <<
        "; z: " << beer_pos.z);    

    geometry_msgs::PoseStamped hover_open_start_pos;
    hover_open_start_pos.pose = arm_motion_commander.transformEigenAffine3dToPose(Affine_des_gripper);
    hover_open_start_pos.pose.position = beer_pos;
    hover_open_start_pos.pose.position.z = hover_open_start_pos.pose.position.z + beer_height + lift_height - torso_height;
    hover_open_start_pos.pose.position.x = hover_open_start_pos.pose.position.x;
    hover_open_start_pos.pose.position.y = hover_open_start_pos.pose.position.y;

    ROS_INFO_STREAM("position, x: " << hover_open_start_pos.pose.position.x <<
        "; y: " << hover_open_start_pos.pose.position.y <<
        "; z: " << hover_open_start_pos.pose.position.z);
    ROS_INFO_STREAM("orientation, x: " << hover_open_start_pos.pose.orientation.x <<
        "; y: " << hover_open_start_pos.pose.orientation.y <<
        "; z: " << hover_open_start_pos.pose.orientation.z <<
        "; w: " << hover_open_start_pos.pose.orientation.w);
    rtn_val = arm_motion_commander.rt_arm_plan_path_current_to_goal_pose(hover_open_start_pos);
    rtn_val = arm_motion_commander.rt_arm_execute_planned_path();
    ros::Duration(time_delay).sleep();

    ROS_INFO("task 2: move the gripper around the beer.");
    // calculate robot joints when around the beer
    geometry_msgs::PoseStamped around_open_start_pos;
    around_open_start_pos.pose = arm_motion_commander.transformEigenAffine3dToPose(Affine_des_gripper);
    around_open_start_pos.pose.position = beer_pos;
    around_open_start_pos.pose.position.z = around_open_start_pos.pose.position.z + beer_height - torso_height-0.05;
    rtn_val = arm_motion_commander.rt_arm_plan_path_current_to_goal_pose(around_open_start_pos);
    rtn_val = arm_motion_commander.rt_arm_execute_planned_path();
    ros::Duration(time_delay).sleep();
    ROS_INFO("task 4: clamp the gripper to grasp the beer.");
    arm_motion_commander.gripperclose();
    ros::Duration(time_delay).sleep();
    //arm_motion_commander.gripper_close();
    // calculate robot joints when grasp the beer
    //geometry_msgs::PoseStamped grasp_close_start_pos;
    //grasp_close_start_pos = around_open_start_pos;
    //std::vector<double> grasp_close_start_jnts; // the corresponding joint position
    //grasp_close_start_jnts.resize(6);
    //grasp_close_start_jnts = inverseKinematics(grasp_close_start_pos, gripper_close);

    

    
    ROS_INFO("task 5: move the gripper up with the beer.");

    // calculate the robot joint when grasp the beer and lift up
    geometry_msgs::PoseStamped hover_close_start_pos;
    hover_close_start_pos.pose = arm_motion_commander.transformEigenAffine3dToPose(Affine_des_gripper);
    hover_close_start_pos = around_open_start_pos;
    hover_close_start_pos.pose.position.z = hover_close_start_pos.pose.position.z + lift_height;
    rtn_val = arm_motion_commander.rt_arm_plan_path_current_to_goal_pose(hover_close_start_pos);
    rtn_val = arm_motion_commander.rt_arm_execute_planned_path();
    ros::Duration(time_delay).sleep();

    ROS_INFO("task 6: move the gripper to the above of target area.");

    // get the position of the table from gazebo
    geometry_msgs::Point table_pos;
    get_model_state_srv_msg.request.model_name = "cafe_table";
    get_model_state_srv_msg.request.relative_entity_name = "link";
    get_model_state_client.call(get_model_state_srv_msg);
    table_pos = get_model_state_srv_msg.response.pose.position;

    // calculate the target position of the beer
    geometry_msgs::PoseStamped target_pos;
    target_pos.pose.position = table_pos;
    ROS_INFO_STREAM("table_pose: " << table_pos.x << ", " << table_pos.y << ", " << table_pos.z);
    target_pos.pose.position.z = target_pos.pose.position.z + table_height;

    // calculate the robot joints at the above of target area
    geometry_msgs::PoseStamped hover_close_end_pos;
    hover_close_end_pos.pose = arm_motion_commander.transformEigenAffine3dToPose(Affine_des_gripper);
    hover_close_end_pos = target_pos;
    hover_close_end_pos.pose.position.z = hover_close_end_pos.pose.position.z + beer_height + lift_height - torso_height;
    hover_close_end_pos.pose.position.x = hover_close_end_pos.pose.position.x ;
    ROS_INFO_STREAM("hover_close_end_pos: " << hover_close_end_pos.pose.position.x << ", " << hover_close_end_pos.pose.position.y << ", " << hover_close_end_pos.pose.position.z);
    rtn_val = arm_motion_commander.rt_arm_plan_path_current_to_goal_pose(hover_close_end_pos);
    rtn_val = arm_motion_commander.rt_arm_execute_planned_path();
    ros::Duration(time_delay).sleep();
    //ros::Duration(time_delay).sleep(); // delay before jumping to next task

    /////////////////////////////////////////////////////
    // 7.move the gripper down to place the beer on table
    /////////////////////////////////////////////////////

    ROS_INFO("task 7: move the gripper down to place the beer on table.");

    // calculate the robot joints when move the gripper down
    geometry_msgs::PoseStamped grasp_close_end_pos;
    grasp_close_end_pos.pose = arm_motion_commander.transformEigenAffine3dToPose(Affine_des_gripper);
    grasp_close_end_pos = target_pos;
    grasp_close_end_pos.pose.position.z = grasp_close_end_pos.pose.position.z + beer_height - torso_height;
    rtn_val = arm_motion_commander.rt_arm_plan_path_current_to_goal_pose(grasp_close_end_pos);
    rtn_val = arm_motion_commander.rt_arm_execute_planned_path();
    //ros::Duration(time_delay).sleep(); // delay before jumping to next task

    /////////////////////////////////////////////
    // 8.unclamp the gripper and release the beer
    /////////////////////////////////////////////

    ROS_INFO("task 8: unclamp the gripper and release the beer.");
    arm_motion_commander.gripperopen();
    ros::Duration(time_delay).sleep();
    // calculate the robot joints after release the beer
    //geometry_msgs::Point around_open_end_pos;
    //around_open_end_pos = target_pos;
    //around_open_end_pos.z = around_open_end_pos.z + beer_height/2;
    //std::vector<double> around_open_end_jnts;
    //around_open_end_jnts.resize(6);
    //around_open_end_jnts = inverseKinematics(around_open_end_pos, gripper_open);

    // assign the start joints and end joints
    //arm_motion_commander.gripper_open();


    // prepare the goal message
    
    ros::Duration(time_delay).sleep(); // delay before jumping to next task

    ///////////////////////////////////
    // 9.move the gripper up from table
    ///////////////////////////////////

    ROS_INFO("task 9: move the gripper up from table.");

    // calculate the robot joints after lift up the gripper
    geometry_msgs::PoseStamped hover_open_end_pos;
    hover_open_end_pos.pose = arm_motion_commander.transformEigenAffine3dToPose(Affine_des_gripper);
    hover_open_end_pos = target_pos;
    hover_open_end_pos.pose.position.z = hover_open_end_pos.pose.position.z + beer_height + lift_height;
    rtn_val = arm_motion_commander.rt_arm_plan_path_current_to_goal_pose(hover_open_end_pos);
    rtn_val = arm_motion_commander.rt_arm_execute_planned_path();
    ros::Duration(time_delay).sleep();
    ROS_INFO("move-the-beer task is finished!");

    return 0;
}

