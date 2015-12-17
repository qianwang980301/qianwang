#include <grabsth/arm_motion_lib.h>
#include <std_msgs/Float64.h>
ArmMotionCommander::ArmMotionCommander(ros::NodeHandle* nodehandle): nh_(*nodehandle),
cart_move_action_client_("cartMoveActionServer", true) { // constructor
    ROS_INFO("in constructor of ArmMotionInterface");

    // attempt to connect to the server:
    ROS_INFO("waiting for server: ");
    bool server_exists = false;
    while ((!server_exists)&&(ros::ok())) {
        server_exists = cart_move_action_client_.waitForServer(ros::Duration(0.5)); // 
        ros::spinOnce();
        ros::Duration(0.5).sleep();
        ROS_INFO("retrying..#*.");
    }
    ROS_INFO("connected to action server"); // if here, then we connected to the server; 
    
    a_first_publisher = nh_.advertise<std_msgs::Float64>("/gripper/dc_first_joint_effort_controller/command", 1);
    b_second_publisher = nh_.advertise<std_msgs::Float64>("/gripper/dc_second_joint_effort_controller/command", 1);
    c_third_publisher = nh_.advertise<std_msgs::Float64>("/gripper/dc_third_joint_effort_controller/command", 1);
    d_fourth_publisher = nh_.advertise<std_msgs::Float64>("/gripper/dc_fourth_joint_effort_controller/command", 1);
    e_first_publisher = nh_.advertise<std_msgs::Float64>("/gripper/mcp_first_joint_effort_controller/command", 1);
    f_second_publisher = nh_.advertise<std_msgs::Float64>("/gripper/mcp_second_joint_effort_controller/command", 1);
    g_third_publisher = nh_.advertise<std_msgs::Float64>("/gripper/mcp_third_joint_effort_controller/command", 1);
    h_fourth_publisher = nh_.advertise<std_msgs::Float64>("/gripper/mcp_fourth_joint_effort_controller/command", 1);
        std_msgs::Float64 mcp_first;

        std_msgs::Float64 mcp_second;

        std_msgs::Float64 mcp_third;

        std_msgs::Float64 mcp_fourth;
        
        std_msgs::Float64 dc_first;

        std_msgs::Float64 dc_second;

        std_msgs::Float64 dc_third;

        std_msgs::Float64 dc_fourth;
        
        dc_first.data=0.0;

        dc_second.data=0.0;

        dc_third.data=0.0;

        dc_fourth.data=0.0;

        mcp_first.data=0.0;

        mcp_second.data=0.0;

        mcp_third.data=0.0;

        mcp_fourth.data=0.0;

        

        a_first_publisher.publish(dc_first);

        b_second_publisher.publish(dc_second);

        c_third_publisher.publish(dc_third);

        d_fourth_publisher.publish(dc_fourth);
        
        e_first_publisher.publish(mcp_first);

        f_second_publisher.publish(mcp_second);

        g_third_publisher.publish(mcp_third);

        h_fourth_publisher.publish(mcp_fourth);

        
        
}

// This function will be called once when the goal completes
// this is optional, but it is a convenient way to get access to the "result" message sent by the server
//int g_return_code=0;
void ArmMotionCommander::gripperclose(void){
   std_msgs::Float64 dc_first;
   std_msgs::Float64 dc_second;
   std_msgs::Float64 dc_third;
   std_msgs::Float64 dc_fourth;
   std_msgs::Float64 mcp_first;
   std_msgs::Float64 mcp_second;
   std_msgs::Float64 mcp_third;
   std_msgs::Float64 mcp_fourth;
   float a=0.3;
   float b=0.3;
   float c=0.3;
   float d=0.3;
   float e=0.3;
   float f=0.3;
   float g=0.3;
   float h=0.3;
   
   dc_first.data=a;
   dc_second.data=b;
   dc_third.data=c;
   dc_fourth.data=d;
   mcp_first.data=e;
   mcp_second.data=f;
   mcp_third.data=g;
   mcp_fourth.data=h;
        a_first_publisher.publish(dc_first);

        b_second_publisher.publish(dc_second);

        c_third_publisher.publish(dc_third);

        d_fourth_publisher.publish(dc_fourth);
        
        e_first_publisher.publish(mcp_first);

        f_second_publisher.publish(mcp_second);

        g_third_publisher.publish(mcp_third);

        h_fourth_publisher.publish(mcp_fourth);

};
void ArmMotionCommander::gripperopen(void){
   std_msgs::Float64 dc_first;
   std_msgs::Float64 dc_second;
   std_msgs::Float64 dc_third;
   std_msgs::Float64 dc_fourth;
   std_msgs::Float64 mcp_first;
   std_msgs::Float64 mcp_second;
   std_msgs::Float64 mcp_third;
   std_msgs::Float64 mcp_fourth;
   float aa=0.0;
   float ab=0.0;
   float ac=0.0;
   float ad=0.0;
   float ae=0.0;
   float af=0.0;
   float ag=0.0;
   float ah=0.0;
   dc_first.data=aa;
   dc_second.data=ab;
   dc_third.data=ac;
   dc_fourth.data=ad;
   mcp_first.data=ae;
   mcp_second.data=af;
   mcp_third.data=ag;
   mcp_fourth.data=ah;
        a_first_publisher.publish(dc_first);

        b_second_publisher.publish(dc_second);

        c_third_publisher.publish(dc_third);

        d_fourth_publisher.publish(dc_fourth);
        
        e_first_publisher.publish(mcp_first);

        f_second_publisher.publish(mcp_second);

        g_third_publisher.publish(mcp_third);

        h_fourth_publisher.publish(mcp_fourth);

};
void ArmMotionCommander::doneCb_(const actionlib::SimpleClientGoalState& state,
        const cwru_action::cwru_baxter_cart_moveResultConstPtr& result) {
    ROS_INFO(" doneCb: server responded with state [%s]", state.toString().c_str());
    ROS_INFO("got return value= %d", result->return_code);
    cart_result_=*result;
}

//provides an Affine from a Pose for convenience in calculating and transferring values from different functions
Eigen::Affine3d ArmMotionCommander::transformPoseToEigenAffine3d(geometry_msgs::Pose pose) {
    Eigen::Affine3d affine;

    Eigen::Vector3d Oe;

    Oe(0) = pose.position.x;
    Oe(1) = pose.position.y;
    Oe(2) = pose.position.z;
    affine.translation() = Oe;

    Eigen::Quaterniond q;
    q.x() = pose.orientation.x;
    q.y() = pose.orientation.y;
    q.z() = pose.orientation.z;
    q.w() = pose.orientation.w;
    Eigen::Matrix3d Re(q);

    affine.linear() = Re;

    return affine;
}

//caculates a Ros pose from an Affine, allowing use of other function's calculations with ease
geometry_msgs::Pose ArmMotionCommander::transformEigenAffine3dToPose(Eigen::Affine3d e) {
    Eigen::Vector3d Oe;
    Eigen::Matrix3d Re;
    geometry_msgs::Pose pose;
    Oe = e.translation();
    Re = e.linear();

    Eigen::Quaterniond q(Re); // convert rotation matrix Re to a quaternion, q
    pose.position.x = Oe(0);
    pose.position.y = Oe(1);
    pose.position.z = Oe(2);

    pose.orientation.x = q.x();
    pose.orientation.y = q.y();
    pose.orientation.z = q.z();
    pose.orientation.w = q.w();

    return pose;
}

//used to test that the arm motion commander is able to send goals for arm motion correctly
void ArmMotionCommander::send_test_goal(void) {
    ROS_INFO("sending a test goal");
    cart_goal_.command_code = cwru_action::cwru_baxter_cart_moveGoal::ARM_TEST_MODE;
    cart_move_action_client_.sendGoal(cart_goal_, boost::bind(&ArmMotionCommander::doneCb_, this, _1, _2)); // we could also name additional callback functions here, if desired

    finished_before_timeout_ = cart_move_action_client_.waitForResult(ros::Duration(2.0));
        //bool finished_before_timeout = action_client.waitForResult(); // wait forever...
    if (!finished_before_timeout_) {
            ROS_WARN("giving up waiting on result");
        } else {
            ROS_INFO("finished before timeout");
            ROS_INFO("return code: %d",cart_result_.return_code);
        }        
}

//gets the motion plan to move the arm to the pre pose
//the pre pose acts as a starting position for each block pickup procedure
int ArmMotionCommander::plan_move_to_pre_pose(void) {
    ROS_INFO("requesting a joint-space motion plan");
    cart_goal_.command_code = cwru_action::cwru_baxter_cart_moveGoal::RT_ARM_PLAN_JSPACE_PATH_CURRENT_TO_PRE_POSE;
    cart_move_action_client_.sendGoal(cart_goal_, boost::bind(&ArmMotionCommander::doneCb_, this, _1, _2)); // we could also name additional callback functions here, if desired
    finished_before_timeout_ = cart_move_action_client_.waitForResult(ros::Duration(2.0));
    ROS_INFO("return code: %d",cart_result_.return_code);
    if (!finished_before_timeout_) {
            ROS_WARN("giving up waiting on result");
            return (int) cwru_action::cwru_baxter_cart_moveResult::NOT_FINISHED_BEFORE_TIMEOUT;
        } 
    
    ROS_INFO("finished before timeout");
    if (cart_result_.return_code==cwru_action::cwru_baxter_cart_moveResult::RT_ARM_PATH_NOT_VALID) {
        ROS_WARN("right arm plan not valid");
        return (int) cart_result_.return_code;
    }
    if (cart_result_.return_code!=cwru_action::cwru_baxter_cart_moveResult::SUCCESS) {
        ROS_WARN("unknown return code... not SUCCESS");
        return (int) cart_result_.return_code;            
    }   
 
    //here if success return code
    ROS_INFO("returned SUCCESS from planning request");
    computed_arrival_time_= cart_result_.computed_arrival_time; //action_client.get_computed_arrival_time();
    ROS_INFO("computed move time: %f",computed_arrival_time_);
    return (int) cart_result_.return_code;
}

int ArmMotionCommander::rt_arm_plan_jspace_path_current_to_qgoal(Eigen::VectorXd q_des_vec) {    
    ROS_INFO("requesting a joint-space motion plan");
    cart_goal_.command_code = cwru_action::cwru_baxter_cart_moveGoal::RT_ARM_PLAN_JSPACE_PATH_CURRENT_TO_QGOAL;
    cart_goal_.q_goal_right.resize(7);
    for (int i=0;i<7;i++) cart_goal_.q_goal_right[i] = q_des_vec[i]; //specify the goal js pose
    cart_move_action_client_.sendGoal(cart_goal_, boost::bind(&ArmMotionCommander::doneCb_, this, _1, _2)); // we could also name additional callback functions here, if desired
    finished_before_timeout_ = cart_move_action_client_.waitForResult(ros::Duration(2.0));
    ROS_INFO("return code: %d",cart_result_.return_code);
    if (!finished_before_timeout_) {
            ROS_WARN("giving up waiting on result");
            return (int) cwru_action::cwru_baxter_cart_moveResult::NOT_FINISHED_BEFORE_TIMEOUT;
        } 
    
    ROS_INFO("finished before timeout");
    if (cart_result_.return_code==cwru_action::cwru_baxter_cart_moveResult::RT_ARM_PATH_NOT_VALID) {
        ROS_WARN("right arm plan not valid");
        return (int) cart_result_.return_code;
    }
    if (cart_result_.return_code!=cwru_action::cwru_baxter_cart_moveResult::SUCCESS) {
        ROS_WARN("unknown return code... not SUCCESS");
        return (int) cart_result_.return_code;            
    }   
 
    //here if success return code
    ROS_INFO("returned SUCCESS from planning request");
    computed_arrival_time_= cart_result_.computed_arrival_time; //action_client.get_computed_arrival_time();
    ROS_INFO("computed move time: %f",computed_arrival_time_);
    return (int) cart_result_.return_code;    
    
}

//calculates the needed path for baxter's right arm to a goal position
//the goal pose can be the upper, grabbing, or destination pose for a block pickup procedure
int ArmMotionCommander::rt_arm_plan_path_current_to_goal_pose(geometry_msgs::PoseStamped des_pose) {
    
    ROS_INFO("requesting a cartesian-space motion plan");
    cart_goal_.command_code = cwru_action::cwru_baxter_cart_moveGoal::RT_ARM_PLAN_PATH_CURRENT_TO_GOAL_POSE;
    cart_goal_.des_pose_gripper_right = des_pose;
    cart_move_action_client_.sendGoal(cart_goal_, boost::bind(&ArmMotionCommander::doneCb_, this, _1, _2)); // we could also name additional callback functions here, if desired
    finished_before_timeout_ = cart_move_action_client_.waitForResult(ros::Duration(2.0));
    ROS_INFO("return code: %d",cart_result_.return_code);
    if (!finished_before_timeout_) {
            ROS_WARN("giving up waiting on result");
            return (int) cwru_action::cwru_baxter_cart_moveResult::NOT_FINISHED_BEFORE_TIMEOUT;
        } 
    
    ROS_INFO("finished before timeout");
    if (cart_result_.return_code==cwru_action::cwru_baxter_cart_moveResult::RT_ARM_PATH_NOT_VALID) {
        ROS_WARN("right arm plan not valid");
        return (int) cart_result_.return_code;
    }
    if (cart_result_.return_code!=cwru_action::cwru_baxter_cart_moveResult::SUCCESS) {
        ROS_WARN("unknown return code... not SUCCESS");
        return (int) cart_result_.return_code;            
    }   
 
    //here if success return code
    ROS_INFO("returned SUCCESS from planning request");
    computed_arrival_time_= cart_result_.computed_arrival_time; //action_client.get_computed_arrival_time();
    ROS_INFO("computed move time: %f",computed_arrival_time_);
    return (int) cart_result_.return_code;        
}

//gets the xyz displacement of the current path planned
int ArmMotionCommander::rt_arm_plan_path_current_to_goal_dp_xyz(Eigen::Vector3d dp_displacement) {
    
    ROS_INFO("requesting a cartesian-space motion plan along vector");
    cart_goal_.command_code = cwru_action::cwru_baxter_cart_moveGoal::RT_ARM_PLAN_PATH_CURRENT_TO_GOAL_DP_XYZ;
    //must fill in desired vector displacement
    cart_goal_.arm_dp_right.resize(3);
    for (int i=0;i<3;i++) cart_goal_.arm_dp_right[i] = dp_displacement[i];
    cart_move_action_client_.sendGoal(cart_goal_, boost::bind(&ArmMotionCommander::doneCb_, this, _1, _2)); // we could also name additional callback functions here, if desired
    finished_before_timeout_ = cart_move_action_client_.waitForResult(ros::Duration(2.0));
    ROS_INFO("return code: %d",cart_result_.return_code);
    if (!finished_before_timeout_) {
            ROS_WARN("giving up waiting on result");
            return (int) cwru_action::cwru_baxter_cart_moveResult::NOT_FINISHED_BEFORE_TIMEOUT;
        } 
    
    ROS_INFO("finished before timeout");
    if (cart_result_.return_code==cwru_action::cwru_baxter_cart_moveResult::RT_ARM_PATH_NOT_VALID) {
        ROS_WARN("right arm plan not valid");
        return (int) cart_result_.return_code;
    }
    if (cart_result_.return_code!=cwru_action::cwru_baxter_cart_moveResult::SUCCESS) {
        ROS_WARN("unknown return code... not SUCCESS");
        return (int) cart_result_.return_code;            
    }   
 
    //here if success return code
    ROS_INFO("returned SUCCESS from planning request");
    computed_arrival_time_= cart_result_.computed_arrival_time; //action_client.get_computed_arrival_time();
    ROS_INFO("computed move time: %f",computed_arrival_time_);
    return (int) cart_result_.return_code;      
}
    
//requests the execution of a calculated arm path
int ArmMotionCommander::rt_arm_execute_planned_path(void) {
    ROS_INFO("requesting execution of planned path");
    cart_goal_.command_code = cwru_action::cwru_baxter_cart_moveGoal::RT_ARM_EXECUTE_PLANNED_PATH;
    cart_move_action_client_.sendGoal(cart_goal_, boost::bind(&ArmMotionCommander::doneCb_, this, _1, _2)); // we could also name additional callback functions here, if desired
    finished_before_timeout_ = cart_move_action_client_.waitForResult(ros::Duration(computed_arrival_time_+12.0));
    if (!finished_before_timeout_) {
        ROS_WARN("did not complete move in expected time");
        return (int) cwru_action::cwru_baxter_cart_moveResult::NOT_FINISHED_BEFORE_TIMEOUT;  
    }
    if (cart_result_.return_code!=cwru_action::cwru_baxter_cart_moveResult::SUCCESS) {
        ROS_WARN("move did not return success; code = %d",cart_result_.return_code);
        return (int) cart_result_.return_code;
    }

    ROS_INFO("move returned success");
    return (int) cart_result_.return_code;
}

//send goal command to request right-arm joint angles; these will be stored in internal variable
int ArmMotionCommander::rt_arm_request_q_data(void) {
   ROS_INFO("requesting right-arm joint angles");
    cart_goal_.command_code = cwru_action::cwru_baxter_cart_moveGoal::RT_ARM_GET_Q_DATA;
    cart_move_action_client_.sendGoal(cart_goal_, boost::bind(&ArmMotionCommander::doneCb_, this, _1, _2)); // we could also name additional callback functions here, if desired
    finished_before_timeout_ = cart_move_action_client_.waitForResult(ros::Duration(computed_arrival_time_+2.0));
   if (!finished_before_timeout_) {
        ROS_WARN("did not respond within timeout");
        return (int) cwru_action::cwru_baxter_cart_moveResult::NOT_FINISHED_BEFORE_TIMEOUT;  
    }
    if (cart_result_.return_code!=cwru_action::cwru_baxter_cart_moveResult::SUCCESS) {
        ROS_WARN("move did not return success; code = %d",cart_result_.return_code);
        return (int) cart_result_.return_code;
    }
    
    q_vec_ = cart_result_.q_arm_right;
    ROS_INFO("move returned success; right arm angles: ");
    ROS_INFO("%f; %f; %f; %f; %f; %f; %f",q_vec_[0],q_vec_[1],q_vec_[2],q_vec_[3],q_vec_[4],q_vec_[5],q_vec_[6]);
    return (int) cart_result_.return_code;
}

//receive right arm joint angles for internal usage
Eigen::VectorXd ArmMotionCommander::get_right_arm_joint_angles(void) {
    rt_arm_request_q_data();
    Eigen::VectorXd rt_arm_angs_vecXd;
    rt_arm_angs_vecXd.resize(7);
    for (int i=0;i<7;i++) {
        rt_arm_angs_vecXd[i] = q_vec_[i];
    }
    return rt_arm_angs_vecXd;
}

//used to check the yale hand pose
int ArmMotionCommander::rt_arm_request_tool_pose_wrt_torso(void) {
    // debug: compare this to output of:
    //rosrun tf tf_echo torso yale_gripper_frame
    ROS_INFO("requesting right-arm tool pose");    
    cart_goal_.command_code = cwru_action::cwru_baxter_cart_moveGoal::RT_ARM_GET_TOOL_POSE;
    cart_move_action_client_.sendGoal(cart_goal_, boost::bind(&ArmMotionCommander::doneCb_, this, _1, _2)); // we could also name additional callback functions here, if desired
    finished_before_timeout_ = cart_move_action_client_.waitForResult(ros::Duration(2.0));
   if (!finished_before_timeout_) {
        ROS_WARN("did not respond within timeout");
        return (int) cwru_action::cwru_baxter_cart_moveResult::NOT_FINISHED_BEFORE_TIMEOUT;  
    }
    if (cart_result_.return_code!=cwru_action::cwru_baxter_cart_moveResult::SUCCESS) {
        ROS_WARN("move did not return success; code = %d",cart_result_.return_code);
        return (int) cart_result_.return_code;
    }    
    
        tool_pose_stamped_ = cart_result_.current_pose_gripper_right;
        ROS_INFO("move returned success; right arm tool pose: ");
        ROS_INFO("origin w/rt torso = %f, %f, %f ",tool_pose_stamped_.pose.position.x,
                tool_pose_stamped_.pose.position.y,tool_pose_stamped_.pose.position.z);
        ROS_INFO("quaternion x,y,z,w: %f, %f, %f, %f",tool_pose_stamped_.pose.orientation.x,
                tool_pose_stamped_.pose.orientation.y,tool_pose_stamped_.pose.orientation.z,
                tool_pose_stamped_.pose.orientation.w);
  return (int) cart_result_.return_code;
}