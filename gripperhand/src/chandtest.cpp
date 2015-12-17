#include <ros/ros.h> 
#include <gripperhand/hand.h>

int main(int argc, char** argv) {

    ros::init(argc, argv, "chandtest"); 
    ros::NodeHandle nh;
    HandMove chandtest(&nh);


    while (ros::ok()) {
        
        chandtest.mcp_third_close();
    	//handtest.gripper_close();
    	//ros::Duration(1).sleep(); 
    	//grippertest.gripper_open();
    	//ros::Duration(1).sleep(); 
        //ros::spinOnce();
    }
}

    
