#include <ros/ros.h> 
#include <gripperhand/hand.h>

int main(int argc, char** argv) {

    ros::init(argc, argv, "bhandtest"); 
    ros::NodeHandle nh;
    HandMove bhandtest(&nh);


    while (ros::ok()) {
        
        bhandtest.mcp_second_close();
    	//handtest.gripper_close();
    	//ros::Duration(1).sleep(); 
    	//grippertest.gripper_open();
    	//ros::Duration(1).sleep(); 
        //ros::spinOnce();
    }
}

    
