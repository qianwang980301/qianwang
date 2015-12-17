#include <ros/ros.h> 
#include <gripperhand/hand.h>

int main(int argc, char** argv) {

    ros::init(argc, argv, "hhandtest"); 
    ros::NodeHandle nh;
    HandMove hhandtest(&nh);


    while (ros::ok()) {
        
        hhandtest.dc_fourth_close();
    	//handtest.gripper_close();
    	//ros::Duration(1).sleep(); 
    	//grippertest.gripper_open();
    	//ros::Duration(1).sleep(); 
        //ros::spinOnce();
    }
}

    
