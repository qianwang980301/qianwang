#include <ros/ros.h> 
#include <gripperhand/hand.h>

int main(int argc, char** argv) {

    ros::init(argc, argv, "ghandtest"); 
    ros::NodeHandle nh;
    HandMove ghandtest(&nh);


    while (ros::ok()) {
        
        ghandtest.dc_third_close();
    	//handtest.gripper_close();
    	//ros::Duration(1).sleep(); 
    	//grippertest.gripper_open();
    	//ros::Duration(1).sleep(); 
        //ros::spinOnce();
    }
}

    
