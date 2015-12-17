#include <ros/ros.h> 
#include <gripperhand/hand.h>

int main(int argc, char** argv) {

    ros::init(argc, argv, "ehandtest"); 
    ros::NodeHandle nh;
    HandMove ehandtest(&nh);


    while (ros::ok()) {
        
        ehandtest.dc_first_close();
    	//handtest.gripper_close();
    	//ros::Duration(1).sleep(); 
    	//grippertest.gripper_open();
    	//ros::Duration(1).sleep(); 
        //ros::spinOnce();
    }
}

    
