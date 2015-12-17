#include <ros/ros.h> 
#include <gripperhand/hand.h>

int main(int argc, char** argv) {

    ros::init(argc, argv, "fhandtest"); 
    ros::NodeHandle nh;
    HandMove fhandtest(&nh);


    while (ros::ok()) {
        
        fhandtest.dc_second_close();
    	//handtest.gripper_close();
    	//ros::Duration(1).sleep(); 
    	//grippertest.gripper_open();
    	//ros::Duration(1).sleep(); 
        //ros::spinOnce();
    }
}

    
