#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <gripperhand/hand.h>
float amcp_first=0;
float amcp_second=0;
float amcp_third=0;
float amcp_fourth=0;
float adc_first=0;
float adc_second=0;
float adc_third=0;
float adc_fourth=0;

//int current=0;
HandMove::HandMove(ros::NodeHandle* nodehandle) : nh_(*nodehandle){
	
        mcp_first_publisher = nh_.advertise<std_msgs::Float64>("/gripper/mcp_first_joint_effort_controller/command", 1);
        mcp_second_publisher = nh_.advertise<std_msgs::Float64>("/gripper/mcp_second_joint_effort_controller/command", 1);
        mcp_third_publisher = nh_.advertise<std_msgs::Float64>("/gripper/mcp_third_joint_effort_controller/command", 1);
        mcp_fourth_publisher = nh_.advertise<std_msgs::Float64>("/gripper/mcp_fourth_joint_effort_controller/command", 1);
        dc_first_publisher = nh_.advertise<std_msgs::Float64>("/gripper/dc_first_joint_effort_controller/command", 1);

        dc_second_publisher = nh_.advertise<std_msgs::Float64>("/gripper/dc_second_joint_effort_controller/command", 1);

        dc_third_publisher = nh_.advertise<std_msgs::Float64>("/gripper/dc_third_joint_effort_controller/command", 1);

        dc_fourth_publisher = nh_.advertise<std_msgs::Float64>("/gripper/dc_fourth_joint_effort_controller/command", 1);
	
        std_msgs::Float64 mcp_first;

        std_msgs::Float64 mcp_second;

        std_msgs::Float64 mcp_third;

        std_msgs::Float64 mcp_fourth;
        
        std_msgs::Float64 dc_first;

	std_msgs::Float64 dc_second;

        std_msgs::Float64 dc_third;

        std_msgs::Float64 dc_fourth;
        
        
        
        //opens the hand on start up to know the position
	//position.data = 3900;
        mcp_first.data=0.2;

        mcp_second.data=0.0;

        mcp_third.data=0.0;

        mcp_fourth.data=0.0;

        dc_first.data=0.3;

        dc_first.data=0.0;

        dc_third.data=0.4;

        dc_first.data=0.5;
        
        mcp_first_publisher.publish(mcp_first);

        mcp_second_publisher.publish(mcp_second);

        mcp_third_publisher.publish(mcp_third);

        mcp_fourth_publisher.publish(mcp_fourth);

        dc_first_publisher.publish(dc_first);

        dc_second_publisher.publish(dc_second);

        dc_third_publisher.publish(dc_third);

        dc_fourth_publisher.publish(dc_fourth);
        

        float amcp_first=0.2;

        float amcp_second=0.0;

        float amcp_third=0.0;

        float amcp_fourth=0.0;

        float adc_first=0.3;

        float adc_second=0.0;

        float adc_third=0.4;

        float adc_fourth=0.5;
        
	//position_publisher.publish(position);
	//keep track of current position
	//curpo = 3900;
}



void HandMove::mcp_first_close(){
std_msgs::Float64 mcp_first;
for(float i=amcp_first;i<=0.6;i=i+0.05){
mcp_first.data=i;
mcp_first_publisher.publish(mcp_first);
ros::Duration(.005).sleep(); 
};
amcp_first=0.6;
}

void HandMove::mcp_second_close(){
std_msgs::Float64 mcp_second;
for(float i=amcp_second;i<=0.6;i=i+0.075){
mcp_second.data=i;
mcp_second_publisher.publish(mcp_second);
ros::Duration(.005).sleep(); 
};
amcp_second=0.6;
}

void HandMove::mcp_third_close(){
std_msgs::Float64 mcp_third;
for(float i=amcp_third;i<=0.6;i=i+0.075){
mcp_third.data=i;
mcp_third_publisher.publish(mcp_third);
ros::Duration(.005).sleep(); 
};
amcp_third=0.6;
}

void HandMove::mcp_fourth_close(){
std_msgs::Float64 mcp_fourth;
for(float i=amcp_fourth;i<=0.6;i=i+0.075){
mcp_fourth.data=i;
mcp_fourth_publisher.publish(mcp_fourth);
ros::Duration(.005).sleep(); 
};
amcp_fourth=0.6;
}

void HandMove::dc_first_close(){
std_msgs::Float64 dc_first;
for(float i=adc_first;i<=0.8;i=i+0.0625){
dc_first.data=i;
dc_first_publisher.publish(dc_first);
ros::Duration(.005).sleep(); 
};
adc_first=0.8;
}

void HandMove::dc_second_close(){
std_msgs::Float64 dc_second;
for(float i=adc_second;i<=0.8;i=i+0.1){
dc_second.data=i;
dc_second_publisher.publish(dc_second);
ros::Duration(.005).sleep(); 
};
adc_second=0.8;
}

void HandMove::dc_third_close(){
std_msgs::Float64 dc_third;
for(float i=adc_third;i<=0.8;i=i+0.05){
dc_third.data=i;
dc_third_publisher.publish(dc_third);
ros::Duration(.005).sleep(); 
};
adc_third=0.8;
}

void HandMove::dc_fourth_close(){
std_msgs::Float64 dc_fourth;
for(float i=adc_fourth;i<=0.8;i=i+0.0375){
dc_fourth.data=i;
dc_fourth_publisher.publish(dc_fourth);
ros::Duration(.005).sleep(); 
};
adc_fourth=0.8;
}

//void GripperMove::gripper_open(){
//std_msgs::Int16 position;
//for(int i = curpo; i<=3900; i++){
//position.data = i;
//position_publisher.publish(position);
//ros::Duration(.0005).sleep(); 
//}

//curpo = 3900;
//}
