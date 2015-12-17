

#ifndef HAND_H_
#define HAND_H_

#include<ros/ros.h>



class HandMove
{
public:
    HandMove(ros::NodeHandle* nodehandle);
    void mcp_first_close();
    void mcp_second_close();
    void mcp_third_close();
    void mcp_fourth_close();
    void dc_first_close();
    void dc_second_close();
    void dc_third_close();
    void dc_fourth_close();    


private:
       ros::NodeHandle nh_;
       ros::Publisher mcp_first_publisher;
       ros::Publisher mcp_second_publisher;
       ros::Publisher mcp_third_publisher;
       ros::Publisher mcp_fourth_publisher;
       ros::Publisher dc_first_publisher;
       ros::Publisher dc_second_publisher;
       ros::Publisher dc_third_publisher;
       ros::Publisher dc_fourth_publisher;
};

#endif  
