// example_action_server: a simple action server
// this version does not depend on actionlib_servers hku package
// wsn, October, 2014

#include<ros/ros.h>
#include <actionlib/server/simple_action_server.h>
//the following #include refers to the "action" message defined for this package
// The action message can be found in: .../example_action_server/action/demo.action
// Automated header generation creates multiple headers for message I/O
// These are referred to by the root name (demo) and appended name (Action)
#include<gripper/dgripperAction.h>
#include <iostream>
#include <string>
#include <std_msgs/Float64.h>
int g_count = 0;
double a;
double b;
double c;
double d;
double e;
double f;
double g;
double h;
bool g_count_failure = false;

class GripperActionServer {
private:
    
    ros::NodeHandle nh_;  // we'll need a node handle; get one upon instantiation
  
    // this class will own a "SimpleActionServer" called "as_".
    // it will communicate using messages defined in example_action_server/action/demo.action
    // the type "demoAction" is auto-generated from our name "demo" and generic name "Action"
    actionlib::SimpleActionServer<gripper::dgripperAction> as_;
    
    // here are some message types to communicate with our client(s)
    gripper::dgripperGoal goal_; // goal message, received from client
    gripper::dgripperResult result_; // put results here, to be sent back to the client when done w/ goal
    gripper::dgripperFeedback feedback_; // not used in this example; 
    // would need to use: as_.publishFeedback(feedback_); to send incremental feedback to the client
    

public:
    GripperActionServer(); //define the body of the constructor outside of class definition

    ~GripperActionServer(void) {
    }
    // Action Interface
    void executeCB(const actionlib::SimpleActionServer<gripper::dgripperAction>::GoalConstPtr& goal);
};

//implementation of the constructor:
// member initialization list describes how to initialize member as_
// member as_ will get instantiated with specified node-handle, name by which this server will be known,
//  a pointer to the function to be executed upon receipt of a goal.
//  
// Syntax of naming the function to be invoked: get a pointer to the function, called executeCB, which is a member method
// of our class exampleActionServer.  Since this is a class method, we need to tell boost::bind that it is a class member,
// using the "this" keyword.  the _1 argument says that our executeCB takes one argument
// the final argument  "false" says don't start the server yet.  (We'll do this in the constructor)

GripperActionServer::GripperActionServer() :
   as_(nh_, "yalegripper", boost::bind(&GripperActionServer::executeCB, this, _1),false) 
// in the above initialization, we name the server "example_action"
//  clients will need to refer to this name to connect with this server
{
    ROS_INFO("in constructor of exampleActionServer...");
    // do any other desired initializations here...specific to your implementation

    as_.start(); //start the server running
}

//executeCB implementation: this is a member method that will get registered with the action server
// argument type is very long.  Meaning:
// actionlib is the package for action servers
// SimpleActionServer is a templated class in this package (defined in the "actionlib" ROS package)
// <example_action_server::demoAction> customizes the simple action server to use our own "action" message 
// defined in our package, "example_action_server", in the subdirectory "action", called "demo.action"
// The name "demo" is prepended to other message types created automatically during compilation.
// e.g.,  "demoAction" is auto-generated from (our) base name "demo" and generic name "Action"
void GripperActionServer::executeCB(const actionlib::SimpleActionServer<gripper::dgripperAction>::GoalConstPtr& goal) {
 

a=goal->right_yale_gripper_command;
b=goal->right_yale_gripper_command;
c=goal->right_yale_gripper_command;
d=goal->right_yale_gripper_command;
e=goal->right_yale_gripper_command;
f=goal->right_yale_gripper_command;
g=goal->right_yale_gripper_command;
h=goal->right_yale_gripper_command;




//ROS_INFO("in executeCB");
    //ROS_INFO("goal input is: %d", goal->input);
    //do work here: this is where your interesting code goes
    
    //....

    // for illustration, populate the "result" message with two numbers:
    // the "input" is the message count, copied from goal->input (as sent by the client)
    // the "goal_stamp" is the server's count of how many goals it has serviced so far
    // if there is only one client, and if it is never restarted, then these two numbers SHOULD be identical...
    // unless some communication got dropped, indicating an error
    // send the result message back with the status of "success"

    g_count++; // keep track of total number of goals serviced since this server was started
    result_.output = g_count; // we'll use the member variable result_, defined in our class
    result_.goal_stamp = goal->input;
    
    // the class owns the action server, so we can use its member methods here
   
    // DEBUG: if client and server remain in sync, all is well--else whine and complain and quit
    // NOTE: this is NOT generically useful code; server should be happy to accept new clients at any time, and
    // no client should need to know how many goals the server has serviced to date
    if (g_count != goal->input) {
        ROS_WARN("hey--mismatch!");
        ROS_INFO("g_count = %d; goal_stamp = %d", g_count, result_.goal_stamp);
        g_count_failure = true; //set a flag to commit suicide
        ROS_WARN("informing client of aborted goal");
        as_.setAborted(result_); // tell the client we have given up on this goal; send the result message as well
    }
    else {
         as_.setSucceeded(result_); // tell the client that we were successful acting on the request, and return the "result" message
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "gripper_action_server_node"); // name this node 

    ROS_INFO("instantiating the gripper action server: ");

    GripperActionServer as_object; // create an instance of the class "exampleActionServer"
    
    ROS_INFO("going into spin");
ros::NodeHandle n;
ros::Publisher a_publisher = n.advertise<std_msgs::Float64>("/gripper/dc_first_joint_effort_controller/command", 1);
ros::Publisher b_publisher = n.advertise<std_msgs::Float64>("/gripper/dc_second_joint_effort_controller/command", 1);
ros::Publisher c_publisher = n.advertise<std_msgs::Float64>("/gripper/dc_third_joint_effort_controller/command", 1);
ros::Publisher d_publisher = n.advertise<std_msgs::Float64>("/gripper/dc_fourth_joint_effort_controller/command", 1);
ros::Publisher e_publisher = n.advertise<std_msgs::Float64>("/gripper/mcp_first_joint_effort_controller/command", 1);
ros::Publisher f_publisher = n.advertise<std_msgs::Float64>("/gripper/mcp_second_joint_effort_controller/command", 1);
ros::Publisher g_publisher = n.advertise<std_msgs::Float64>("/gripper/mcp_third_joint_effort_controller/command", 1);
ros::Publisher h_publisher = n.advertise<std_msgs::Float64>("/gripper/mcp_fourth_joint_effort_controller/command", 1);
   std_msgs::Float64 _a;
   std_msgs::Float64 _b;
   std_msgs::Float64 _c;
   std_msgs::Float64 _d;
   std_msgs::Float64 _e;
   std_msgs::Float64 _f;
   std_msgs::Float64 _g;
   std_msgs::Float64 _h;


    // from here, all the work is done in the action server, with the interesting stuff done within "executeCB()"
    // you will see 5 new topics under example_action: cancel, feedback, goal, result, status
    while (ros::ok()) {
_a.data=a;
_b.data=b;
_c.data=c;
_d.data=d;
_e.data=e;
_f.data=f;
_g.data=g;
_h.data=h;
a_publisher.publish(_a);
b_publisher.publish(_b);
c_publisher.publish(_c);
d_publisher.publish(_d);
e_publisher.publish(_e);
f_publisher.publish(_f);
g_publisher.publish(_g);
h_publisher.publish(_h);

        ros::spinOnce(); //normally, can simply do: ros::spin();  
        // for debug, induce a halt if we ever get our client/server communications out of sync
    }

    return 0;
}

