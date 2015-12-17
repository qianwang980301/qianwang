// example_action_client: 
// wsn, October, 2014

#include<ros/ros.h>
#include <actionlib/client/simple_action_client.h>

//this #include refers to the new "action" message defined for this package
// the action message can be found in: .../example_action_server/action/demo.action
// automated header generation creates multiple headers for message I/O
// these are referred to by the root name (demo) and appended name (Action)
// If you write a new client of the server in this package, you will need to include example_action_server in your package.xml,
// and include the header file below
#include<gripper/dgripperAction.h>
#include <iostream>
#include <string>
#include <std_msgs/Float64.h>

// This function will be called once when the goal completes
// this is optional, but it is a convenient way to get access to the "result" message sent by the server
void doneCb(const actionlib::SimpleClientGoalState& state,
        const gripper::dgripperResultConstPtr& result) {
    ROS_INFO(" doneCb: server responded with state [%s]", state.toString().c_str());
    int diff = result->output - result->goal_stamp;
    ROS_INFO("got result output = %d; goal_stamp = %d; diff = %d",result->output,result->goal_stamp,diff);
}

int main(int argc, char** argv) {
        using namespace std;
        ros::init(argc, argv, "gripper_action_client_node"); // name this node 
        int g_count = 0;
        actionlib::SimpleActionClient<gripper::dgripperAction> action_client("yalegripper", true);
        
        // attempt to connect to the server:
        ROS_INFO("waiting for server: ");
        bool server_exists = action_client.waitForServer(ros::Duration(5.0)); // wait for up to 5 seconds
        // something odd in above: does not seem to wait for 5 seconds, but returns rapidly if server not running
        //bool server_exists = action_client.waitForServer(); //wait forever

        if (!server_exists) {
            ROS_WARN("could not connect to server; halting");
            return 0; // bail out; optionally, could print a warning message and retry
        }
        
       
        ROS_INFO("connected to action server");  // if here, then we connected to the server;

        while(true) { while (ros::ok()) {
        double yale;
        gripper::dgripperGoal goal; 
        cout <<"please input yale";
        cin >>yale;
        g_count++;
        goal.input = g_count; 
        std_msgs::Float64 _yale;
        _yale.data=yale;
        goal.right_yale_gripper_command = _yale.data;
        action_client.sendGoal(goal,&doneCb); 
        
        bool finished_before_timeout = action_client.waitForResult(ros::Duration(5.0));
        
        if (!finished_before_timeout) {
            ROS_WARN("giving up waiting on result for goal number %d",g_count);
            return 0;
        }
        else {
          //if here, then server returned a result to us
        }}
        
        }

    return 0;
}

