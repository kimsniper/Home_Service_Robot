#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <visualization_msgs/Marker.h>
#include <my_robot_interfaces/RobotMoveState.h>
 
// Define a client for to send goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int robot_goal_type = my_robot_interfaces::RobotMoveState::PICKUP;

double pick_up[2] = {2, -4};
double drop_off[2] = {2, -7};

void move_robot(double *position)
{
    ros::NodeHandle n;
    // Publisher to notify add marker node when pick up is successful or not
    ros::Publisher pub = n.advertise<my_robot_interfaces::RobotMoveState>("/my_robot_interfaces/robot_move_state", 10);

    ROS_INFO("Position x: %f", position[0]);
    ROS_INFO("Position y: %f", position[1]);
    //tell the action client that we want to spin a thread by default
    MoveBaseClient ac("move_base", true);

    // Wait 5 sec for move_base action server to come up
    while(!ac.waitForServer(ros::Duration(5.0))){
        ROS_INFO("Waiting for the move_base action server to come up");
    }

    move_base_msgs::MoveBaseGoal goal;

    // set up the frame parameters
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();

    // Define a position and orientation for the robot to reach
    goal.target_pose.pose.position.x = position[0];
    goal.target_pose.pose.position.y = position[1];
    goal.target_pose.pose.position.z = 0;
    goal.target_pose.pose.orientation.w = 1;

    // Send the goal position and orientation for the robot to reach
    ROS_INFO("Sending goal");
    ac.sendGoal(goal);
    
    // Wait an infinite time for the results
    ac.waitForResult();
    
    my_robot_interfaces::RobotMoveState msg;

    // Check if the robot reached its goal
    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        if(robot_goal_type == my_robot_interfaces::RobotMoveState::PICKUP)
        {
            // Publish msg to notify the add markers node that the object was picked up
            ROS_INFO("The robot reached the pickup location of object");
            msg.goal_type = robot_goal_type;
            msg.robot_goal_reached = true;
            robot_goal_type = my_robot_interfaces::RobotMoveState::DROPOFF;
            pub.publish(msg);
        }
        else
        {
            ROS_INFO("The robot reached the dropoff location of object");
            msg.goal_type = robot_goal_type;
            msg.robot_goal_reached = true;
            pub.publish(msg);
        }
    }
    else
    {
        ROS_INFO("The robot failed to pick up object");
        msg.goal_type = robot_goal_type;
        msg.robot_goal_reached = false;
        pub.publish(msg);
    }
}

int main(int argc, char** argv){
    // Initialize the pick_objects node
    ros::init(argc, argv, "pick_objects");

    ros::NodeHandle n;

    /* Go to pickup location */
    move_robot(pick_up);
    
    sleep(5);
    /* Go to dropoff location */
    move_robot(drop_off);

    // Spin so that the terminal window will not close after execution. Stop script file to stop execution
    ros::spin();

    return 0;
}