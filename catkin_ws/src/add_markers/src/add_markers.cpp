#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Odometry.h>
#include <my_robot_interfaces/RobotMoveState.h>

visualization_msgs::Marker marker;

double pick_up[2] = {2, -4};
double drop_off[2] = {2, -7};

void set_marker_pose(double *posittion, bool delete_marker)
{
    ros::NodeHandle n;
    ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);

    // Wait until there's a subscriber
    while (marker_pub.getNumSubscribers() < 1)
    {
        if (!ros::ok())
        {
            return;
        }
        ROS_WARN_ONCE("Please create a subscriber to the marker");
        
        sleep(1);
    }

    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker.ns = "add_markers";
    marker.id = 0;

    // Set the marker type to cube.
    marker.type = visualization_msgs::Marker::CUBE;

    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    if(delete_marker)
        marker.action = visualization_msgs::Marker::DELETE;
    else
        marker.action = visualization_msgs::Marker::ADD;

    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    marker.pose.position.x = posittion[0];
    marker.pose.position.y = posittion[1];
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = 0.3;
    marker.scale.y = 0.3;
    marker.scale.z = 0.3;

    // Set the color -- be sure to set alpha to something non-zero!
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;

    marker.lifetime = ros::Duration();

    // Publish the marker
    marker_pub.publish(marker);
    ROS_INFO("Marker published");

}

void robot_state(my_robot_interfaces::RobotMoveState msg)
{
    
    if(msg.robot_goal_reached == true)
    {
        if(msg.goal_type == my_robot_interfaces::RobotMoveState::PICKUP)
        {
            ROS_INFO("Object picked up successfully");
            set_marker_pose(pick_up, true);
        }
        else
        {
            ROS_INFO("Object dropped off successfully");
            set_marker_pose(drop_off, false);
        }
    }
    else
    {
        ROS_ERROR("Failed to move the robot");
    }
}

void marker_simulation()
{
    ROS_INFO("Object picked up successfully");
    set_marker_pose(pick_up, true);
    sleep(5);
    ROS_INFO("Object dropped off successfully");
    set_marker_pose(drop_off, false);
}

int main( int argc, char** argv )
{
    ros::init(argc, argv, "add_markers");
    
    ros::NodeHandle n;

    // Monitor the state of the robot
    ros::Subscriber sub = n.subscribe("/my_robot_interfaces/robot_move_state", 10, robot_state);

    set_marker_pose(pick_up, false);

    bool _add_marker_simulation;
    
    n.getParam("add_marker_simulation_param", _add_marker_simulation);

    if(_add_marker_simulation)
    {
        marker_simulation();
    }

    ros::spin();

    return 0;
}
