
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Odometry.h>
#include <cmath>

ros::Publisher marker_pub;
visualization_msgs::Marker marker;
/// move to pickUp position? If yes, pickUp_flag = true
bool pickUp_flag = true;




bool check_close(const nav_msgs::Odometry::ConstPtr& msg) {
    float dx = msg->pose.pose.position.x - marker.pose.position.x;
    float dy = msg->pose.pose.position.y - marker.pose.position.y;
    float dist = sqrt(dx * dx + dy * dy);
    if (dist <= 0.8) {
       //turn_on = true;
       return true;
    }
    return false;
}


void add_marker_callback(const nav_msgs::Odometry::ConstPtr& msg){

    // Publish the marker
     ROS_INFO("Current position x:%1.2f, y:%1.2f ", msg->pose.pose.position.x,msg->pose.pose.position.y);

     
    
     /// is the robot close to the marker? If yes close_flag = true;
     bool close_flag = check_close(msg);
     /// status_flag = true when publish the marker
     bool status_flag = (close_flag ^ pickUp_flag);
     ROS_INFO("Current status %d, close_flage: %d, pickUp_flag:%d ", status_flag, close_flag, pickUp_flag);


    if (status_flag) { 
        //// if status_flag is true, publish the marker 
        marker.lifetime = ros::Duration();
        marker_pub.publish(marker);
    } else {
        ///// otherwise, change marker parameters to the drop_off position
        ros::Duration(5);
        pickUp_flag = false;
        marker.action = visualization_msgs::Marker::DELETE;
        marker_pub.publish(marker);
        marker.pose.position.x = 4;
        marker.pose.position.y = 4; 
        marker.action = visualization_msgs::Marker::ADD;    
    }
    
   // r.sleep();  
}




int main( int argc, char** argv )
{
  ros::init(argc, argv, "add_markers");
  ros::NodeHandle n;
  ros::Rate r(1);
  
  marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
   //// subscribe to the odom 
  ros::Subscriber odom_sub = n.subscribe("/odom", 100, add_marker_callback);

   // Set our initial shape type to be a cube 
   uint32_t shape = visualization_msgs::Marker::CUBE;
  
    
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one

    marker.ns = "basic_shapes";
    marker.id = 0;

    // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
    marker.type = shape;

    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    marker.action = visualization_msgs::Marker::ADD;

    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    marker.pose.position.x = -1;
    marker.pose.position.y = -1;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = 1.0;
    marker.scale.y = 1.0;
    marker.scale.z = 1.0;

    // Set the color -- be sure to set alpha to something non-zero!
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;
    
    
    ros::spin();
      
}
