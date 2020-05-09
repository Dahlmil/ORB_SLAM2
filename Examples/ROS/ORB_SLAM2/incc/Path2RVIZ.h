#include <tf/tf.h>
#include <ros/ros.h>
#include <ros/console.h>
#include <nav_msgs/Path.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/Marker.h> //可视化

struct RVIZ_POS
{
    int x;
    int y;
    int z;
    int yaw;
};

void DrawPath(double x, double y, double th, ros::Publisher &path_pub, nav_msgs::Path &path);

void DrawMarker(double x, double y, ros::Publisher &marker_pub, visualization_msgs::Marker &marker);