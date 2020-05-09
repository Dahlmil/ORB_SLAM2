#include "Path2RVIZ.h"

void DrawPath(double x, double y, double th, ros::Publisher &path_pub, nav_msgs::Path &path)
{

    geometry_msgs::PoseStamped this_pose_stamped;
    this_pose_stamped.pose.position.x = x;
    this_pose_stamped.pose.position.y = y;

    geometry_msgs::Quaternion goal_quat = tf::createQuaternionMsgFromYaw(th);
    this_pose_stamped.pose.orientation.x = goal_quat.x;
    this_pose_stamped.pose.orientation.y = goal_quat.y;
    this_pose_stamped.pose.orientation.z = goal_quat.z;
    this_pose_stamped.pose.orientation.w = goal_quat.w;

    this_pose_stamped.header.stamp = ros::Time::now();
    this_pose_stamped.header.frame_id = "map";
    path.poses.push_back(this_pose_stamped);

    path_pub.publish(path);
}

void DrawMarker(double x, double y, ros::Publisher &marker_pub, visualization_msgs::Marker &marker)
{
    uint32_t shape = visualization_msgs::Marker::CYLINDER;
    //实例化一个Marker

    // 设置frame ID 和 时间戳
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();
    // 为这个marker设置一个独一无二的ID，一个marker接收到相同ns和id就会用新的信息代替旧的
    marker.ns = "basic_shapes";
    marker.id = 0;
    // 设置marker类型，初始化是立方体
    marker.type = shape;
    // 添加marker
    marker.action = visualization_msgs::Marker::ADD;
    // 设置marker的位置
    marker.pose.position.x = x;
    marker.pose.position.y = y;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    // 设置marker的大小
    marker.scale.x = 15.0;
    marker.scale.y = 15.0;
    marker.scale.z = 0.5;
    // Set the color -- be sure to set alpha to something non-zero!
    // 设置marker的颜色
    // marker.color.r = 1.0f;
    // marker.color.g = 1.0f;
    // marker.color.b = 1.0f;
    // marker.color.a = 1.0;

    //取消自动删除
    marker.lifetime = ros::Duration();

    // Publish the marker
    // 必须有订阅者才会发布消息
    while (marker_pub.getNumSubscribers() < 1)
    {
        ROS_WARN_ONCE("Please create a subscriber to the marker");
        sleep(1);
    }
    marker_pub.publish(marker);
}