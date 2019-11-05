#ifndef WAREHOUSE_LOCALIZATION_H
#define WAREHOUSE_LOCALIZATION_H

#include <math.h>
#include <stdint.h>
#include <iostream>
#include <ros/ros.h>
#include <geometry_msgs/Pose2D.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

class WarehouseLocalization
{
private:
    geometry_msgs::Pose2D v_odom_pose_;
    //static pose_3 v_odom_pose_;
    geometry_msgs::Pose2D tag_pose_;
    ros::Time last_tag_msg_time_;
    ros::Duration tag_msg_time_threshold_;
public:
    WarehouseLocalization();
    ~WarehouseLocalization();

    void setVOdomPose(geometry_msgs::Pose2D pose);
    void setTagPose(geometry_msgs::Pose2D pose);
    void setLastTagMsgTime(ros::Time now);

    geometry_msgs::Pose2D getVOdomPose();
    geometry_msgs::Pose2D getTagPose();
    ros::Time getLastTagMsgTime();
    ros::Duration getTagMsgTimeThreshold();
    geometry_msgs::Pose2D getOdomMapTransform(geometry_msgs::Pose2D odom_nav_pose, geometry_msgs::Pose2D map_nav_pose);

    void visualOdometryDataCallback(const geometry_msgs::Pose2D::ConstPtr& msg);
    void brodcastVOdomWRTStart(tf::Transform& transform_v_odom, tf::Quaternion& q_v_odom, tf::TransformBroadcaster& br);

    void tagGlobalPoseCallback(const geometry_msgs::Pose2D::ConstPtr& msg);
    void lookupTransform(std::string from_frame, std::string to_frame,
                         tf::TransformListener& listener, tf::StampedTransform& transform);

};
#endif