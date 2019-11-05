#include <warehouse_localization.h>



WarehouseLocalization::WarehouseLocalization()
{
    tag_msg_time_threshold_.sec = 0;
    tag_msg_time_threshold_.nsec = 100000000;
}

WarehouseLocalization::~WarehouseLocalization()
{

}

void WarehouseLocalization::setVOdomPose(geometry_msgs::Pose2D pose)
{
    v_odom_pose_ = pose;
}

void WarehouseLocalization::setTagPose(geometry_msgs::Pose2D pose)
{
    tag_pose_ = pose;
}

void WarehouseLocalization::setLastTagMsgTime(ros::Time now)
{
    last_tag_msg_time_ = now;
}

geometry_msgs::Pose2D WarehouseLocalization::getVOdomPose()
{
    return v_odom_pose_;
}

geometry_msgs::Pose2D WarehouseLocalization::getTagPose()
{
    return tag_pose_;
}

geometry_msgs::Pose2D WarehouseLocalization::getOdomMapTransform(geometry_msgs::Pose2D odom_nav_pose,
                                                                 geometry_msgs::Pose2D map_nav_pose)
{
    double xm_prime = map_nav_pose.x * cos(map_nav_pose.theta - odom_nav_pose.theta) +
                      map_nav_pose.y * sin(map_nav_pose.theta - odom_nav_pose.theta);
    double ym_prime = - map_nav_pose.x * sin(map_nav_pose.theta - odom_nav_pose.theta) +
                      map_nav_pose.y * cos(map_nav_pose.theta - odom_nav_pose.theta);

    double delta_xm_prime = xm_prime - odom_nav_pose.x;
    double delta_ym_prime = ym_prime - odom_nav_pose.y;
    geometry_msgs::Pose2D transform;
    transform.x = -delta_xm_prime;
    transform.y = -delta_ym_prime;
    transform.theta = -(map_nav_pose.theta - odom_nav_pose.theta);
    return transform;
}

ros::Time WarehouseLocalization::getLastTagMsgTime()
{
    return last_tag_msg_time_;
}

ros::Duration WarehouseLocalization::getTagMsgTimeThreshold()
{
    return tag_msg_time_threshold_;
}

void WarehouseLocalization::visualOdometryDataCallback(const geometry_msgs::Pose2D::ConstPtr& msg)
{
    setVOdomPose(*msg);
}

void WarehouseLocalization::brodcastVOdomWRTStart(tf::Transform& transform_v_odom, tf::Quaternion& q_v_odom, tf::TransformBroadcaster& br)
{
    geometry_msgs::Pose2D vodom_pose = getVOdomPose();
    transform_v_odom.setOrigin( tf::Vector3(vodom_pose.y, 0, vodom_pose.x));
    q_v_odom.setRPY(0, vodom_pose.theta, 0);
    transform_v_odom.setRotation(q_v_odom);
    br.sendTransform(tf::StampedTransform(transform_v_odom, ros::Time::now(), "start", "v_odom_camera"));
}

void WarehouseLocalization::tagGlobalPoseCallback(const geometry_msgs::Pose2D::ConstPtr& msg)
{
    setTagPose(*msg);
    setLastTagMsgTime(ros::Time::now());
}

void WarehouseLocalization::lookupTransform(std::string from_frame, std::string to_frame,
                     tf::TransformListener& listener, tf::StampedTransform& transform)
{
    try
    {
        listener.lookupTransform(from_frame, to_frame, ros::Time(0), transform);
    }
    catch (tf::TransformException &ex)
    {
        //ROS_ERROR("%s",ex.what());
    }
}