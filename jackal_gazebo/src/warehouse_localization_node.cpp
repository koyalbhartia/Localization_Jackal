/*
 * Node file
 * subscribes to visual_odometry_data and tag_global_pose
 * based on visual odometry,
 * updates zed tf with respect to start frame and updates start frame with respect to odom frame
 * based on tag data,
 * updates map frame with respect to odom frame
 * Final pose of the robot can be determined by looking up base_link with respect to map
 */

#include <warehouse_localization.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "warehouse_localization_node");
    ros::NodeHandle n;
    WarehouseLocalization whl;
    ros::Subscriber visual_odometry_data_sub = n.subscribe("visual_odometry_data", 1, &WarehouseLocalization::visualOdometryDataCallback, &whl);
    ros::Subscriber tag_global_pose_sub =n.subscribe("tag_global_pose", 1, &WarehouseLocalization::tagGlobalPoseCallback, &whl);

    ros::Publisher log_tag_data_pub = n.advertise<geometry_msgs::Pose2D>("log_tag_data", 1);
    ros::Publisher log_vodom_data_pub = n.advertise<geometry_msgs::Pose2D>("log_vodom_data", 1);
    ros::Publisher log_pose_data_pub = n.advertise<geometry_msgs::Pose2D>("log_pose_data", 1);

    ros::Rate loop_rate(30);

    tf::TransformBroadcaster br;
    tf::TransformListener lr;

    tf::Transform transform_v_odom;
    transform_v_odom.setRotation({0, 0, 0, 1});
    tf::Quaternion q_v_odom;
    tf::StampedTransform transform_odom_nav;
    tf::Transform transform_map;
    transform_map.setOrigin(tf::Vector3(0, 0, 0));
    transform_map.setRotation({0, 0, 0, 1});
    tf::Transform transform_map_debug;
    tf::Quaternion q_map_debug;
    transform_map_debug.setRotation({0, 0, 0, 1});
    tf::Quaternion q_map;

    geometry_msgs::Pose2D map_nav_pose;
    geometry_msgs::Pose2D odom_nav_pose;

    tf::TransformListener log_lr;
    tf::StampedTransform log_transform_map_bl;
    while(ros::ok())
    {
        //publish VOdom wrt Start
        //whl.brodcastVOdomWRTStart(transform_v_odom, q_v_odom, br);

        //to adjust map frame wrt odom, nav_camera is looked up wrt odom
        /*whl.lookupTransform("odom", "nav_camera", lr, transform_odom_nav);
        odom_nav_pose.x = transform_odom_nav.getOrigin().x();
        odom_nav_pose.y = transform_odom_nav.getOrigin().y();
        tf::Matrix3x3 cam_m(transform_odom_nav.getRotation());
        double c_roll, c_pitch, c_yaw;
        cam_m.getRPY(c_roll, c_pitch, c_yaw);
        odom_nav_pose.theta = c_yaw;*/

        //map pose is nav_camera wrt map
        map_nav_pose = whl.getTagPose();
        transform_map_debug.setOrigin(tf::Vector3(map_nav_pose.x, map_nav_pose.y, 0));
        q_map_debug.setRPY(0, 0, map_nav_pose.theta);
        transform_map_debug.setRotation(q_map_debug);
        br.sendTransform(tf::StampedTransform(transform_map_debug, ros::Time::now(), "map", "base_link"));


        //look up when was the last tag message was received
        /*ros::Time last_msg_time = whl.getLastTagMsgTime();
        ros::Duration time_diff=ros::Time::now()-last_msg_time;
        //only change the tf between map and odom while seeing tag
        if(time_diff <= whl.getTagMsgTimeThreshold())
        {
            //Relation between odom and map is updated
            geometry_msgs::Pose2D odom_map_transform = whl.getOdomMapTransform(odom_nav_pose, map_nav_pose);
            transform_map.setOrigin( tf::Vector3(odom_map_transform.x, odom_map_transform.y, 0) );
            q_map.setRPY(0, 0, odom_map_transform.theta);
            transform_map.setRotation(q_map);
        }*/
        br.sendTransform(tf::StampedTransform(transform_map, ros::Time::now(), "map", "odom"));

        //This code block is just for logging purposes
        //block begin
        /*geometry_msgs::Pose2D vodom_pose = whl.getVOdomPose();
        log_vodom_data_pub.publish(vodom_pose);

        geometry_msgs::Pose2D tag_pose = whl.getTagPose();
        log_tag_data_pub.publish(tag_pose);

        geometry_msgs::Pose2D pose;
        whl.lookupTransform("map", "base_link", log_lr, log_transform_map_bl);
        pose.x = log_transform_map_bl.getOrigin().x();
        pose.y = log_transform_map_bl.getOrigin().y();
        tf::Matrix3x3 log_rot(log_transform_map_bl.getRotation());
        double log_roll, log_pitch, log_yaw;
        log_rot.getRPY(log_roll, log_pitch, log_yaw);
        pose.theta = log_yaw;
        log_pose_data_pub.publish(pose);*/

        //block end
        ros::spinOnce();
        loop_rate.sleep();
    }


    return 0;
}
