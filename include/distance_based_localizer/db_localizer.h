#ifndef DB_LOCALIZER_H
#define DB_LOCALIZER_H

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Header.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <vector>
#include <tf/tf.h>
#include "object_detector_msgs/ObjectPosition.h"
#include "object_detector_msgs/ObjectPositions.h"

class DistanceBasedLocalizer
{
    public:
        DistanceBasedLocalizer();
        void process();

    private:
        //method
        void object_callback(const object_detector_msgs::ObjectPositions::ConstPtr &msg);
        void map_callback(const nav_msgs::OccupancyGrid::ConstPtr &msg);
        void odometry_callback(const nav_msgs::Odometry::ConstPtr &msg);
        void pose_callback(const geometry_msgs::PoseStamped::ConstPtr &msg);

        void get_rpy(const geometry_msgs::Quaternion &q,double &roll,double &pitch,double &yaw);
        void get_quat(double roll,double pitch,double yaw,geometry_msgs::Quaternion &q);

        void param_reset();

        //param
        double theta;
        double dist;
        int num; //num of landmark
        double probs;
        double x_by_obj;
        double y_by_obj;

        double x_by_bench;
        double y_by_bench;
        double bench_prob;
        int bench_num;
        double x_by_fire;
        double y_by_fire;
        double fire_prob;
        double x_by_big;
        double y_by_big;
        double big_prob;
        double x_by_trash;
        double y_by_trash;
        double trash_prob;
        int trash_num;

        double roll;
        double pitch;
        double yaw;
        double yawyaw;

        bool map_checker = false;
        bool nya = false;

        //member
        ros::NodeHandle nh;
        ros::NodeHandle private_nh;

        ros::Subscriber sub_object;
        ros::Subscriber sub_map;
        ros::Subscriber sub_odometry;

        ros::Publisher pub_db_pose;

        geometry_msgs::PoseStamped db_pose;
        geometry_msgs::PoseStamped old_db_pose;
        nav_msgs::OccupancyGrid map;
        nav_msgs::Odometry old_odom;
        nav_msgs::Odometry current_odom;
        object_detector_msgs::ObjectPositions objects;

};

#endif
