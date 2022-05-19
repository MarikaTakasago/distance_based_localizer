#ifndef LISTEN6_H
#define LISTEN6_H

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <std_msgs/Header.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <vector>
#include <cmath>
#include <random>
#include <string>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include "distance_based_localizer_msgs/RoombaScore.h"

class Listen6
{
    public:
        Listen6();
        void process();

    private:
        //class

        //method
        void map_callback(const nav_msgs::OccupancyGrid::ConstPtr &msg);
        void roomba1_callback(const distance_based_localizer_msgs::RoombaScore::ConstPtr &msg);
        void roomba2_callback(const distance_based_localizer_msgs::RoombaScore::ConstPtr &msg);
        void roomba3_callback(const distance_based_localizer_msgs::RoombaScore::ConstPtr &msg);
        void roomba4_callback(const distance_based_localizer_msgs::RoombaScore::ConstPtr &msg);
        void roomba5_callback(const distance_based_localizer_msgs::RoombaScore::ConstPtr &msg);
        void roomba6_callback(const distance_based_localizer_msgs::RoombaScore::ConstPtr &msg);
        void make_sum();
        void normalize_score();

        //param
        double sum_score;
        double roomba1;
        double roomba2;
        double roomba3;
        double roomba4;
        double roomba5;
        double roomba6;

        bool roomba1_checker = false;
        bool roomba2_checker = false;
        bool roomba3_checker = false;
        bool roomba4_checker = false;
        bool roomba5_checker = false;
        bool roomba6_checker = false;

        //member
        ros::NodeHandle nh;
        ros::NodeHandle private_nh;

        ros::Subscriber sub_roomba1;
        ros::Subscriber sub_roomba2;
        ros::Subscriber sub_roomba3;
        ros::Subscriber sub_roomba4;
        ros::Subscriber sub_roomba5;
        ros::Subscriber sub_roomba6;

        ros::Publisher pub_roomba1;
        ros::Publisher pub_roomba2;
        ros::Publisher pub_roomba3;
        ros::Publisher pub_roomba4;
        ros::Publisher pub_roomba5;
        ros::Publisher pub_roomba6;

        distance_based_localizer_msgs::RoombaScore roomba1_score;
        distance_based_localizer_msgs::RoombaScore roomba2_score;
        distance_based_localizer_msgs::RoombaScore roomba3_score;
        distance_based_localizer_msgs::RoombaScore roomba4_score;
        distance_based_localizer_msgs::RoombaScore roomba5_score;
        distance_based_localizer_msgs::RoombaScore roomba6_score;

        distance_based_localizer_msgs::RoombaScore roomba1_fixed;
        distance_based_localizer_msgs::RoombaScore roomba2_fixed;
        distance_based_localizer_msgs::RoombaScore roomba3_fixed;
        distance_based_localizer_msgs::RoombaScore roomba4_fixed;
        distance_based_localizer_msgs::RoombaScore roomba5_fixed;
        distance_based_localizer_msgs::RoombaScore roomba6_fixed;

};

#endif
