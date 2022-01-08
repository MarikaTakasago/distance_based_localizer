#ifndef EVALUATION_H
#define EVALUATION_H

#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <geometry_msgs/PoseStamped.h>
#include <cmath>
#include <tf/tf.h>
#include "evaluation_msgs/Difference.h"


class Evaluation
{
    public:
        Evaluation();
        void process();

    private:
        //method
        void mcl_pose_callback(const geometry_msgs::PoseStamped::ConstPtr& msg);
        void db_pose_callback(const geometry_msgs::PoseStamped::ConstPtr& msg);

        double get_rpy(const geometry_msgs::Quaternion &q);

        void squar_root();

        //param
        double mcl_x;
        double mcl_y;
        double mcl_yaw;
        double db_x;
        double db_y;
        double db_yaw;

        int roomba_num;
        double dx;
        double dy;
        double dyaw;
        double diff;

        bool mcl_pose_checker = false;
        bool db_pose_checker = false;
        bool eva_checker = false;

        //member
        ros::NodeHandle nh;
        ros::NodeHandle private_nh;

        ros::Subscriber sub_mcl_pose;
        ros::Subscriber sub_db_pose;

        ros::Publisher pub_evaluation;

        geometry_msgs::PoseStamped mcl_pose;
        geometry_msgs::PoseStamped db_pose;

        evaluation_msgs::Difference eva;
};

#endif

