#ifndef MESSAGEFILTERED_H
#define MESSAGEFILTERED_H

#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include "distance_based_localizer_msgs/RoombaScore.h"
#include <nav_msgs/Odometry.h>

class MessageFiltered
{
    public:
        MessageFiltered();
    private:
        //class

        //method
        void score_callback(const distance_based_localizer_msgs::RoombaScore::ConstPtr &msg1 ,
                            const distance_based_localizer_msgs::RoombaScore::ConstPtr &msg2 );//,
                            // const distance_based_localizer_msgs::RoombaScore::ConstPtr &msg3 ,
                            // const distance_based_localizer_msgs::RoombaScore::ConstPtr &msg4 ,
                            // const distance_based_localizer_msgs::RoombaScore::ConstPtr &msg5 ,
                            // const distance_based_localizer_msgs::RoombaScore::ConstPtr &msg6);
        void tst_callback(const nav_msgs::Odometry::ConstPtr &msg1,const nav_msgs::Odometry::ConstPtr &msg2);
        void normalize_score();

        //param
        int robot_num_ = 6;
        double scores_[6] = {0};
        double roomba1_ = 0;
        double roomba2_ = 0;
        double roomba3_ = 0;
        double roomba4_ = 0;
        double roomba5_ = 0;
        double roomba6_ = 0;


        //member

        ros::NodeHandle nh;
        ros::NodeHandle private_nh;

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

        //message_filters
        // typedef message_filters::sync_policies::ApproximateTime<
        //     distance_based_localizer_msgs::RoombaScore,
        //     distance_based_localizer_msgs::RoombaScore>//,
        //     MySyncPolicy;
        //                                                 distance_based_localizer_msgs::RoombaScore,
        //                                                 distance_based_localizer_msgs::RoombaScore,
        //                                                 distance_based_localizer_msgs::RoombaScore,
        //                                                 distance_based_localizer_msgs::RoombaScore>
        // MySyncPolicy;

        typedef message_filters::sync_policies::ApproximateTime<nav_msgs::Odometry , nav_msgs::Odometry> MySyncPolicy;
        message_filters::Subscriber<nav_msgs::Odometry> roomba1_subo;
        message_filters::Subscriber<nav_msgs::Odometry> roomba2_subo;
        // message_filters::Subscriber<distance_based_localizer_msgs::RoombaScore> roomba1_sub;
        // message_filters::Subscriber<distance_based_localizer_msgs::RoombaScore> roomba2_sub;
        // message_filters::Subscriber<distance_based_localizer_msgs::RoombaScore> roomba3_sub(nh,"roomba3/score",10);
        // message_filters::Subscriber<distance_based_localizer_msgs::RoombaScore> roomba4_sub(nh,"roomba4/score",10);
        // message_filters::Subscriber<distance_based_localizer_msgs::RoombaScore> roomba5_sub(nh,"roomba5/score",10);
        // message_filters::Subscriber<distance_based_localizer_msgs::RoombaScore> roomba6_sub(nh,"roomba6/score",10);
        // message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10),roomba1_sub,roomba2_sub,roomba3_sub,roomba4_sub,roomba5_sub,roomba6_sub);
        message_filters::Synchronizer<MySyncPolicy> sync;

};

#endif
