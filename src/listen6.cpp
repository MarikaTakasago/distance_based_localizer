#include "distance_based_localizer/listen6.h"

Listen6::Listen6():private_nh("~"),nh("~")
{
    // param

    // sub&pub
    sub_roomba1 = nh.subscribe("/roomba1/score",100,&Listen6::roomba1_callback,this);
    sub_roomba2 = nh.subscribe("/roomba2/score",100,&Listen6::roomba2_callback,this);
    sub_roomba3 = nh.subscribe("/roomba3/score",100,&Listen6::roomba3_callback,this);
    sub_roomba4 = nh.subscribe("/roomba4/score",100,&Listen6::roomba4_callback,this);
    sub_roomba5 = nh.subscribe("/roomba5/score",100,&Listen6::roomba5_callback,this);
    sub_roomba6 = nh.subscribe("/roomba6/score",100,&Listen6::roomba6_callback,this);

    pub_roomba1 = nh.advertise<distance_based_localizer_msgs::RoombaScore>("/roomba1/fixed_score",100);
    pub_roomba2 = nh.advertise<distance_based_localizer_msgs::RoombaScore>("/roomba2/fixed_score",100);
    pub_roomba3 = nh.advertise<distance_based_localizer_msgs::RoombaScore>("/roomba3/fixed_score",100);
    pub_roomba4 = nh.advertise<distance_based_localizer_msgs::RoombaScore>("/roomba4/fixed_score",100);
    pub_roomba5 = nh.advertise<distance_based_localizer_msgs::RoombaScore>("/roomba5/fixed_score",100);
    pub_roomba6 = nh.advertise<distance_based_localizer_msgs::RoombaScore>("/roomba6/fixed_score",100);

}

void Listen6::roomba1_callback(const distance_based_localizer_msgs::RoombaScore::ConstPtr& msg)
{
    roomba1_score = roomba1_fixed = *msg;
    roomba1 = roomba1_score.score;
    roomba1_checker = true;
}

void Listen6::roomba2_callback(const distance_based_localizer_msgs::RoombaScore::ConstPtr& msg)
{
    roomba2_score = roomba2_fixed = *msg;
    roomba2 = roomba2_score.score;
    roomba2_checker = true;
}

void Listen6::roomba3_callback(const distance_based_localizer_msgs::RoombaScore::ConstPtr& msg)
{
    roomba3_score = roomba3_fixed = *msg;
    roomba3 = roomba3_score.score;
    roomba3_checker = true;
}

void Listen6::roomba4_callback(const distance_based_localizer_msgs::RoombaScore::ConstPtr& msg)
{
    roomba4_score = roomba4_fixed = *msg;
    roomba4 = roomba4_score.score;
    roomba4_checker = true;
}

void Listen6::roomba5_callback(const distance_based_localizer_msgs::RoombaScore::ConstPtr& msg)
{
    roomba5_score = roomba5_fixed = *msg;
    roomba5 = roomba5_score.score;
    roomba5_checker = true;
}

void Listen6::roomba6_callback(const distance_based_localizer_msgs::RoombaScore::ConstPtr& msg)
{
    roomba6_score = roomba6_fixed = *msg;
    roomba6 = roomba6_score.score;
    roomba6_checker = true;
}

void Listen6::make_sum()
{
    sum_score = 0.0;
    if(roomba1_checker) sum_score += roomba1;
    if(roomba2_checker) sum_score += roomba2;
    if(roomba3_checker) sum_score += roomba3;
    if(roomba4_checker) sum_score += roomba4;
    if(roomba5_checker) sum_score += roomba5;
    if(roomba6_checker) sum_score += roomba6;
}

void Listen6::normalize_score()
{
    make_sum();
    if(sum_score != 0)
    {
        if(roomba1_checker)
        {
            roomba1_fixed.score = roomba1/sum_score;
            roomba1_fixed.header.stamp = ros::Time::now();
            pub_roomba1.publish(roomba1_fixed);
        }
        if(roomba2_checker)
        {
            roomba2_fixed.score = roomba2/sum_score;
            roomba2_fixed.header.stamp = ros::Time::now();
            pub_roomba2.publish(roomba2_fixed);
        }
        if(roomba3_checker)
        {
            roomba3_fixed.score = roomba3/sum_score;
            roomba3_fixed.header.stamp = ros::Time::now();
            pub_roomba3.publish(roomba3_fixed);
        }
        if(roomba4_checker)
        {
            roomba4_fixed.score = roomba4/sum_score;
            roomba4_fixed.header.stamp = ros::Time::now();
            pub_roomba4.publish(roomba4_fixed);
        }
        if(roomba5_checker)
        {
            roomba5_fixed.score = roomba5/sum_score;
            roomba5_fixed.header.stamp = ros::Time::now();
            pub_roomba5.publish(roomba5_fixed);
        }
        if(roomba6_checker)
        {
            roomba6_fixed.score = roomba6/sum_score;
            roomba6_fixed.header.stamp = ros::Time::now();
            pub_roomba6.publish(roomba6_fixed);
        }
    }
    //scoreは一度配信されたらずっと配信され続ける想定なのでchecker=falseに戻してない(めんどくさかった)
}

void Listen6::process()
{
    ros::Rate rate(10);
    while(ros::ok())
    {
        normalize_score();
        ros::spinOnce();
        rate.sleep();
    }
}

int main(int argc , char** argv)
{
    ros::init(argc,argv,"listen6");
    Listen6 listen6;
    listen6.process();
    return 0;
}

