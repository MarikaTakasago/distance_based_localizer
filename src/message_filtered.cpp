#include<ros/ros.h>
#include<message_filters/subscriber.h>
#include<message_filters/time_synchronizer.h>
#include<message_filters/sync_policies/approximate_time.h>
#include"distance_based_localizer_msgs/RoombaScore.h"


ros::Publisher pub1;
ros::Publisher pub2;
ros::Publisher pub3;
ros::Publisher pub4;
ros::Publisher pub5;
ros::Publisher pub6;

void callback(const distance_based_localizer_msgs::RoombaScore::ConstPtr& msg1,
              const distance_based_localizer_msgs::RoombaScore::ConstPtr& msg2,
              const distance_based_localizer_msgs::RoombaScore::ConstPtr& msg3,
              const distance_based_localizer_msgs::RoombaScore::ConstPtr& msg4,
              const distance_based_localizer_msgs::RoombaScore::ConstPtr& msg5,
              const distance_based_localizer_msgs::RoombaScore::ConstPtr& msg6)
{
    // ROS_INFO("get scores!");
    distance_based_localizer_msgs::RoombaScore fixed_msg1 = *msg1;
    distance_based_localizer_msgs::RoombaScore fixed_msg2 = *msg2;
    distance_based_localizer_msgs::RoombaScore fixed_msg3 = *msg3;
    distance_based_localizer_msgs::RoombaScore fixed_msg4 = *msg4;
    distance_based_localizer_msgs::RoombaScore fixed_msg5 = *msg5;
    distance_based_localizer_msgs::RoombaScore fixed_msg6 = *msg6;

    double score1 = fixed_msg1.score;
    double score2 = fixed_msg2.score;
    double score3 = fixed_msg3.score;
    double score4 = fixed_msg4.score;
    double score5 = fixed_msg5.score;
    double score6 = fixed_msg6.score;

    double sum = score1 + score2 + score3 + score4 + score5 + score6;
    fixed_msg1.score = score1 / sum;
    fixed_msg2.score = score2 / sum;
    fixed_msg3.score = score3 / sum;
    fixed_msg4.score = score4 / sum;
    fixed_msg5.score = score5 / sum;
    fixed_msg6.score = score6 / sum;

    pub1.publish(fixed_msg1);
    pub2.publish(fixed_msg2);
    pub3.publish(fixed_msg3);
    pub4.publish(fixed_msg4);
    pub5.publish(fixed_msg5);
    pub6.publish(fixed_msg6);

    if(sum<0) ROS_WARN("score1:%.2f score2:%.2f score3:%.2f score4:%.2f score5:%.2f score6:%.2f", score1, score2, score3, score4, score5, score6);
    ROS_INFO("score1:%.2f score2:%.2f score3:%.2f score4:%.2f score5:%.2f score6:%.2f", fixed_msg1.score, fixed_msg2.score, fixed_msg3.score, fixed_msg4.score, fixed_msg5.score, fixed_msg6.score);


}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "message_filtered");
    ros::NodeHandle nh_;

    pub1 = nh_.advertise<distance_based_localizer_msgs::RoombaScore>("/roomba1/fixed_score", 10);
    pub2 = nh_.advertise<distance_based_localizer_msgs::RoombaScore>("/roomba2/fixed_score", 10);
    pub3 = nh_.advertise<distance_based_localizer_msgs::RoombaScore>("/roomba3/fixed_score", 10);
    pub4 = nh_.advertise<distance_based_localizer_msgs::RoombaScore>("/roomba4/fixed_score", 10);
    pub5 = nh_.advertise<distance_based_localizer_msgs::RoombaScore>("/roomba5/fixed_score", 10);
    pub6 = nh_.advertise<distance_based_localizer_msgs::RoombaScore>("/roomba6/fixed_score", 10);

    message_filters::Subscriber<distance_based_localizer_msgs::RoombaScore> sub1(nh_, "roomba1/score", 1);
    message_filters::Subscriber<distance_based_localizer_msgs::RoombaScore> sub2(nh_, "roomba2/score", 1);
    message_filters::Subscriber<distance_based_localizer_msgs::RoombaScore> sub3(nh_, "roomba3/score", 1);
    message_filters::Subscriber<distance_based_localizer_msgs::RoombaScore> sub4(nh_, "roomba4/score", 1);
    message_filters::Subscriber<distance_based_localizer_msgs::RoombaScore> sub5(nh_, "roomba5/score", 1);
    message_filters::Subscriber<distance_based_localizer_msgs::RoombaScore> sub6(nh_, "roomba6/score", 1);

    typedef message_filters::sync_policies::ApproximateTime<distance_based_localizer_msgs::RoombaScore,
                                                            distance_based_localizer_msgs::RoombaScore,
                                                            distance_based_localizer_msgs::RoombaScore,
                                                            distance_based_localizer_msgs::RoombaScore,
                                                            distance_based_localizer_msgs::RoombaScore,
                                                            distance_based_localizer_msgs::RoombaScore> MySyncPolicy;
    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), sub1, sub2, sub3, sub4, sub5, sub6);
    sync.registerCallback(boost::bind(&callback, _1, _2,_3,_4,_5,_6));

    ros::spin();
    return 0;
}
