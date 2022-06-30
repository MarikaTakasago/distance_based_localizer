#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>

void callback(const nav_msgs::Odometry::ConstPtr& msg1, const nav_msgs::Odometry::ConstPtr& msg2) {
  ROS_INFO("Callback: %f %f", msg1->pose.pose.position.x, msg2->pose.pose.position.x);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "otamesifilter");
  ros::NodeHandle nh;
  message_filters::Subscriber<nav_msgs::Odometry> sub1(nh, "/roomba1/roomba/odometry", 1);
  message_filters::Subscriber<nav_msgs::Odometry> sub2(nh, "/roomba2/roomba/odometry", 1);

    typedef message_filters::sync_policies::ApproximateTime<nav_msgs::Odometry, nav_msgs::Odometry> MySyncPolicy;
    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), sub1, sub2);
    sync.registerCallback(boost::bind(&callback, _1, _2));

    ros::spin();
    return 0;
}
