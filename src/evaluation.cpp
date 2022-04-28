#include "distance_based_localizer/evaluation.h"

Evaluation::Evaluation():private_nh("~")
{
    sub_mcl_pose = nh.subscribe("/roomba/mcl_pose",100,&Evaluation::mcl_pose_callback,this);
    sub_db_pose = nh.subscribe("/roomba/db_pose",100,&Evaluation::db_pose_callback,this);

    pub_evaluation = nh.advertise<evaluation_msgs::Difference>("/eva" ,100);

    private_nh.getParam("roomba_num",roomba_num);
}

void Evaluation::mcl_pose_callback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    mcl_pose = *msg;
    if(!isnan(mcl_pose.pose.position.x) || !isnan(mcl_pose.pose.position.y))
    {
        mcl_pose_checker = true;
        mcl_x = mcl_pose.pose.position.x;
        mcl_y = mcl_pose.pose.position.y;
        mcl_yaw = get_rpy(mcl_pose.pose.orientation);
    }
}

void Evaluation::db_pose_callback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    db_pose = *msg;
    if(!isnan(db_pose.pose.position.x) || !isnan(db_pose.pose.position.y))
    {
        db_pose_checker = true;
        db_x = db_pose.pose.position.x;
        db_y = db_pose.pose.position.y;
        db_yaw = get_rpy(db_pose.pose.orientation);
    }
}

double Evaluation::get_rpy(const geometry_msgs::Quaternion &q)
{
    double roll;
    double pitch;
    double yaw;
    tf::Quaternion qua(q.x,q.y,q.z,q.w);
    tf::Matrix3x3(qua).getRPY(roll,pitch,yaw);
    return yaw;
}

void Evaluation::squar_root()
{
    if(mcl_pose_checker && db_pose_checker)
    {
        dx = mcl_x - db_x;
        dy = mcl_y - db_y;
        dyaw = fabs(mcl_yaw - db_yaw);

        diff = sqrt(dx*dx + dy*dy);

        if(!isnan(diff) && !isnan(dyaw))
        {
            eva.header.stamp = ros::Time::now();
            eva.name = roomba_num;
            eva.dyaw = dyaw;
            eva.diff = diff;
            eva_checker = true;
        }
    }
}

void Evaluation::process()
{
    ros::Rate rate(10);
    while(ros::ok())
    {
        squar_root();
        if(eva_checker) pub_evaluation.publish(eva);

        ros::spinOnce();
        rate.sleep();
    }
}

int main(int argc , char** argv)
{
    ros::init(argc,argv,"evaluation");
    Evaluation eva;
    eva.process();
    return 0;
}
