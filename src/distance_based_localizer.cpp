#include "distance_based_localizer/db_localizer.h"

DistanceBasedLocalizer::DistanceBasedLocalizer():private_nh("~")
{
    sub_object = nh.subscribe("/object_positions",100,&DistanceBasedLocalizer::object_callback,this);
    sub_map = nh.subscribe("/map",10,&DistanceBasedLocalizer::map_callback,this);
    sub_odometry = nh.subscribe("/roomba/odometry",100,&DistanceBasedLocalizer::odometry_callback,this);

    pub_db_pose = nh.advertise<geometry_msgs::PoseStamped>("/db_pose",100);
}

void DistanceBasedLocalizer::map_callback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
    map = *msg;
    map_checker = true;
    if(map_checker)
    {
        std::cout << "map!" << std::endl;
    }
    else
    {
        std::cout << "map?????" << std::endl;
    }
}

void DistanceBasedLocalizer::odometry_callback(const nav_msgs::Odometry::ConstPtr &msg)
{
   if(map_checker)
   {
        old_odom = current_odom;
        current_odom = *msg;
        db_pose.pose.orientation = current_odom.pose.pose.orientation;

        get_rpy(db_pose.pose.orientation,roll,pitch,yaw);
        yawyaw = yaw + M_PI/2;
        get_quat(roll,pitch,yawyaw,db_pose.pose.orientation);
   }
}

void DistanceBasedLocalizer::pose_callback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    old_db_pose = db_pose;
}

void DistanceBasedLocalizer::object_callback(const object_detector_msgs::ObjectPositions::ConstPtr& msg)
{
    objects = *msg;
    param_reset();

    for(auto object:objects.object_position)
    {
       std::cout << object.Class << std::endl;

       theta = object.theta + M_PI/2;
       dist = object.d;

       if(object.Class == "bench" && dist < 4.0 && old_db_pose.pose.position.x < -4.0)
       {
           x_by_bench = -6.0 + dist*cos(theta);
           y_by_bench = -13.75 + dist*sin(theta);
           bench_prob = object.probability;
           num += 1;
           probs += bench_prob;
           bench_num += 1;
       }

       if(object.Class == "bench" && dist < 6.5 && old_db_pose.pose.position.x > -3.0)
       {
           x_by_bench = 5.0 - dist*sin(theta);
           y_by_bench = -20.5 + dist*cos(theta);
           bench_prob = object.probability;
           num += 1;
           probs += bench_prob;
           bench_num += 1;
       }

       if(object.Class == "fire_hydrant" && dist < 7.0)
       {
           x_by_fire = -8.0 + dist*cos(theta);
           y_by_fire = -20.5 + dist*sin(theta);
           fire_prob = object.probability;
           num += 1;
           probs += fire_prob;
       }

       if(object.Class == "big_bench" && dist < 4.0)
       {
           x_by_big = -5.0 + dist*cos(theta);
           y_by_big = -19.0 + dist*sin(theta);
           big_prob = object.probability;
           num += 1;
           probs += big_prob;
       }

       if(object.Class == "trash_can" && dist < 4.0)
       {
           x_by_trash = 3.0 + dist*sin(theta);
           y_by_trash = -15.5 + dist*cos(theta);
           trash_prob = object.probability;
           num += 1;
           probs += trash_prob;
           trash_num += 1;
       }
    }

    x_by_obj = (bench_num*x_by_bench + x_by_fire + x_by_big + trash_num*x_by_trash)/num;
    y_by_obj = (bench_num*y_by_bench + y_by_fire + y_by_big + trash_num*y_by_trash)/num;

    db_pose.pose.position.x = x_by_obj;
    db_pose.pose.position.y = y_by_obj;

    db_pose.header.frame_id = "map";
    // db_pose.pose.position.x = 3.0;
    // db_pose.pose.position.y = -15.5;
    std::cout << num << std::endl;
    std::cout << "(X,Y)" << "(" << db_pose.pose.position.x << "," << db_pose.pose.position.y << ")" << std::endl;
    pub_db_pose.publish(db_pose);

}

void DistanceBasedLocalizer::get_rpy(const geometry_msgs::Quaternion &q, double &roll, double &pitch, double &yaw)
{
    tf::Quaternion qua(q.x,q.y,q.z,q.w);
    tf::Matrix3x3(qua).getRPY(roll,pitch,yaw);
}

void DistanceBasedLocalizer::get_quat(double roll, double pitch, double yaw, geometry_msgs::Quaternion &q)
{
    tf::Quaternion qua = tf::createQuaternionFromRPY(roll,pitch,yaw);
    quaternionTFToMsg(qua,q);
}

void DistanceBasedLocalizer::param_reset()
{
    num = 0;
    probs = 0;
    x_by_obj = y_by_obj = 0;
    x_by_bench = y_by_bench = bench_prob = bench_num = 0;
    x_by_fire = y_by_fire = fire_prob = 0;
    x_by_big = y_by_big = big_prob = 0;
    x_by_trash = y_by_trash = trash_prob = trash_num = 0;
}
void DistanceBasedLocalizer::process()
{
    ros::spin();
}

int main(int argc , char** argv)
{
    ros::init(argc,argv,"distance_based_localizer");
    DistanceBasedLocalizer db_localizer;
    db_localizer.process();
    return 0;
}

