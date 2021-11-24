#include "distance_based_localizer/db_localizer_ver2.h"

DistanceBasedLocalizer::DistanceBasedLocalizer():private_nh("~")
{
    sub_object = nh.subscribe("/object_positions",100,&DistanceBasedLocalizer::object_callback,this);
    sub_map = nh.subscribe("/map",10,&DistanceBasedLocalizer::map_callback,this);
    sub_odometry = nh.subscribe("/roomba/odometry",100,&DistanceBasedLocalizer::odometry_callback,this);

    pub_db_pose = nh.advertise<geometry_msgs::PoseStamped>("/db_pose",100);
    pub_front_roomba_pose = nh.advertise<geometry_msgs::PoseStamped>("/front_roomba_pose",100);
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
        odom_checker = true;
        db_pose.pose.orientation = current_odom.pose.pose.orientation;

        get_rpy(db_pose.pose.orientation,roll,pitch,yaw);
        yawyaw = yaw + M_PI/2;
        get_quat(roll,pitch,yawyaw,db_pose.pose.orientation);

        x_by_odom = -current_odom.pose.pose.position.y + 7.0;
        y_by_odom = current_odom.pose.pose.position.x + 1.0;

   }
}

void DistanceBasedLocalizer::pose_callback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    // old_db_pose = db_pose;
    // old_x = old_db_pose.pose.position.x;
    // old_y = old_db_pose.pose.position.y;

}

void DistanceBasedLocalizer::object_callback(const object_detector_msgs::ObjectPositions::ConstPtr& msg)
{
    objects = *msg;
    param_reset();

    for(auto object:objects.object_position)
    {
       theta = yawyaw + object.theta;
       dist = object.d;

       // std::cout << "theta:" << theta << std::endl;
       // std::cout << "yaw:" << yawyaw << std::endl;
       // std::cout << "obj_theta:" << object.theta << std::endl;

       if(object.Class == "bench" && dist < 4.0)
       {
           if(nya == 4 || nya == 5)
           {
               x_by_bench = -6.0 - dist*cos(theta);
               y_by_bench = -13.75 - dist*sin(theta);
               bench_prob = object.probability;
               num += 1;
               probs += bench_prob;
               bench_num += 1;
           }
           if(nya == 6)
           {
               x_by_bench = 5.0 - dist*cos(theta);
               y_by_bench = -20.5 - dist*sin(theta);
               bench_prob = object.probability;
               num += 1;
               probs += bench_prob;
               bench_num += 1;
           }
       }

       if(object.Class == "fire_hydrant" && dist < 5.0)
       {
           // if(old_x < 10 && old_x > 0 && old_y > 0)
           if(nya == 0)
           {
                x_by_fire = 8.0 - dist*cos(theta);
                y_by_fire = 8.8 - dist*sin(theta);
           }
           if(nya == 1)
           {
                x_by_fire = 7.5 - dist*cos(theta);
                y_by_fire = 18.0 - dist*sin(theta);
           }
           if(nya == 3)
           {
                x_by_fire = -8.0 - dist*cos(theta);
                y_by_fire = 9.0 - dist*sin(theta);
           }
           if(nya == 4)
           {
                x_by_fire = -8.0 - dist*cos(theta);
                y_by_fire = -9.1 - dist*sin(theta);
           }
           if(nya == 5 && nya == 6 && dist < 4.0)
           {
                x_by_fire = -8.0 - dist*cos(theta);
                y_by_fire = -20.5 - dist*sin(theta);
           }
           if(nya == 7)
           {
                x_by_fire = 8.1 - dist*cos(theta);
                y_by_fire = -8.5 - dist*sin(theta);
           }
                fire_prob = object.probability;
                num += 1;
                probs += fire_prob;
       }

       if(object.Class == "big_bench" && dist < 4.0)
       {
           x_by_big = -5.0 - dist*cos(theta);
           y_by_big = -19.0 - dist*sin(theta);
           big_prob = object.probability;
           num += 1;
           probs += big_prob;
       }

        if(object.Class == "fire_extinguisher" && dist < 4.0)
        {
            // x_by_firee = 4.9 - dist*cos(theta);
            // y_by_firee = -15.3 - dist*sin(theta);
            // firee_prob = object.probability;
            // num += 1;
            // probs += firee_prob;
            // FIRE //

         }

       if(object.Class == "trash_can" && dist < 7.0)
       {
           // if(old_x > 0 && old_y < 0)
           if(nya == 6)
           {
                x_by_trash = 3.0 - dist*cos(theta);
                y_by_trash = -15.5 - dist*sin(theta);
           }
           // else if(old_x > 0 && old_y > 0 && old_y < 3.0)
           if(nya == 0 || nya == 7)
           {
                x_by_trash = 5.2 - dist*cos(theta);
                y_by_trash = 2.1 - dist*sin(theta);
           }

           // else if(old_x > 3.0 && old_y > 3.0)
           if(nya == 1)
           {
                x_by_trash = 6.5 - dist*cos(theta);
                y_by_trash = 17.5 - dist*sin(theta);
           }
           if(nya == 2)
           {
                x_by_trash = -4.0 - dist*cos(theta);
                y_by_trash = 15.5 - dist*sin(theta);
           }
                trash_prob = object.probability;
                num += 1;
                probs += trash_prob;
                trash_num += 1;
       }

       if(object.Class == "roomba" && dist < 5.0)
       {
           roomba_dist_x = dist*cos(theta);
           roomba_dist_y = dist*sin(theta);
       }

    }

    x_by_obj = (bench_num*x_by_bench + x_by_fire + x_by_big + trash_num*x_by_trash + x_by_firee)/num;
    y_by_obj = (bench_num*y_by_bench + y_by_fire + y_by_big + trash_num*y_by_trash + x_by_firee)/num;

    // std::cout << "x_by_obj" << x_by_obj << std::endl;
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
    x_by_firee = y_by_firee = firee_prob = 0;
    x_by_big = y_by_big = big_prob = 0;
    x_by_trash = y_by_trash = trash_prob = trash_num = 0;
}

void DistanceBasedLocalizer::estimate_pose()
{
    db_pose.header.frame_id = "map";
    obj_checker = std::isnan(x_by_obj); //true = nan

    if(!obj_checker && odom_checker) //obj ok & odom
    {
        current_x = (x_by_odom + num*x_by_obj)/(num+1);
        current_y = (y_by_odom + num*y_by_obj)/(num+1);
        // std::cout << "1" << std::endl;
    }
    if(obj_checker && odom_checker) //no obj & odom
    {
        current_x = x_by_odom;
        current_y = y_by_odom;
        // std::cout << "2" << std::endl;
    }
    if(!odom_checker && !obj_checker) //no odom & obj ok
    {
        current_x = x_by_obj;
        current_y = y_by_obj;
        // std::cout << "3" << std::endl;
    }

    db_pose.pose.position.x = current_x;
    db_pose.pose.position.y = current_y;
    // db_pose.pose.position.x = 8.1;
    // db_pose.pose.position.y = -8.5;
    pub_db_pose.publish(db_pose);

    // std::cout << "(X,Y)" << "(" << db_pose.pose.position.x << "," << db_pose.pose.position.y << ")" << std::endl;

    int nyaa = nya;

    if(current_x > 6.0 && current_x < 8.5 && current_y > 0.0 && current_y < 8.0|| nya == 0) nya = 0;
    if(nya <= 0  && current_x > 6.0 && current_x < 8.5 && current_y > 8.0 || nya == 1) nya = 1;
    if((nya <= 1 && current_x > -6.0 && current_x < 6.0 && current_y > 0.0) || nya ==2) nya = 2;
    if((nya <= 2 && current_x > -8.5 && current_x < -6.0 && current_y > 5.0) || nya ==3) nya = 3;
    if((nya <= 3 && current_x > -8.5 && current_x < -6.0 && current_y < 5.0 && current_y > -10.0) || nya ==4) nya = 4;
    if((nya <= 4 && current_x > -8.5 && current_x < -6.0 && current_y < -10.0 && current_y > -15.5) || nya ==5) nya = 5;
    if((nya <= 5 && current_x > -8.5 && current_x < 4.0 && current_y < -15.5) || nya ==6) nya = 6;
    if((nya <= 6 && current_x > 4.0 && current_x < 8.5 && current_y < 0.0) || nya ==7) nya = 7;

    if(nya != nyaa) std::cout << "nya" << nya << std::endl;

}

void DistanceBasedLocalizer::roomba_position()
{
    front_roomba_pose.header.frame_id = "map";
    front_roomba_pose.pose.position.x = db_pose.pose.position.x + roomba_dist_x;
    front_roomba_pose.pose.position.y = db_pose.pose.position.y + roomba_dist_y;

    pub_front_roomba_pose.publish(front_roomba_pose);
}
void DistanceBasedLocalizer::process()
{
    ros::Rate rate(10);
    while(ros::ok())
    {
        estimate_pose();
        roomba_position();

        ros::spinOnce();
        rate.sleep();
    }
}

int main(int argc , char** argv)
{
    ros::init(argc,argv,"distance_based_localizer");
    DistanceBasedLocalizer db_localizer;
    db_localizer.process();
    return 0;
}

