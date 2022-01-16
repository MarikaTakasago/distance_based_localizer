#ifndef DB_LOCALIZER_H
#define DB_LOCALIZER_H

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PointStamped.h>
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
#include "object_detector_msgs/ObjectPosition.h"
#include "object_detector_msgs/ObjectPositions.h"
#include "distance_based_localizer_msgs/RoombaScore.h"

class DistanceBasedLocalizer
{
    public:
        DistanceBasedLocalizer();
        void process();

    private:
        //class
        class Particle
        {
            public:
                Particle(DistanceBasedLocalizer* db_localizer);
                geometry_msgs::PoseStamped p_pose;
                double weight;
                void set_particle(double x,double y,double yaw,double sigma_x,double sigma_y,double sigma_yaw);
                void p_move(double dtrans,double drot1,double drot2);
            private:
                DistanceBasedLocalizer* dbl;
        };

        class Objects
        {
            public:
                std::string name;
                int num;
                double prob;
                double x;
                double y;
                double yaw;
                double sigma;
                int flag = 0;
        };

        //method
        void object_callback(const object_detector_msgs::ObjectPositions::ConstPtr &msg);
        void map_callback(const nav_msgs::OccupancyGrid::ConstPtr &msg);
        void odometry_callback(const nav_msgs::Odometry::ConstPtr &msg);
        void roomba_callback_1(const distance_based_localizer_msgs::RoombaScore::ConstPtr &msg);
        void roomba_callback_2(const distance_based_localizer_msgs::RoombaScore::ConstPtr &msg);
        void kf(double pre_x,double pre_y,double cur_x,double cur_y,int sum_num);
        double get_rpy(const geometry_msgs::Quaternion &q);
        void get_quat(double yaw,geometry_msgs::Quaternion &q);
        double set_yaw(double yaw);
        double make_gaussian(double mu,double sigma);
        void param_reset();
        void normalize_ps(std::vector<Particle> &p_array);
        void normalize_number(std::vector<int> &nums);

        Particle make_particle();
        void motion_update();
        void observation_update();
        void calculate_pose_by_odom(int only_odom);
        void calculate_pose_by_objects(int num,double probs);
        double calculate_obj_weight(int num,double probs);
        bool prob_sort(Objects& land,Objects& mark);
        void expansion_resetting(double x,double y,double wa);
        void estimate_pose();
        void calculate_weight(double estimated_weight);
        void adaptive_resampling();
        void expansion_reset();
        void make_poses(std::vector<Particle> &p_array);
        void change_flags(geometry_msgs::PoseStamped &current_pose);

        void roomba_position();
        void make_path(nav_msgs::Path &path);
        void calculate_score(int num,double max_weight,geometry_msgs::PoseStamped &current_pose);
        int xy_map(double x,double y);
        double road_or_wall(double x,double y);
        double dist_from_wall(double x,double y,double yaw);
        double calculate_delta(geometry_msgs::PoseStamped pose,double x,double y);

        double make_dyaw(double yawa,double yawi);

        //param

        int particle_num;

        //score
        int roomba_name;
        double num_s;
        double weight_s;
        double s;
        double probs_for_score;

        //odom
        double x_old;
        double y_old;
        double yaw_old;
        double x_by_odom;
        double y_by_odom;
        double x_init;
        double y_init;
        double yaw_init;
        double dx;
        double dy;
        double dyaw;
        double dtrans;
        int only_odom;

        //object
        double theta;
        double dist;
        int obj_num; //num of landmark
        double probs;
        double x_by_obj;
        double y_by_obj;
        double roomba_dist_x;
        double roomba_dist_y;
        double roomba_dist;
        int roomba_num;
        double front_th; //以上だったらそのルンバの位置使う

        Objects Bench;
        Objects Fire;
        Objects Big;
        Objects Trash;

        double x_by_bench;
        double y_by_bench;
        double bench_prob;
        int bench_num;
        double x_by_fire;
        double y_by_fire;
        double fire_prob;
        double x_by_firee;
        double y_by_firee;
        double firee_prob;
        double x_by_big;
        double y_by_big;
        double big_prob;
        double x_by_trash;
        double y_by_trash;
        double trash_prob;
        int trash_num;

        double bench;
        double fire;
        double big;
        double trash;
        double odom;
        double obj_weight;

        int bench_flag = 0;
        int fire_flag = 0;
        int big_flag = 0;
        int trash_flag = 0;

        double x_by_roomba1;
        double y_by_roomba1;
        double roomba1_prob;

        double yaw;
        double yawyaw;
        double weights_max;

        //calc_weight
        double alpha = 0;
        double alpha_slow;
        double alpha_fast;
        double alpha_slow_th;
        double alpha_fast_th;
        double estimated_weight_th;
        double max_weight;
        int reset = 0;
        int limit;

        double move_noise;

        //covariance
        double x_cov_init;
        double y_cov_init;
        double yaw_cov_init;
        double x_cov_1;
        double y_cov_1;
        double yaw_cov_1;
        double x_cov_2;
        double y_cov_2;
        double yaw_cov_2;

        double current_x;
        double current_y;

        //behind_info
        double behind_x = 0;
        double behind_y = 0;
        double behind_score = 0;
        double behind_yaw;
        double behind_th; //以下になったら後ろのルンバの位置もらう

        std::string roomba_odom;

        bool map_checker = false;
        int nya = 0;
        bool objects_checker = false;
        bool odom_checker = false;
        bool roomba1_checker = false;
        bool behind_roomba_checker = false;
        bool is_move = false;
        bool is_only_odom = false; //比較用


        //member
        ros::NodeHandle nh;
        ros::NodeHandle private_nh;

        ros::Subscriber sub_object;
        ros::Subscriber sub_map;
        ros::Subscriber sub_odometry;
        ros::Subscriber sub_roomba_a;
        ros::Subscriber sub_roomba_b;

        ros::Publisher pub_db_pose;
        ros::Publisher pub_db_poses;
        ros::Publisher pub_front_roomba_pose;
        ros::Publisher pub_path;
        ros::Publisher pub_score;

        geometry_msgs::PoseStamped db_pose;
        geometry_msgs::PoseStamped old_pose;
        geometry_msgs::PoseArray db_poses;
        geometry_msgs::PointStamped front_roomba_pose;
        // geometry_msgs::PoseStamped roomba1_pose;

        nav_msgs::OccupancyGrid map;
        nav_msgs::Odometry current_odom;
        nav_msgs::Odometry old_odom;
        nav_msgs::Path roomba_path;
        nav_msgs::Path mini_path;

        object_detector_msgs::ObjectPositions objects;

        distance_based_localizer_msgs::RoombaScore score;
        distance_based_localizer_msgs::RoombaScore roomba_a_score;
        distance_based_localizer_msgs::RoombaScore roomba_b_score;

        std::vector<Particle> p_array;
        std::vector<Objects> landmark;
        std::vector<double> per_prob;

};

#endif
