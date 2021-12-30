#include "distance_based_localizer/db_localizer_ver2.h"
std::random_device seed;
std::mt19937 engine(seed());
std::default_random_engine engine2(seed());

DistanceBasedLocalizer::DistanceBasedLocalizer():private_nh("~")
{
    sub_object = nh.subscribe("/object_positions",100,&DistanceBasedLocalizer::object_callback,this);
    sub_map = nh.subscribe("/map",100,&DistanceBasedLocalizer::map_callback,this);
    sub_odometry = nh.subscribe("/roomba/odometry",100,&DistanceBasedLocalizer::odometry_callback,this);
    sub_roomba_a = nh.subscribe("/roomba/score",100,&DistanceBasedLocalizer::roomba_callback_1,this);

    pub_db_pose = nh.advertise<geometry_msgs::PoseStamped>("/db_pose",100);
    pub_db_poses = nh.advertise<geometry_msgs::PoseArray>("/db_poses",100);
    pub_path = nh.advertise<nav_msgs::Path>("/path",100);
    pub_front_roomba_pose = nh.advertise<geometry_msgs::PoseStamped>("/front_roomba_pose",100);
    pub_score = nh.advertise<distance_based_localizer_msgs::RoombaScore>("/score",100);

    private_nh.getParam("particle_num",particle_num);
    private_nh.getParam("roomba_name",roomba_name);
    private_nh.getParam("num_s",num_s);
    private_nh.getParam("weight_s",weight_s);
    private_nh.getParam("x_init",x_init);
    private_nh.getParam("y_init",y_init);
    private_nh.getParam("yaw_init",yaw_init);
    private_nh.getParam("obj_weight",obj_weight);
    private_nh.getParam("bench",bench);
    private_nh.getParam("fire",fire);
    private_nh.getParam("big",big);
    private_nh.getParam("trash",trash);
    private_nh.getParam("odom",odom);
    private_nh.getParam("alpha_slow_th",alpha_slow_th);
    private_nh.getParam("alpha_fast_th",alpha_fast_th);
    private_nh.getParam("limit",limit);
    private_nh.getParam("x_cov_init",x_cov_init);
    private_nh.getParam("y_cov_init",y_cov_init);
    private_nh.getParam("yaw_cov_init",yaw_cov_init);
    private_nh.getParam("x_cov_1",x_cov_1);
    private_nh.getParam("y_cov_1",y_cov_1);
    private_nh.getParam("yaw_cov_1",yaw_cov_1);
    private_nh.getParam("x_cov_2",x_cov_2);
    private_nh.getParam("y_cov_2",y_cov_2);
    private_nh.getParam("yaw_cov_2",yaw_cov_2);
    private_nh.getParam("move_noise",move_noise);
    private_nh.getParam("estimated_weight_th",estimated_weight_th);
    private_nh.getParam("roomba_odom",roomba_odom);

    db_pose.header.frame_id = "map";
    // db_pose.pose.position.x = x_init;
    // db_pose.pose.position.y = y_init;
    // get_quat(yaw_init,db_pose.pose.orientation);

    db_poses.header.frame_id = "map";
    db_poses.poses.reserve(300);

    mini_path.header.frame_id = "map";
    roomba_path.header.frame_id = "map";

    p_array.reserve(300);

    landmark.reserve(30);
    per_prob.reserve(30);
}

void DistanceBasedLocalizer::map_callback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
    map = *msg;
    map_checker = true;
    if(map_checker)
    {
        std::cout << "map!" << std::endl;
        for(int i=0;i<300;++i)
        {
            Particle p = make_particle();
            p_array.push_back(p);
        }
    }
    // else std::cout << "map?????" << std::endl;
}

void DistanceBasedLocalizer::odometry_callback(const nav_msgs::Odometry::ConstPtr &msg)
{
   if(map_checker)
   {
        old_odom = current_odom;
        current_odom = *msg;
        if(!odom_checker) old_odom = current_odom;
        odom_checker = true;
        // std::cout << "odom!" << std::endl;
        // db_pose.pose.orientation = current_odom.pose.pose.orientation;

        // x_by_odom = -current_odom.pose.pose.position.y + x_init;
        // y_by_odom = current_odom.pose.pose.position.x + y_init;

        motion_update();
        // std::cout<<"cal"<<p_array.size()<<std::endl;
        yawyaw = get_rpy(current_odom.pose.pose.orientation) + M_PI/2;
   }
}
double DistanceBasedLocalizer::make_dyaw(double yawa,double yawi)
{
    double yawu = yawi -yawa;
    yawu = set_yaw(yawu);
    return yawu;
}
void DistanceBasedLocalizer::motion_update()
{
    double dx = current_odom.pose.pose.position.x - old_odom.pose.pose.position.x;
    double dy = current_odom.pose.pose.position.y - old_odom.pose.pose.position.y;
    double c_yaw = get_rpy(current_odom.pose.pose.orientation);
    double o_yaw = get_rpy(old_odom.pose.pose.orientation);
    double dyaw = make_dyaw(o_yaw,c_yaw);
    double dtrans = sqrt(dx*dx + dy*dy); //距離変化
    double drot1 = set_yaw(atan2(dy,dx) - o_yaw);
    double drot2 = set_yaw(dyaw - drot1);//角度変化
    // std::cout<<"o_yaw"<<dtrans<<std::endl;
    // std::cout<<dtrans<<","<<drot1<<","<<drot2<<std::endl;

    for(auto& p:p_array)
    {
        p.p_move(dtrans,drot1,drot2);
    }
    // std::cout<<"motion"<<std::endl;
    if(!objects_checker) //objectがないときにodomだけで頑張ってもらうパターン
    {
        only_odom += 1;
        calculate_pose_by_odom(only_odom);
        estimate_pose();
        calculate_weight(max_weight);
        // calculate_score();
    }
}
void DistanceBasedLocalizer::roomba_callback_1(const distance_based_localizer_msgs::RoombaScore::ConstPtr& msg)
{
    roomba_a_score = *msg;
}

void DistanceBasedLocalizer::object_callback(const object_detector_msgs::ObjectPositions::ConstPtr& msg)
{
    objects = *msg;
    for(auto o:objects.object_position)
    {
        if(o.Class == "bench" || o.Class == "fire_hydrant" || o.Class == "big_bench" || o.Class == "trash_can") objects_checker = true;
        if(objects_checker)
        {
            only_odom = 0;
            break;
        }
    }
    // std::cout << "obj!" << std::endl;
    if(objects_checker) observation_update();
}

double DistanceBasedLocalizer::get_rpy(const geometry_msgs::Quaternion &q)
{
    double roll;
    double pitch;
    double yaw;
    tf::Quaternion qua(q.x,q.y,q.z,q.w);
    tf::Matrix3x3(qua).getRPY(roll,pitch,yaw);
    return yaw;
}

void DistanceBasedLocalizer::get_quat(double yaw, geometry_msgs::Quaternion &q)
{
    tf::Quaternion qua = tf::createQuaternionFromYaw(yaw);
    quaternionTFToMsg(qua,q);
}

double DistanceBasedLocalizer::set_yaw(double yaw)
{
    if(yaw > M_PI) yaw -= 2*M_PI;
    if(yaw < -M_PI) yaw += 2*M_PI;
    return yaw;
}

double DistanceBasedLocalizer::make_gaussian(double mu,double sigma)
{
    std::normal_distribution<> dist(mu,sigma);
    return dist(engine);
}

void DistanceBasedLocalizer::param_reset()
{
    obj_num = 0;
    probs = 0;
    x_by_roomba1 = y_by_roomba1 = roomba1_prob = 0;
    for(int i=0;i<30;i++)
    {
        landmark[i].prob = 0;
    }
}

void DistanceBasedLocalizer::normalize_ps(std::vector<Particle> &p_array)
{
    double i = 0;
    for(auto& p:p_array) i += p.weight;
    for(auto& p:p_array) p.weight /= i;
}

void DistanceBasedLocalizer::normalize_number(std::vector<int> &nums)
{
    double sum  = 0;
    for(auto& num:nums) sum += num;
    for(auto& num:nums) num /= sum;
}

DistanceBasedLocalizer::Particle DistanceBasedLocalizer::make_particle()
{
    Particle p(this);
    return p;
}

DistanceBasedLocalizer::Particle::Particle(DistanceBasedLocalizer* db_localizer)
{
    dbl = db_localizer;
    p_pose.header.frame_id = "map";
    set_particle(dbl->x_init, dbl->y_init, dbl->yaw_init, dbl->x_cov_init, dbl->y_cov_init, dbl->yaw_cov_init);
    weight = 1.0/dbl->particle_num;
}

void DistanceBasedLocalizer::Particle::set_particle(double x,double y,double yaw,double sigma_x,double sigma_y,double sigma_yaw)
{
    p_pose.pose.position.x = dbl->make_gaussian(x,sigma_x);
    p_pose.pose.position.y = dbl->make_gaussian(y,sigma_y);
    double yyaw = dbl->make_gaussian(yaw,sigma_yaw);
    dbl->get_quat(dbl->set_yaw(yyaw),p_pose.pose.orientation);
}

void DistanceBasedLocalizer::Particle::p_move(double dtrans,double drot1,double drot2)
{
    dtrans += dbl->make_gaussian(0.0,dtrans*dbl->move_noise);
    drot1 += dbl->make_gaussian(0.0,drot1*dbl->move_noise);
    drot2 += dbl->make_gaussian(0.0,drot2*dbl->move_noise);

    double old_yaw = dbl->get_rpy(p_pose.pose.orientation);
    // std::cout << "old_yaw" <<old_yaw<<std::endl;
    p_pose.pose.position.x += dtrans * cos(dbl->set_yaw(old_yaw + drot1));
    p_pose.pose.position.y += dtrans * sin(dbl->set_yaw(old_yaw + drot1));
    dbl->get_quat(dbl->set_yaw(old_yaw + drot1 + drot2),p_pose.pose.orientation);
}

void DistanceBasedLocalizer::observation_update()
{
    int i=0;
    param_reset();
    for(auto& object:objects.object_position)
    {
       theta = yawyaw + object.theta;
       dist = object.d;

       if(object.Class == "bench" && dist < 4.0)
       {
           landmark[i].name = "bench";
           double gyaku = 1/dist;
           double sigma = 4.0*gyaku*0.05;
           landmark[i].sigma = sigma;

           if(bench_flag ==0)
           {
               landmark[i].x = -6.0 - dist*cos(theta);
               landmark[i].y = -13.75 - dist*sin(theta);
           }
           if(bench_flag == 1)
           {
               landmark[i].x = 5.0 - dist*cos(theta);
               landmark[i].y = -20.5 - dist*sin(theta);
           }

           landmark[i].prob = object.probability;
           obj_num += 1;
           probs += object.probability;
           bench_num += 1;
       }

       if(object.Class == "fire_hydrant" && dist < 5.0)
       {
           landmark[i].name = "fire";
           double gyaku = 1/dist;
           double sigma = 5.0*gyaku*0.05;
           landmark[i].sigma = sigma;

           // if(old_x < 10 && old_x > 0 && old_y > 0)
           if(fire_flag == 0)
           {
                landmark[i].x = 8.0 - dist*cos(theta);
                landmark[i].y = 8.8 - dist*sin(theta);
           }
           if(fire_flag == 1)
           {
                landmark[i].x = 7.5 - dist*cos(theta);
                landmark[i].y = 18.0 - dist*sin(theta);
           }
           if(fire_flag == 2)
           {
                landmark[i].x = -8.0 - dist*cos(theta);
                landmark[i].y = 9.0 - dist*sin(theta);
           }
           if(fire_flag == 3)
           {
                landmark[i].x = -8.0 - dist*cos(theta);
                landmark[i].y = -9.1 - dist*sin(theta);
           }
           if(fire_flag == 4)
           {
                landmark[i].x = -8.0 - dist*cos(theta);
                landmark[i].y = -20.5 - dist*sin(theta);
           }
           if(fire_flag == 5)
           {
                landmark[i].x = 8.1 - dist*cos(theta);
                landmark[i].y = -8.5 - dist*sin(theta);
           }
           landmark[i].prob = object.probability;
           obj_num += 1;
           probs += object.probability;

       }

       if(object.Class == "big_bench" && dist < 4.0)
       {
           landmark[i].name = "big";
           landmark[i].x = -5.0 - dist*cos(theta);
           landmark[i].y = -19.0 - dist*sin(theta);
           landmark[i].prob = object.probability;
           double gyaku = 1/dist;
           double sigma = 4.0*gyaku*0.05;
           landmark[i].sigma = sigma;
           obj_num += 1;
           probs += object.probability;

       }

       if(object.Class == "trash_can" && dist < 5.0)
       {
           landmark[i].name = "trash";
           // if(old_x > 0 && old_y < 0)
           double gyaku = 1/dist;
           double sigma = 5.0*gyaku*0.05;
           landmark[i].sigma = sigma;
           if(trash_flag == 0)
           {
                landmark[i].x = 5.2 - dist*cos(theta);
                landmark[i].y = 2.1 - dist*sin(theta);
           }
           if(trash_flag == 1)
           {
                landmark[i].x = 6.5 - dist*cos(theta);
                landmark[i].y = 17.5 - dist*sin(theta);
           }
           if(trash_flag == 2)
           {
                landmark[i].x = -4.0 - dist*cos(theta);
                landmark[i].y = 15.5 - dist*sin(theta);
           }
           if(trash_flag == 3)
           {
                landmark[i].x = 3.0 - dist*cos(theta);
                landmark[i].y = -15.5 - dist*sin(theta);
           }
           landmark[i].prob = object.probability;
           obj_num += 1;
           probs += object.probability;

           trash_num += 1;
      }

       if(object.Class == "roomba" && dist < 6.0 && dist > 0.0 && object.probability > 0.99)
       {
           // roomba_dist_x = dist*cos(theta);
           // roomba_dist_y = dist*sin(theta);
           roomba1_checker = true;
           roomba_dist_x = dist*cos(theta);
           roomba_dist_y = dist*sin(theta);

           double gyaku = 1/dist;
           double sigma = 6*gyaku*0.01;
           landmark[i].sigma = sigma;
           if(roomba_a_score.score > s)
           {
               // roomba1_checker = true;
               landmark[i].name = "roomba";
               landmark[i].x = roomba_a_score.pose.pose.position.x - roomba_dist_x;
               landmark[i].y = roomba_a_score.pose.pose.position.y - roomba_dist_y;
               landmark[i].prob = object.probability*roomba_a_score.score;
               obj_num += 1;
               probs += object.probability;
           }
       }

       i += 1;
    }

    calculate_pose_by_objects(i,probs);
    estimate_pose();
    calculate_weight(max_weight);
    // calculate_score();
    // std::cout << "observation" << std::endl;
}

void DistanceBasedLocalizer::calculate_pose_by_objects(int num,double probs)
{

    double dx = current_odom.pose.pose.position.x - old_odom.pose.pose.position.x;
    double dy = current_odom.pose.pose.position.y - old_odom.pose.pose.position.y;
    double dtrans = sqrt(dx * dx + dy * dy);
    //std::round()で整数に四捨五入
    // int per_obj = std::round(probs/obj_num*obj_weight*p_array.size());//particleの何割をobjからの情報にするかobj_weight=1でアキュラシーの平均値をそのまま使うことになる
    obj_weight = calculate_obj_weight(num,probs);
    double per_obj = obj_weight*p_array.size();
    // std::cout << "obj_weeeei:" << obj_weight  <<std::endl;
    // sort(landmark.begin(),landmark.end(),prob_sort();
    // std::cout << "num" <<num<<std::endl;
    for(int i=0;i<num;i++)
    {
        per_prob[i] = per_obj*(landmark[i].prob/probs);//landmark[i]を使う回数
        // std::cout<<"landmark["<<i<<"].prob"<<landmark[i].prob<<std::endl;
        // std::cout<<"probs"<<probs<<std::endl;
        // std::cout<<"per_prob["<<i<<"]"<<per_prob[i]<<std::endl;
    }

    int j = 0;
    int per = 0;
    bool landmark_checker = false;
    if(landmark[j].prob != 0) landmark_checker = true;

    // std::cout << "lm_checker" << landmark_checker  << std::endl;
    for(auto& p:p_array)
    {
        if(!landmark_checker)
        {
            p.weight = dtrans*odom;
            // std::cout<<"weight_odm"<<p.weight<<std::endl;
            // p.weight = p.weight * road_or_wall(p.p_pose.pose.position.x,p.p_pose.pose.position.y);
            // if(!isnan(p.p_pose.pose.position.x)) p.weight = dtrans*odom*road_or_wall(p.p_pose.pose.position.x,p.p_pose.pose.position.y);
        }

        else
        {
            p.p_pose.pose.position.x = make_gaussian(landmark[j].x,landmark[j].sigma);
            p.p_pose.pose.position.y = make_gaussian(landmark[j].y,landmark[j].sigma);
            double yaw_by_obj = make_gaussian(yawyaw,landmark[j].sigma*0.01);
            get_quat(yaw_by_obj,p.p_pose.pose.orientation);

            //weight
            if(landmark[j].name == "bench") p.weight =landmark[j].prob*bench;//bench = 他のweightとの桁数など調整用
            if(landmark[j].name == "fire") p.weight = landmark[j].prob*fire;//bench = 他のweightとの桁数など調整用
            if(landmark[j].name == "big") p.weight = landmark[j].prob*big;//bench = 他のweightとの桁数など調整用
            if(landmark[j].name == "trash") p.weight = landmark[j].prob*trash;//bench = 他のweightとの桁数など調整用
            if(landmark[j].name == "roomba") p.weight = landmark[j].prob*roomba_a_score.score;//bench = 他のweightとの桁数など調整用

            // std::cout<<"weight_obj"<<p.weight<<std::endl;
            //landmark[j]の使用回数
            per += 1;
            if(per >= per_prob[j])
            {
                // std::cout<<"weight_obj"<<p.weight<<std::endl;
                j += 1;
                per = 0;
                if(landmark[j].prob == 0) landmark_checker = false;
                // landmark_checker = std::isnan(landmark[j].prob);
                // std::cout << "lm_checker" << landmark_checker  << std::endl;
            }
            // p.weight = p.weight * road_or_wall(p.p_pose.pose.position.x,p.p_pose.pose.position.y);

        }
        double w = p.weight;
        if(!isnan(p.p_pose.pose.position.x))
        {
        p.weight = w * road_or_wall(p.p_pose.pose.position.x,p.p_pose.pose.position.y);
        }
    }

}
double DistanceBasedLocalizer::calculate_obj_weight(int num,double probs)
{
    double wei;
    double ave_prob = probs/num;
    wei = num*0.1*probs;
    if(wei>=1.0) wei = 0.5*wei;
    return wei;
}

bool DistanceBasedLocalizer::prob_sort(Objects& land,Objects& mark)
{
    return land.prob > mark.prob;
}

void DistanceBasedLocalizer::calculate_pose_by_odom(int only_odom)
{
    // std::cout<<"by_odom" << std::endl;
    double dx = current_odom.pose.pose.position.x - old_odom.pose.pose.position.x;
    double dy = current_odom.pose.pose.position.y - old_odom.pose.pose.position.y;
    double dtrans = sqrt(dx * dx + dy * dy);
    for(auto& p:p_array)
    {
        double w = dtrans*odom/(only_odom*10); //odom = 他との兼ね合い　わざと小さくなるように
        // p.weight = dtrans*odom; //odom = 他との兼ね合い　わざと小さくなるように
        // p.weight = p.weight * road_or_wall(p.p_pose.pose.position.x,p.p_pose.pose.position.y);
        if(!isnan(p.p_pose.pose.position.x)) p.weight = w*road_or_wall(p.p_pose.pose.position.x,p.p_pose.pose.position.y);
        // std::cout << "weo:" << p.weight << std::endl;
    }
}

void DistanceBasedLocalizer::estimate_pose()
{
    normalize_ps(p_array);
    double x = 0;
    double y = 0;
    double yaw = 0;
    max_weight = 0;
    double min_weight = 1;
    for(auto& p:p_array)
    {
        x += p.p_pose.pose.position.x * p.weight;
        y += p.p_pose.pose.position.y * p.weight;
        if(p.weight > max_weight)
        {
            max_weight = p.weight;
            yaw = get_rpy(p.p_pose.pose.orientation);
        }
        // if(p.weight < min_weight) min_weight = p.weight;
    }

    // std::cout << "min_weight:" << min_weight <<std::endl;
    db_pose.pose.position.x = x;
    db_pose.pose.position.y = y;
    // get_quat(yawyaw,db_pose.pose.orientation);
    get_quat(yaw,db_pose.pose.orientation);
    // std::cout<<"estimate"<<db_pose.pose.position.x<<std::endl;
}

void DistanceBasedLocalizer::calculate_weight(double estimated_weight)
{
    if(alpha_slow == 0) alpha_slow = alpha;
    else alpha_slow += alpha_slow_th * (alpha - alpha_slow);
    if(alpha_fast == 0) alpha_fast = alpha;
    else alpha_fast += alpha_fast_th * (alpha - alpha_fast);

    if(estimated_weight > estimated_weight_th || reset > limit)
    {
        reset = 0;
        adaptive_resampling();
    }
    else
    {
        reset += 1;
        expansion_reset();
    }
    std::cout <<"reset" << reset << std::endl;
}

void DistanceBasedLocalizer::adaptive_resampling()
{
    std::uniform_real_distribution<> random(0.0,1.0);
    double rand = random(engine2);
    int index = 0;
    std::vector<Particle> resampling_p_array;
    double a = 1 - (alpha_fast/alpha_slow);
    resampling_p_array.reserve(p_array.size());

    for(int i=0 ,size=p_array.size();i<size;++i)
    {
        rand += 1/particle_num;
        while(rand > p_array[index].weight)
        {
            rand -= p_array[index].weight;
            index = (index + 1) % particle_num;
        }
        if(random(engine2) > a)
        {
            Particle p = p_array[index];
            p.weight = 1.0/particle_num;
            resampling_p_array.push_back(p);
        }
        else
        {
            Particle p = p_array[index];
            p.weight = 1.0/particle_num;
            double yaw = get_rpy(db_pose.pose.orientation);

            p.set_particle(db_pose.pose.position.x,db_pose.pose.position.y,yaw,x_cov_1,y_cov_1,yaw_cov_1);
            resampling_p_array.push_back(p);
        }
    }
    p_array = resampling_p_array;
}

void DistanceBasedLocalizer::expansion_reset()
{
    for(auto& p:p_array)
   {
       double yaw = get_rpy(p.p_pose.pose.orientation);
       p.set_particle(p.p_pose.pose.position.x,p.p_pose.pose.position.y,yaw,x_cov_2,y_cov_2,yaw_cov_2);
   }
}

void DistanceBasedLocalizer::make_poses(std::vector<Particle> &p_array)
{
    db_poses.poses.clear();
    for(auto& p:p_array) db_poses.poses.push_back(p.p_pose.pose);
}

void DistanceBasedLocalizer::change_flags(geometry_msgs::PoseStamped &current_pose)
{
    current_x = current_pose.pose.position.x;
    current_y = current_pose.pose.position.y;
    //bench
    if((current_x < -6.0 && current_y > -14.0) || bench_flag == 0) bench_flag = 0;
    if((bench_flag <= 0 && current_x > -6.0 && current_y < -14.0) || bench_flag == 1) bench_flag = 1;

    //fire
    if((current_x > 0.0 && current_y > 0.0 && current_y < 8.8) || fire_flag == 0) fire_flag = 0;
    if((fire_flag <= 0 && current_x > 0.0 && current_y > 8.8) || fire_flag == 1) fire_flag = 1;
    if((fire_flag <= 1 && current_x < 0.0 && current_y > 9.2) || fire_flag == 2) fire_flag = 2;
    if((fire_flag <= 2 && current_x < 0.0 && current_y > -9.2 && current_y < 9.2) || fire_flag == 3) fire_flag = 3;
    if((fire_flag <= 3 && current_x < 0.0 && current_y < -9.2) || fire_flag == 4) fire_flag = 4;
    if((fire_flag <= 4 && current_x > 0.0 && current_y < 0.0) || fire_flag == 5) fire_flag = 5;

    //trash
    if((current_x > 5.0 && current_y < 2.3) || trash_flag == 0) trash_flag = 0;
    if((trash_flag <= 0 && current_x > 5.0 && current_y > 2.3 && current_y < 16.0) || trash_flag == 1) trash_flag = 1;
    if((trash_flag <= 1 && current_x < 6.3 && current_y > 16.0) || trash_flag == 2) trash_flag = 2;
    if((trash_flag <= 2 && current_x < -4.2 && current_y < 0.0) || trash_flag == 3) trash_flag = 3;
    // std::cout << "bench_flag:" << bench_flag << std::endl;
    // std::cout << "fire_flag:" << fire_flag << std::endl;
    // std::cout << "big_flag:" << big_flag << std::endl;
    // std::cout << "trash_flag:" << trash_flag << std::endl;
}

void DistanceBasedLocalizer::roomba_position()
{
    if(roomba1_checker)
    {
        front_roomba_pose.header.frame_id = "map";
        front_roomba_pose.pose.position.x = db_pose.pose.position.x + roomba_dist_x;
        front_roomba_pose.pose.position.y = db_pose.pose.position.y + roomba_dist_y;

        pub_front_roomba_pose.publish(front_roomba_pose);
    }
    roomba1_checker = false;
}

void DistanceBasedLocalizer::make_path(nav_msgs::Path &path)
{
    // std::cout << "make_path" << std::endl;
    // nav_msgs::Path new_path;
    // new_path.header.frame_id = "map";
    std::reverse(path.poses.begin(),path.poses.end());
    // path = new_path;
    int i = 0;
    // int j = 0;
    for(auto &pth : path.poses)
    {
        bool path_nan_checker = isnan(pth.pose.position.x);
        if(!path_nan_checker) i = 1;
        if(path_nan_checker) i = 0;
    }
    if(i == 1)
    {
        std::reverse(path.poses.begin(),path.poses.end());
        roomba_path.poses.insert(roomba_path.poses.end(),path.poses.begin(),path.poses.end());
        pub_path.publish(roomba_path);
        // std::cout << "path" << roomba_path.poses.size()<< std::endl;
    }
    mini_path.poses.clear();
}

void DistanceBasedLocalizer::calculate_score(int num,double max_weight,geometry_msgs::PoseStamped &current_pose)
{
    score.name = roomba_name;
    score.pose = current_pose;
    score.score = num_s*num + weight_s*max_weight + probs/num;
    s = score.score;
}
int DistanceBasedLocalizer::xy_map(double x,double y)
{
    int index_x = floor((x - map.info.origin.position.x)/map.info.resolution);
    int index_y = floor((y - map.info.origin.position.y)/map.info.resolution);
    return index_x + index_y*map.info.width;
}

double DistanceBasedLocalizer::road_or_wall(double x,double y)
{
    double wa = 1.0;
    int index = xy_map(x,y);
    if(map.data[index] == 100) wa = 0.001;
    else if(map.data[index] == -1) wa = 0;
    return wa;
}

void DistanceBasedLocalizer::process()
{
    tf::TransformBroadcaster odom_broadcaster;
    int path = 0;
    mini_path.poses.clear();

    ros::Rate rate(10);
    while(ros::ok())
    {
        if(map_checker && odom_checker)
        {
            try
            {
                double x_map_baselink = db_pose.pose.position.x;
                double y_map_baselink = db_pose.pose.position.y;
                double yaw_map_baselink = get_rpy(db_pose.pose.orientation);

                double x_odom_baselink = current_odom.pose.pose.position.x;
                double y_odom_baselink = current_odom.pose.pose.position.x;
                double yaw_odom_baselink = get_rpy(current_odom.pose.pose.orientation);

                double yaw_map_odom = make_dyaw(yaw_odom_baselink,yaw_map_baselink);
                double x_map_odom = x_map_baselink - x_odom_baselink*cos(yaw_map_odom) + y_odom_baselink*sin(yaw_map_odom);
                double y_map_odom = y_map_baselink + y_odom_baselink*cos(yaw_map_odom) - x_odom_baselink*sin(yaw_map_odom);

                geometry_msgs::TransformStamped odom;
                odom.header.stamp = ros::Time::now();

                odom.header.frame_id = "map";
                // odom.child_frame_id = "odom";
                odom.child_frame_id = roomba_odom;

                odom.transform.translation.x = x_map_odom;
                odom.transform.translation.y = y_map_odom;
                get_quat(yaw_map_odom,odom.transform.rotation);

                odom_broadcaster.sendTransform(odom);
            }
            catch(tf::TransformException &ex)
            {
                ROS_ERROR("%s",ex.what());
            }


            make_poses(p_array);
            if(!isnan(db_poses.poses[0].position.x) || !isnan(db_poses.poses[0].position.y)) pub_db_poses.publish(db_poses);
            if(!isnan(db_pose.pose.position.x) || !isnan(db_pose.pose.position.y)) pub_db_pose.publish(db_pose);
            calculate_score(obj_num,max_weight,db_pose);
            pub_score.publish(score);

            objects_checker = false;

            change_flags(db_pose);
            mini_path.poses.push_back(db_pose);
            make_path(mini_path);

            roomba_position();
            path += 1;
        }
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

