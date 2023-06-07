#pragma once
#include "mpc/trajectory_.hpp"
using namespace casadi;

#define bezier_ 1
#define scurve_ 2
#define INF 1e4
#define cm_m 100

inline bool comparePolygon(Polygon p1, Polygon p2);
class MPC {
    private:
        ros::Subscriber path_sub;
        ros::Subscriber x_pose_sub;
        ros::Subscriber y_pose_sub;
        ros::Subscriber theta_pose_sub;
        ros::Subscriber obs_sub;

        ros::Publisher vel1_pub;
        ros::Publisher vel2_pub;
        ros::Publisher vel3_pub;

        ros::Publisher tra_pub;
        ros::Publisher path_predict_pub;
        // ros::Publisher pose_pub;

        bool path_;
        bool lzpose_;
        bool mode1, mode2;
        bool polygon_, control_, trajectory_;

        int trajectory_type;
        string map_frame;

        double T_sample;
        int N_predictsize;
        int N_predictcontrol;
        double e_safety;
        int N_maxpolygon;

        void pathCallback(const nav_msgs::Path& msg);
        void x_poseCallback(const std_msgs::Float64& msg);
        void y_poseCallback(const std_msgs::Float64& msg);
        void theta_poseCallback(const std_msgs::Float64& msg);

        vector<Polygon> polygons;

        void polygonCallback(const visualization_msgs::MarkerArray& msg);
        double norm_2(Point q1, Point q2);
        Polygon polygon_centroid(Polygon p);
        void trajectory_publisher();
        void initial_optimal();
        void optimal_solution();
        void mpc_publisher();

        double publish_frequency;

        int max_iter;
        SX X, V, P;
        SX obj, cst_states, cst_states_collision;
        SXDict nlp;
        Dict opts;

        double kernel_size;
        double max_dist;

        DM lbx, ubx, lbg_mode1, ubg_mode1, lbg_mode2, ubg_mode2;
        DMDict args;
        DM u0, p, x0;

        void objective_func();
        void cst_func();
    public:
        Trajectory ref;
        ros::Time current_time = ros::Time::now();
        ros::Time last_time = ros::Time::now();
        double t;

        std_msgs::Float64 v1, v2, v3;
        int count = 0;
        MPC(ros::NodeHandle* nh) {
            path_sub = nh->subscribe("global_path", 10, &MPC::pathCallback, this);
            x_pose_sub = nh->subscribe("x_position", 10, &MPC::x_poseCallback, this);
            y_pose_sub = nh->subscribe("y_position", 10, &MPC::y_poseCallback, this);
            theta_pose_sub = nh->subscribe("angle", 10, &MPC::theta_poseCallback, this);
            obs_sub = nh->subscribe("visualization_marker",10, &MPC::polygonCallback, this);

            vel1_pub = nh->advertise<std_msgs::Float64>("/robot_kist/joint_1_velocity/command", 10);
            vel2_pub = nh->advertise<std_msgs::Float64>("/robot_kist/joint_2_velocity/command", 10);
            vel3_pub = nh->advertise<std_msgs::Float64>("/robot_kist/joint_3_velocity/command", 10);
    
            tra_pub = nh->advertise<nav_msgs::Path>("path_predict", 10);
            path_predict_pub = nh->advertise<nav_msgs::Path>("quintic_bezier", 10);
            // pose_pub = nh->advertise<geometry_msgs::PoseStamped>("goal", 10);

            if(!ros::param::get("~publish_frequency", publish_frequency))
                publish_frequency = 10;

            if(!ros::param::get("~T_sample", T_sample))
                T_sample = 0.5;
            if(!ros::param::get("~N_predictsize", N_predictsize))
                N_predictsize = 10;
            if(!ros::param::get("~N_predictcontrol", N_predictcontrol))
                N_predictcontrol = 8;
            if(!ros::param::get("~e_safety", e_safety))
                e_safety = 0.1;
            if(!ros::param::get("~N_maxpolygon", N_maxpolygon))
                N_maxpolygon = 5;

            if(!ros::param::get("~V_max", V_max))
                V_max = 0.3;
            if(!ros::param::get("~at_max", at_max))
                at_max = 0.3;
            if(!ros::param::get("~V_wheelmax", V_wheelmax))
                V_wheelmax = 0.3;
            if(!ros::param::get("~mass", mass))
                mass = 35.0;
            if(!ros::param::get("~W_max", W_max))
                W_max = 1.0;
            if(!ros::param::get("~F_max", F_max))
                F_max = 35.0;
            if(!ros::param::get("~J_max", J_max))
                J_max = 0.3;

            if(!ros::param::get("~L", L))
                L = 0.28;
            if(!ros::param::get("~wheel_radius", wheel_radius))
                wheel_radius = 0.0625;
            
            if(!ros::param::get("~map_frame", map_frame))
                map_frame = "map_base_link";
            if(!ros::param::get("~trajectory_type", trajectory_type))
                trajectory_type = bezier_;

            if(!ros::param::get("~max_iter", max_iter))
                max_iter = 1000;
            if(!ros::param::get("~kernel_size", kernel_size))
                kernel_size = 3.0;
            if(!ros::param::get("~max_dist", max_dist))
                max_dist = 3.0;
            
            initial_optimal();
            ros::Rate rate(publish_frequency);
            while(ros::ok()) {
                ros::spinOnce();
                current_time = ros::Time::now();
                trajectory_publisher();
                if(trajectory_ && lzpose_ && control_ && polygon_) {
                    optimal_solution();
                    if(sqrt(pow(lz_pose.x - target_pose.x,2) + pow(lz_pose.y - target_pose.y,2)) < 0.05) {
                        ROS_INFO(" Robot is at the target point!");
                        control_ = false;
                        v1.data = 0.0;
                        v2.data = 0.0;
                        v3.data = 0.0;

                        // geometry_msgs::PoseStamped pose_current;        
                        // pose_current.pose.position.x = q_target.x*cm_m;
                        // pose_current.pose.position.y = q_target.y*cm_m;
                        // pose_pub.publish(pose_current);
                        continue;
                    }
                    mpc_publisher();
                }
                last_time = current_time;
                rate.sleep();
            }
        };
        ~MPC() {};
};

