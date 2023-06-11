#include "mpc/mpc.hpp"
#include "mpc/controller.hpp"
#include "mpc/trajectory.hpp"
int main(int argc,char **argv) {
    ROS_INFO("Running model predict controller node!");
    ros::init(argc, argv, "mpc");
    ros::NodeHandle nh;

    vel1_pub = nh.advertise<std_msgs::Float64>("/robot_kist/joint_1_velocity/command", 10);
    vel2_pub = nh.advertise<std_msgs::Float64>("/robot_kist/joint_2_velocity/command", 10);
    vel3_pub = nh.advertise<std_msgs::Float64>("/robot_kist/joint_3_velocity/command", 10);
    
    tra_pub = nh.advertise<nav_msgs::Path>("path_predict", 10);
    path_predict_pub = nh.advertise<nav_msgs::Path>("quintic_bezier", 10);

    path_sub = nh.subscribe("global_path", 10, pathCallback);
    x_pose_sub = nh.subscribe("x_position", 10, x_poseCallback);
    y_pose_sub = nh.subscribe("y_position", 10, y_poseCallback);
    theta_pose_sub = nh.subscribe("angle", 10, theta_poseCallback);
    obs_sub = nh.subscribe("visualization_marker",10, obstaclesCallback);

    if(!ros::param::get("~publish_frequency", publish_frequency)) publish_frequency = 10;

    if(!ros::param::get("~T_sample", T_sample)) T_sample = 0.5;
    if(!ros::param::get("~N_predictsize", N_predictsize)) N_predictsize = 10;
    if(!ros::param::get("~N_predictcontrol", N_predictcontrol)) N_predictcontrol = 8;
    if(!ros::param::get("~N_maxobstacles", N_maxobstacles)) N_maxobstacles = 5;
    if(!ros::param::get("~e_safety", e_safety)) e_safety = 0.1;

    if(!ros::param::get("~V_max", V_max)) V_max = 0.3;
    if(!ros::param::get("~at_max", at_max)) at_max = 0.3;
    if(!ros::param::get("~V_wheelmax", V_wheelmax)) V_wheelmax = 0.3;
    if(!ros::param::get("~mass", mass)) mass = 35.0;
    if(!ros::param::get("~W_max", W_max)) W_max = 1.0;
    if(!ros::param::get("~F_max", F_max)) F_max = 35.0;
    if(!ros::param::get("~J_max", J_max)) J_max = 0.3;
    if(!ros::param::get("~L", L)) L = 0.28;
    if(!ros::param::get("~wheel_radius", wheel_radius)) wheel_radius = 0.0625;
            
    if(!ros::param::get("~map_frame", map_frame)) map_frame = "map_base_link";
    if(!ros::param::get("~trajectory_type", trajectory_type)) trajectory_type = bezier_;
    
    if(!ros::param::get("~max_iter", max_iter)) max_iter = 1000;
    if(!ros::param::get("~kernel_size", kernel_size)) kernel_size = 3.0;
    if(!ros::param::get("~max_dist", max_dist)) max_dist = 3.0;
    if(!ros::param::get("~err_goal", err_goal)) err_goal = 0.05;

    SX X = SX::sym("X", 4, N_predictsize+1);
    SX V = SX::sym("V", 4, N_predictsize);

    SX states = SX::vertcat({SX::reshape(X,-1,1), SX::reshape(V,-1,1)});
    SXDict nlp;
    nlp["x"] = states;

    DMDict args;

    Dict opts = Dict();
    opts["print_time"] = 0;
    opts["ipopt.print_level"] = 0;
    opts["ipopt.acceptable_tol"] = 1e-8;
    opts["ipopt.max_iter"] = max_iter;
    opts["ipopt.acceptable_obj_change_tol"] = 1e-6;

    DM lbx = DM::zeros(4*(N_predictsize+1) + 4*N_predictsize, 1);
    DM ubx = DM::zeros(4*(N_predictsize+1) + 4*N_predictsize, 1);

    lbx(Slice(0, 4*(N_predictsize+1), 4)) = -INF;
    ubx(Slice(0, 4*(N_predictsize+1), 4)) = INF;

    lbx(Slice(1, 4*(N_predictsize+1), 4)) = -INF;
    ubx(Slice(1, 4*(N_predictsize+1), 4)) = INF;

    lbx(Slice(2, 4*(N_predictsize+1), 4)) = -V_max;
    ubx(Slice(2, 4*(N_predictsize+1), 4)) = V_max;

    lbx(Slice(3, 4*(N_predictsize+1), 4)) = -V_max;
    ubx(Slice(3, 4*(N_predictsize+1), 4)) = V_max;
    
    lbx(Slice(4*(N_predictsize+1), 4*(N_predictsize+1) + 4*N_predictsize, 4)) = -V_max;
    ubx(Slice(4*(N_predictsize+1), 4*(N_predictsize+1) + 4*N_predictsize, 4)) = V_max;

    lbx(Slice(4*(N_predictsize+1) + 1, 4*(N_predictsize+1) + 4*N_predictsize, 4)) = -V_max;
    ubx(Slice(4*(N_predictsize+1) + 1, 4*(N_predictsize+1) + 4*N_predictsize, 4)) = V_max;

    lbx(Slice(4*(N_predictsize+1) + 2, 4*(N_predictsize+1) + 4*N_predictsize, 4)) = -at_max;
    ubx(Slice(4*(N_predictsize+1) + 2, 4*(N_predictsize+1) + 4*N_predictsize, 4)) = at_max;

    lbx(Slice(4*(N_predictsize+1) + 3, 4*(N_predictsize+1) + 4*N_predictsize, 4)) = -at_max;
    ubx(Slice(4*(N_predictsize+1) + 3, 4*(N_predictsize+1) + 4*N_predictsize, 4)) = at_max;

    DM Q = DM::diagcat({2, 2, 1, 1});
    DM R = DM::diagcat({0.1, 0.1, 0.1, 0.1});
    DM Q_N = DM::diagcat({2, 2, 1, 1});

    DM S = DM({{0 , -sqrt(3)/3, sqrt(3)/3}, {(double)2/3, (double)-1/3, (double)-1/3}, {1/(3*L), 1/(3*L), 1/(3*L)}});
    DM S_inv = DM({{0, 1, L}, {-sqrt(3)/2, -0.5, L}, {sqrt(3)/2, -0.5, L}});

    DM u0 = DM::zeros(4*N_predictsize, 1);
    DM x0 = DM::zeros(4, N_predictsize+1);

    State q_ref;
    int count = 0;
    std_msgs::Float64 v1, v2, v3;
    double v_xr, v_yr;

    ros::Time current_time, last_time;
    current_time = ros::Time::now();
    last_time = ros::Time::now();
    
    ros::Rate rate(publish_frequency);
    while(ros::ok()) {
        ros::spinOnce();
        current_time = ros::Time::now();
        if(path_) {
            trajectory_publisher();
        }
        if(trajectory_ && lzpose_ && control_ && obstacles_) {
            if(sqrt(pow(lz_pose.x - target_pose.x,2) + pow(lz_pose.y - target_pose.y,2)) < err_goal) {
                ROS_INFO(" Robot is at the target point!");
                control_ = false;
                v1.data = 0.0;
                v2.data = 0.0;
                v3.data = 0.0;
                vel1_pub.publish(v1);
                vel2_pub.publish(v2);
                vel3_pub.publish(v3);
                continue;
            }
            double dt = (current_time - last_time).toSec();

            lbx(Slice(0, 4*(N_predictsize+1), 4)) = lz_pose.x - kernel_size;
            lbx(Slice(1, 4*(N_predictsize+1), 4)) = lz_pose.y - kernel_size;

            ubx(Slice(0, 4*(N_predictsize+1), 4)) = lz_pose.x + kernel_size;
            ubx(Slice(1, 4*(N_predictsize+1), 4)) = lz_pose.y + kernel_size;

            args["lbx"] = lbx;
            args["ubx"] = ubx;
            args["x0"] = DM::vertcat({reshape(x0, 3*(N_predictsize+1),1), reshape(u0,3*N_predictsize,1)});

            get_obstacles_nearest(set_obs, set_obs_nearest);

            DM lbg = DM::zeros(4*(N_predictsize+1) + N_predictsize*N_obstacles, 1);
            DM ubg = DM::zeros(4*(N_predictsize+1) + N_predictsize*N_obstacles, 1);
            ubg(Slice(4*(N_predictsize+1), 4*(N_predictsize+1) + N_predictsize*N_obstacles)) = INF;

            DM p_ = DM::zeros(5, N_predictsize+1+N_obstacles);
            p_(Slice(),0) = {lz_pose.x, lz_pose.y, 0, 0, lz_pose.theta};
            for(int i = 0; i < N_predictsize; i++) {
                switch (trajectory_type) {
                    case bezier_:
                        {
                            q_ref = bezier_trajectory(t + (i+1)*T_sample);
                            break;
                        }
                    case scurve_:
                        {
                            q_ref = scurve_trajectory(t + (i+1)*T_sample);
                            break;
                        }
                    default:
                        {
                            ROS_ERROR("Trajectory type incorrect!");
                        }
                }
                p_(Slice(),i+1) = {q_ref.x, q_ref.y, q_ref.vx, q_ref.vy, q_ref.theta};
            }
            for(int i = 0; i < N_obstacles; i++) {
                p_(Slice(), N_predictsize+1+i) = {set_obs_nearest[i].x, set_obs_nearest[i].y, set_obs_nearest[i].theta, set_obs_nearest[i].v, set_obs_nearest[i].radius};
            }
            SX p = SX::sym("p", 5, N_predictsize+1+N_obstacles);
            p = (SX)p_;
            if(sqrt(pow(lz_pose.x - q_ref.x,2) + pow(lz_pose.y - q_ref.y,2)) < max_dist) {
                t = t + dt;
            }

            SX q_N = X(Slice(), N_predictsize);
            SX obj = SX::mtimes(SX::mtimes((q_N - p(Slice(0, 4), N_predictsize)).T(), Q_N), (q_N - p(Slice(0, 4), N_predictsize)));
            for(int k = 0; k < N_predictcontrol; k++) {
                SX u_k = V(Slice(), k);
                SX u_knext = V(Slice(), k+1);
                obj = obj + SX::mtimes(SX::mtimes((u_knext - u_k).T(), R), (u_knext - u_k));
            }
            for(int k = 0; k < N_predictsize; k++) {
                SX q_k = X(Slice(),k);
                SX q_knext = X(Slice(), k+1);
                obj = obj + SX::mtimes(SX::mtimes((q_k - p(Slice(0, 4),k)).T(), Q), (q_k - p(Slice(0, 4),k)));
            }

            SX cst = X(Slice(), 0) - p(Slice(0, 4), 0);
            for(int k = 0; k < N_predictsize; k++) {
                SX q_k = X(Slice(), k);
                SX q_knext = X(Slice(), k+1);
                SX u_k = V(Slice(), k);
                SX H = SX::vertcat({SX::horzcat({cos(p(4, 0)), -sin(p(4, 0)), 0, 0}), SX::horzcat({sin(p(4, 0)), cos(p(4, 0)), 0, 0}), SX::horzcat({0, 0, cos(p(4, 0)), -sin(p(4, 0))}), SX::horzcat({0, 0, sin(p(4, 0)), cos(p(4, 0))})});
                SX h = SX::mtimes(H, u_k);
                SX q_knext_RK4 = q_k + T_sample * h;
                cst = vertcat(cst, q_knext - q_knext_RK4);
            }
            for(int k = 0; k < N_predictsize; k++) {
                SX q_knext = X(Slice(), k+1);
                for(int i = 0; i < N_obstacles; i++) {
                    SX q_obs = p(Slice(), N_predictsize+1+i);
                    if((double)q_obs(3,0) > 1e-2) {
                        q_obs(0,0) += (k+1)*T_sample*q_obs(3,0)*cos(q_obs(2,0));
                        q_obs(1,0) += (k+1)*T_sample*q_obs(3,0)*sin(q_obs(2,0));
                    }
                    SX d = sqrt(pow(q_knext(0,0) - q_obs(0,0), 2) + pow(q_knext(1,0) - q_obs(1,0), 2)) - (L + q_obs(4,0) + e_safety);
                    cst = vertcat(cst, d);
                }
            }

            nlp["f"] = obj;
            nlp["g"] = cst;

            Function solver = nlpsol("solver", "ipopt", nlp, opts);
            ros::Time T1 = ros::Time::now();
            DMDict res = solver(DMDict{{"x0",args["x0"]}, {"lbx",args["lbx"]}, {"ubx", args["ubx"]}, {"lbg", args["lbg"]}, {"ubg", args["ubg"]}});
            ros::Time T2 = ros::Time::now();
            ROS_INFO("Time processing MPC controller: %f",(T2 - T1).toSec());

            DM x_opt = (DM)res["x"];
            x0 = x_opt(Slice(0, 4*(N_predictsize+1)));
            u0 = x_opt(Slice(4*(N_predictsize+1), 4*(N_predictsize+1) + 4*N_predictsize));

            v_xr = (double)u0(0);
            v_yr = (double)u0(1);

            if(pow(v_xr, 2) + pow(v_yr, 2) < 1e-4) {
                count += 1;
                if(count > 5) {
                    ROS_WARN("Robot is stuck!");
                }
            }else {
                count = 0;
            }

            v1.data = v_yr;
            v2.data = -0.5*sqrt(3)*v_xr - 0.5*v_yr;
            v3.data = 0.5*sqrt(3)*v_xr - 0.5*v_yr;
            
            vel1_pub.publish(v1);
            vel2_pub.publish(v2);
            vel3_pub.publish(v3);

            nav_msgs::Path path_predict;
            geometry_msgs::PoseStamped pose_predict;
            path_predict.header.frame_id = "map_frame";
            path_predict.header.stamp = ros::Time::now();
            path_predict.poses.clear();
            for(int i = 0; i < N_predictsize+1; i++) {
                pose_predict.pose.position.x = int((double)x0(3*i)/cm_m);
                pose_predict.pose.position.y = int((double)x0(3*i+1)/cm_m);
                path_predict.poses.push_back(pose_predict);
            }
            path_predict_pub.publish(path_predict);
        }
        last_time = current_time;
        rate.sleep();
    }
    return 0;
}