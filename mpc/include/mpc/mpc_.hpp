#pragma once
#include "mpc/mpc.hpp"

void MPC::pathCallback(const nav_msgs::Path& msg) {
    Point point;
    points.clear();
    for(int i = msg.poses.size()-1; i >= 0; i--) {
        point.x = msg.poses[i].pose.position.x;
        point.y = msg.poses[i].pose.position.y;
        points.push_back(point);
    }
    target_pose = points.back();
    num_point = points.size();
    path_ = true;
}

void MPC::x_poseCallback(const std_msgs::Float64& msg) {
    lz_pose.x = msg.data/cm_m;
    lzpose_ = true;
}

void MPC::y_poseCallback(const std_msgs::Float64& msg) {
    lz_pose.y = msg.data/cm_m;
}

void MPC::theta_poseCallback(const std_msgs::Float64& msg) {
    double z = msg.data*M_PI/180;
    lz_pose.theta = atan2(sin(z), cos(z));
}

void MPC::polygonCallback(const visualization_msgs::MarkerArray& msg) {
    Polygon p;
    Point vert;
    polygons.clear();
    if(msg.markers.size() != 0) {
        mode1 = false;
        mode2 = true;
        for(int i = 0; i < msg.markers.size(); i++) {
            p.vertices.clear();
            for(int j = 0; j < msg.markers[i].points.size(); j++) {
                vert.x = msg.markers[i].points[j].x/cm_m;
                vert.y = msg.markers[i].points[j].y/cm_m;
                p.vertices.push_back(vert);
            }
            polygons.push_back(p);
        }
    }else {
        mode1 = true;
        mode2 = false;
    }
    polygon_ = true;
}

double MPC::norm_2(Point q1, Point q2) {
    return sqrt(pow(q1.x - q2.x,2) + pow(q1.y - q2.y,2));
}

Polygon MPC::polygon_centroid(Polygon p) {
    double a_so = 0;
    double mx = 0, my = 0;
    double m;
    Point p_robot;
    p_robot.x = lz_pose.x; p_robot.y = lz_pose.y;
    if(p.vertices.size() > 2) {
        p.vertices.push_back(p.vertices[0]);
        for(int i = 0; i < p.vertices.size()-1; i++) {
            m = p.vertices[i].x*p.vertices[i+1].y - p.vertices[i+1].x*p.vertices[i].y;
            a_so = a_so + m/2;
            mx += (p.vertices[i].x + p.vertices[i+1].x)*m;
            my += (p.vertices[i].y + p.vertices[i+1].y)*m;
        }
        if(fabs(a_so) != 0) {
            p.centroid.x = mx/(6*a_so);
            p.centroid.y = my/(6*a_so);
            p.vertices.pop_back();
            double d_max = norm_2(p.centroid, p.vertices[0]);
            for(int i = 0; i < p.vertices.size(); i++) {
                double d = norm_2(p.centroid, p.vertices[i]);
                if(d > d_max) {
                    d_max = d;
                }
            }
            p.radius = d_max;
             if(d_max > 0.1) {
                 p.radius = d_max;
             }else {
                 p.radius = 0.1;
             }
        }else {
            p.vertices.pop_back();
            p.centroid.x = (p.vertices[1].x + p.vertices[2].x)/2;
            p.centroid.y = (p.vertices[1].y + p.vertices[2].y)/2;
            p.radius = 0.5*norm_2(p.vertices[1], p.vertices[2]);
             if(p.radius < 0.1) {
                 p.radius = 0.1;
             }
        }
    }else {
        ROS_WARN("Numpoint of the polygon is 2!");
        p.centroid.x = (p.vertices[0].x + p.vertices[1].x)/2;
        p.centroid.y = (p.vertices[0].y + p.vertices[1].y)/2;
        p.radius = 0.5*norm_2(p.vertices[0], p.vertices[1]);
    }
    // if(p.vertices.size() > 3) {
    //     for(int i = 0; i < p.vertices.size()-1; i++) {
    //         m = p.vertices[i].x*p.vertices[i+1].y - p.vertices[i+1].x*p.vertices[i].y;
    //         a_so = a_so + m/2;
    //         mx += (p.vertices[i].x + p.vertices[i+1].x)*m;
    //         my += (p.vertices[i].y + p.vertices[i+1].y)*m;
    //     }
    //     if(fabs(a_so) != 0) {
    //         p.centroid.x = mx/(6*a_so);
    //         p.centroid.y = my/(6*a_so);
    //         p.vertices.pop_back();
    //         double d_max = norm_2(p.centroid, p.vertices[0]);
    //         for(int i = 0; i < p.vertices.size(); i++) {
    //             double d = norm_2(p.centroid, p.vertices[i]);
    //             if(d > d_max) {
    //                 d_max = d;
    //             }
    //         }
    //         p.radius = d_max;
    //         // if(d_max > 0.1) {
    //         //     p.radius = d_max;
    //         // }else {
    //         //     p.radius = 0.1;
    //         // }
    //     }else {
    //         ROS_WARN("Area is 0");
    //     }
    // }else {
    //     p.vertices.pop_back();
    //     p.centroid.x = (p.vertices[0].x + p.vertices[1].x)/2;
    //     p.centroid.y = (p.vertices[0].y + p.vertices[1].y)/2;
    //     p.radius = norm_2(p.vertices[0], p.vertices[1])/2;
    //     // if(p.radius < 0.1) {
    //     //     p.radius = 0.1;
    //     // }
    // }
    return p;
}

bool MPC::comparePolygon(Polygon p1, Polygon p2) {
    double d1 = pow(lz_pose.x - p1.centroid.x, 2) + pow(lz_pose.y - p1.centroid.y, 2);
    double d2 = pow(lz_pose.x - p2.centroid.x, 2) + pow(lz_pose.y - p2.centroid.y, 2);
    return (d1 < d2);   
}

void MPC::objective_func() {
    DM Q = DM::diagcat({2, 2, 0.5});
    DM R = DM::diagcat({0.1, 0.1, 0.05});
    DM Q_N = DM::diagcat({2, 2, 0.5});

    SX q_N = X(Slice(),N_predictsize);
    SX obj = SX::mtimes(SX::mtimes((q_N - P(Slice(),N_predictsize)).T(), Q_N), (q_N - P(Slice(),N_predictsize)));
    for(int k = 0; k < N_predictcontrol; k++) {
        SX u_k = V(Slice(),k);
        SX u_knext = V(Slice(), k+1);
        obj = obj + SX::mtimes(SX::mtimes((u_knext-u_k).T(), R), (u_knext-u_k));
    }
    for(int k = 0; k < N_predictsize; k++) {
        SX q_k = X(Slice(),k);
        SX q_knext = X(Slice(), k+1);
        obj = obj + SX::mtimes(SX::mtimes((q_k - P(Slice(),k)).T(), Q), (q_k - P(Slice(),k)));
    }
}

void MPC::cst_func() {
    cst_states = X(Slice(),0) - P(Slice(),0);
    for(int k = 0; k < N_predictsize; k++) {
        SX q_k = X(Slice(),k);
        SX q_knext = X(Slice(), k+1);
        SX u_k = V(Slice(),k);
        
        SX H11 = SX::vertcat({SX::horzcat({cos(q_k(2,0)), -sin(q_k(2,0)), 0}), SX::horzcat({sin(q_k(2,0)), cos(q_k(2,0)), 0}), SX::horzcat({0, 0, 1})});
        SX k1 = SX::mtimes(H11, u_k);

        SX H12 = SX::vertcat({SX::horzcat({cos(q_k(2,0)+0.5*T_sample*u_k(2,0)), -sin(q_k(2,0)+0.5*T_sample*u_k(2,0)), 0}), SX::horzcat({sin(q_k(2,0)+0.5*T_sample*u_k(2,0)), cos(q_k(2,0)+0.5*T_sample*u_k(2,0)), 0}), SX::horzcat({0, 0, 1})});
        SX k2 = SX::mtimes(H12, u_k);

        SX k3 = k2;

        SX H14 = SX::vertcat({SX::horzcat({cos(q_k(2,0)+T_sample*u_k(2,0)), -sin(q_k(2,0)+T_sample*u_k(2,0)), 0}), SX::horzcat({sin(q_k(2,0)+T_sample*u_k(2,0)), cos(q_k(2,0)+T_sample*u_k(2,0)), 0}), SX::horzcat({0, 0, 1})});
        SX k4 = SX::mtimes(H14, u_k);
        
        SX q_knext_RK4 = q_k + (T_sample/6)*(k1 + 2*k2 + 2*k3 + k4);
        cst_states = vertcat(cst_states, q_knext - q_knext_RK4);
    }
    cst_states_collision = cst_states;
    for(int k = 0; k < N_predictsize; k++) {
        SX q_knext = X(Slice(), k+1);
        for(int i = 0; i < N_maxpolygon; i++) {
            SX q_obs = P(Slice(),N_predictsize+1+i);
            SX d = pow(q_knext(0,0) - q_obs(0,0), 2) + pow(q_knext(1,0) - q_obs(1,0), 2) - pow(L + q_obs(2,0) + e_safety, 2); 
            cst_states_collision = vertcat(cst_states_collision, d);
        }
    }
}

void MPC::initial_optimal() {
    X = SX::sym("X", 3, N_predictsize+1);
    V = SX::sym("V", 3, N_predictsize);
    P = SX::sym("P", 3, N_predictsize+1+N_maxpolygon);

    objective_func();
    cst_func();

    SX states = SX::vertcat({SX::reshape(X,-1,1), SX::reshape(V,-1,1)});
    nlp["x"] = states;
    nlp["f"] = obj;
    nlp["p"] = P;

    opts = Dict();
    opts["print_time"] = 0;
    opts["ipopt.print_level"] = 0;
    opts["ipopt.acceptable_tol"] = 1e-8;
    opts["ipopt.max_iter"] = max_iter;
    opts["ipopt.acceptable_obj_change_tol"] = 1e-6;

    lbx = DM::zeros(3*(N_predictsize+1)+3*N_predictsize,1);
    ubx = DM::zeros(3*(N_predictsize+1)+3*N_predictsize,1);

    lbx(Slice(0,3*(N_predictsize+1),3)) = -INF;
    lbx(Slice(1,3*(N_predictsize+1),3)) = -INF;
    lbx(Slice(2,3*(N_predictsize+1),3)) = -INF;

    ubx(Slice(0,3*(N_predictsize+1),3)) = INF;
    ubx(Slice(1,3*(N_predictsize+1),3)) = INF;
    ubx(Slice(2,3*(N_predictsize+1),3)) = INF;
    
    lbx(Slice(3*(N_predictsize+1),3*(N_predictsize+1)+3*N_predictsize,3)) = -V_max;
    ubx(Slice(3*(N_predictsize+1),3*(N_predictsize+1)+3*N_predictsize,3)) = V_max;

    lbx(Slice(3*(N_predictsize+1)+1,3*(N_predictsize+1)+3*N_predictsize,3)) = -V_max;
    ubx(Slice(3*(N_predictsize+1)+1,3*(N_predictsize+1)+3*N_predictsize,3)) = V_max;

    lbg_mode1 = DM::zeros(3*(N_predictsize+1),1);
    ubg_mode1 = DM::zeros(3*(N_predictsize+1),1);

    lbg_mode2 = DM::zeros(3*(N_predictsize+1)+N_predictsize*N_maxpolygon,1);
    ubg_mode2 = DM::zeros(3*(N_predictsize+1)+N_predictsize*N_maxpolygon,1);
    ubg_mode2(Slice(3*(N_predictsize+1), 3*(N_predictsize+1)+N_predictsize*N_maxpolygon)) = INF;

    args["lbx"] = lbx;
    args["ubx"] = ubx;

    u0 = DM::zeros(3*N_predictsize,1);
    p = DM::zeros(3,N_predictsize+1+N_maxpolygon);
    x0 = DM::repmat(p(Slice(),0),1,N_predictsize+1);
}

void MPC::trajectory_publisher() {
    ref.split_spline(0, 0);
    nav_msgs::Path path;
	path.poses.clear();
	State q;
    if(trajectory_type == 1)
        q = ref.bezier_trajectory(0);
    else if(trajectory_type == 2) {
        q = ref.scurve_trajectory(0);
    }
	geometry_msgs::PoseStamped p;
	path.header.frame_id = "map_frame";
	path.header.stamp = ros::Time::now();
	p.pose.position.x = int(cm_m*q.x);
	p.pose.position.y = int(cm_m*q.y);
	path.poses.push_back(p);  
    for(int i = 1; i < ref.T[ref.M]/0.01; i++){	
		double dt = i*0.01;
        if(trajectory_type == 1)
            q = ref.bezier_trajectory(dt);
        else if(trajectory_type == 2) {
            q = ref.scurve_trajectory(dt);
        }
		p.pose.position.x = int(cm_m*q.x);
		p.pose.position.y = int(cm_m*q.y);
		path.poses.push_back(p);	
	}
	tra_pub.publish(path);;
	path_ = false;
    trajectory_ = true;
    control_ = true;
    t = 0;
}

void MPC::optimal_solution() {
    DM H2 = DM({{0 , -sqrt(3)/3, sqrt(3)/3}, {(double)2/3, (double)-1/3, (double)-1/3}, {1/(3*L), 1/(3*L), 1/(3*L)}});
    DM H2_inv = DM({{0, 1, L}, {-sqrt(3)/2, -0.5, L}, {sqrt(3)/2, -0.5, L}});
    double dt = (current_time - last_time).toSec();
    p(Slice(),0) = {lz_pose.x, lz_pose.y, lz_pose.theta};
    State q_ref;
    for(int i = 0; i < N_predictsize; i++) {
        if(trajectory_type == 1)
            q_ref = ref.bezier_trajectory(t + (i+1)*T_sample);
        else if(trajectory_type == 2) {
            q_ref = ref.scurve_trajectory(t + (i+1)*T_sample);
        }
        p(Slice(),i+1) = {q_ref.x, q_ref.y, q_ref.theta};
        //int j = N_predictsize+1+N_maxpolygon+i;
        //p(Slice(),j) = {q_ref.vx, q_ref.vy, q_ref.w};
    }
    lbx(Slice(0,3*(N_predictsize+1),3)) = lz_pose.x - kernel_size;
    lbx(Slice(1,3*(N_predictsize+1),3)) = lz_pose.y - kernel_size;
    lbx(Slice(2,3*(N_predictsize+1),3)) = -M_PI;

    ubx(Slice(0,3*(N_predictsize+1),3)) = lz_pose.x + kernel_size;
    ubx(Slice(1,3*(N_predictsize+1),3)) = lz_pose.y + kernel_size;
    ubx(Slice(2,3*(N_predictsize+1),3)) = M_PI;
    args["lbx"] = lbx;
    args["ubx"] = ubx;
    if(sqrt(pow(lz_pose.x - q_ref.x,2) + pow(lz_pose.y - q_ref.y,2)) < max_dist) {
        t = t + dt;
    }
    if(mode1 == false && mode2 == true) {
        SX cst = cst_states_collision;
        nlp["g"] = cst;
        args["lbg"] = lbg_mode2;
        args["ubg"] = ubg_mode2;
        for(int i = 0; i < polygons.size(); i++) {
            polygons[i] = polygon_centroid(polygons[i]);
        }
        sort(polygons.begin(), polygons.end(), comparePolygon);
        vector<Polygon> nearest_polygons;
        nearest_polygons.clear();
        if(polygons.size() > N_maxpolygon) {
            for(int i = 0; i < N_maxpolygon; i++) {
                nearest_polygons.push_back(polygons[i]);
            }
        }else {
            for(int i = 0; i < polygons.size(); i++) {
                nearest_polygons.push_back(polygons[i]);
            }
            for(int i = 0; i < N_maxpolygon-polygons.size(); i++) {
                nearest_polygons.push_back(polygons[0]);
            }
        }
        for(int i = 0; i < N_maxpolygon; i++) {
            Polygon q_obs = nearest_polygons[i];
            p(Slice(),N_predictsize+1+i) = {q_obs.centroid.x, q_obs.centroid.y, q_obs.radius};
        }
    }else {
        SX cst = cst_states;
        nlp["g"] = cst;
        args["lbg"] = lbg_mode1;
        args["ubg"] = ubg_mode1;
    }
    args["p"] = p;
    args["x0"] = DM::vertcat({reshape(x0, 3*(N_predictsize+1),1), reshape(u0,3*N_predictsize,1)});

    Function solver = nlpsol("solver", "ipopt", nlp, opts);
            
    ros::Time T1 = ros::Time::now();
    DMDict res = solver(DMDict{{"x0",args["x0"]}, {"lbx",args["lbx"]}, {"ubx", args["ubx"]}, {"lbg", args["lbg"]}, {"ubg", args["ubg"]}, {"p", args["p"]}});
    ros::Time T2 = ros::Time::now();
            
    DM x_opt = (DM)res["x"];
    x0 = x_opt(Slice(0,3*(N_predictsize+1)));
    u0 = x_opt(Slice(3*(N_predictsize+1),3*(N_predictsize+1)+3*N_predictsize));
    // ROS_INFO("Time processing MPC controller: %f",(T2 - T1).toSec());
            
    DM u_local = mtimes(H2_inv, u0(Slice(0,3)));
    if(pow((double)u_local(0), 2) + pow((double)u_local(1), 2) < 1e-4) {
        count += 1;
        if(count > 5) {
            ROS_WARN("Robot is stuck!");
        }
    }else {
        count = 0;
    }
    v1.data = (double)u_local(0);
    v2.data = (double)u_local(1);
    v3.data = (double)u_local(2);
}

void MPC::mpc_publisher() {
    vel1_pub.publish(v1);
    vel2_pub.publish(v2);
    vel3_pub.publish(v3);

    nav_msgs::Path path_predict;
    geometry_msgs::PoseStamped pose_predict;
    path_predict.header.frame_id = "map_base_link";
    path_predict.header.stamp = ros::Time::now();
    path_predict.poses.clear();
    for(int i = 0; i < N_predictsize+1; i++) {
        pose_predict.pose.position.x = int((double)x0(3*i)*cm_m);
        pose_predict.pose.position.y = int((double)x0(3*i+1)*cm_m);
        path_predict.poses.push_back(pose_predict);
    }
    path_predict_pub.publish(path_predict);
}

