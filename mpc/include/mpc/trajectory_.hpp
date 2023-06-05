#pragma once
#include "mpc/trajectory.hpp"

State Trajectory::scurve_trajectory(double t) {
    double kx, ky;
    double d, v, a, J;
    double t0, t1, t2, t3, t4, t5, t6, t7, T;
    State q_t;
    for(int i = 0; i < num_point-1; i++) {
        kx = points[i+1].x - points[i].x;
        ky = points[i+1].y - points[i].y;
        d = sqrt(pow(kx, 2) + pow(ky, 2));

        v = V_max/d;
        a = at_max/d;
        J = J_max/d;
        if(v*v/a + a*v/J >= 1 || a/J >= v/a) {
            ROS_WARN("Choosing the wrong parameter v a J!");
        }
        T = a/J + v/a + 1/v;
        t0 = 0;
        t1 = a/J;
        t2 = v/a;
        t3 = v/a + a/J;
        t4 = T - t3;
        t5 = T - t2;
        t6 = T - t1;
        t7 = T;
        double n, m, s;
        if(t <= t1) {
            n = J*t;
            m = J*t*t/2;
            s = J*pow(t, 3)/6;
        }else if(t > t1 && t <= t2) {
            n = a;
            m = a*t - a*a/(2*J);
            s = (a*t*t)/2 + pow(a, 3)/(6*J*J) - (a*a*t)/(2*J);
        }else if(t > t2 && t <= t3) {
            n = a - J*(t - v/a);
            m = v - a*a/(2*J) - ((v - a*t)*(2*a*a - a*J*t + J*v))/(2*a*a);
            s = (-pow(J*a*t, 3) + 3*pow(J, 3)*pow(a*t, 2)*v - 3*pow(J, 3)*a*t*pow(v, 2) + pow(J*v, 3) + 3*J*J*pow(a*a*t, 2) - 3*J*pow(a, 5)*t + pow(a, 6))/(6*J*J*pow(a, 3));
        }else if(t > t3 && t <= t4) {
            n = 0;
            m = v;
            s = -(v*(a*a - 2*J*a*t + J*v))/(2*J*a);
        }else if(t > t4 && t <= t5) {
            n = -J*(t - T + a/J + v/a);
            m = v - pow((J*v + a*a - J*T*a + J*a*t), 2)/(2*J*a*a);
            s = -pow(J*T*a, 3) + 3*pow(J*a, 3)*T*T*t + 3*pow(J, 3)*pow(a*T, 2)*v - 3*pow(J*a, 3)*T*t*t - 6*pow(J, 3)*T*a*a*t*v - 3*pow(J, 3)*T*a*v*v + pow(J*a*t, 3) + 3*pow(J, 3)*pow(a*t, 2)*v;
            s += 3*pow(J, 3)*a*t*v*v + pow(J*v, 3) + 3*pow(J*T*a*a, 2) - 6*pow(J*a*a, 2)*T*t - 6*J*J*T*pow(a, 3)*v + 3*pow(J*a*a*t, 2) + 6*pow(J*a*v, 2) - 3*J*T*pow(a, 5);
            s += 3*J*pow(a, 5)*t + 6*J*pow(a, 4)*v + pow(a, 6);
            s = -s/(6*J*J*pow(a, 3));
        }else if(t > t5 && t <= t6) {
            n = -a;
            m = -(a*(a + 2*J*t - 2*J*T))/(2*J);
            s = ((v - T*a + a*t)*(J*v - a*a +J*T*a - J*a*t))/(2*J*a) - (-6*T*J*J*a*v + 9*J*J*v*v + 3*J*a*a*v + pow(a, 4))/(6*J*J*a);
        }else if(t > t6 && t <= t7) {
            n = J*(t - T + a/J) - a;
            m = (J*pow((T - t), 2))/2;
            s = pow(a, 3)/(6*J*J) - J*(pow(T, 3)/6 - (T*T*t)/2 + (T*t*t)/2 - pow(t, 3)/6) - (-6*T*J*J*a*v + 6*pow(J*v, 2) + 6*J*a*a*v + pow(a, 4))/(6*J*J*a);
        }else {
            t = T;
            n = J*(t - T + a/J) - a;
            m = (J*pow((T - t), 2))/2;
            s = pow(a, 3)/(6*J*J) - J*(pow(T, 3)/6 - (T*T*t)/2 + (T*t*t)/2 - pow(t, 3)/6) - (-6*T*J*J*a*v + 6*pow(J*v, 2) + 6*J*a*a*v + pow(a, 4))/(6*J*J*a);
        }
        q_t.ax = kx * n; q_t.ay = ky * n;
        q_t.vx = kx * m; q_t.vy = ky * m;
        q_t.x = points[i].x + kx * s; q_t.y = points[i].y + ky * s;
    }
    return q_t;
}

void Trajectory::path_planner(vector<Point>& p1, State& p2) {
    double factor = 0.5;
    vector<double> dis_point;
    Vector n1, n2, n;
    Vector tangent[num_point];
    Vector second_der[num_point];
    double r1, r2;
    double dis_min, vec_magnitude;
    double anpha, beta;
    Point P0, P1, P2, P3, P4, P5;
    Point c;
    c0.clear();
    c1.clear();
    c2.clear();
    c3.clear();
    c4.clear();
    c5.clear();
    for(int i = 0; i < num_point-1; i++) {
        double d = sqrt(pow(p1[i].x - p1[i+1].x, 2) + pow(p1[i].y - p1[i+1].y, 2));
        if (d > 1e-4) {
            dis_point.push_back(d);
        } else {
            ROS_WARN("Two identical points!");
            p1.erase(p1.begin() + i);
            num_point--;
            i--;
        }
    }
    // tangent[0].nx = factor*dis_point[0]*cos(p2.theta);
    // tangent[0].ny = factor*dis_point[0]*sin(p2.theta);
    tangent[0].nx = factor*(p1[1].x - p1[0].x); 
    tangent[0].ny = factor*(p1[1].y - p1[0].y);

    tangent[num_point-1].nx = factor*(p1[num_point-1].x - p1[num_point-2].x);
    tangent[num_point-1].ny = factor*(p1[num_point-1].y - p1[num_point-2].y);

    for(int i = 1; i < num_point-1; i++){
        n1.nx = (p1[i-1].x - p1[i].x)/dis_point[i-1];
        n1.ny = (p1[i-1].y - p1[i].y)/dis_point[i-1];
 
        n2.nx = (p1[i+1].x - p1[i].x)/dis_point[i];
        n2.ny = (p1[i+1].y - p1[i].y)/dis_point[i];

        n.nx = -(n1.ny + n2.ny);
        n.ny = n1.nx + n2.nx;

        r1 = acos((n1.nx*n.nx + n1.ny*n.ny)/(sqrt(pow(n1.nx, 2) + pow(n1.ny, 2))*sqrt(pow(n.nx, 2) + pow(n.ny, 2))));
        r2 = acos((n.nx*n2.nx + n.ny*n2.ny)/(sqrt(pow(n.nx, 2) + pow(n.ny, 2))*sqrt(pow(n2.nx, 2) + pow(n2.ny, 2))));
        if (r2 < r1) {
            n.nx = -(n1.ny + n2.ny);
            n.ny = n1.nx + n2.nx;
        } else {
            n.nx = n1.ny + n2.ny;
            n.ny = -(n1.nx + n2.nx);
        }
        if(dis_point[i-1] < dis_point[i]) {
            dis_min = dis_point[i-1];
        } else {
            dis_min = dis_point[i];
        }
        vec_magnitude = sqrt(pow(n.nx, 2) + pow(n.ny, 2));
        tangent[i].nx = factor*dis_min*n.nx/vec_magnitude;
        tangent[i].ny = factor*dis_min*n.ny/vec_magnitude;
    }
    second_der[0].nx = -6*p1[0].x - 4*tangent[0].nx - 2*tangent[1].nx + 6*p1[1].x;
    second_der[0].ny = -6*p1[0].y - 4*tangent[0].ny - 2*tangent[1].ny + 6*p1[1].y;
    second_der[num_point-1].nx = 6*p1[num_point-2].x + 2*tangent[num_point-2].nx + 4*tangent[num_point-1].nx - 6*p1[num_point-1].x;
    second_der[num_point-1].ny = 6*p1[num_point-2].y + 2*tangent[num_point-2].ny + 4*tangent[num_point-1].ny - 6*p1[num_point-1].y;
    for(int i = 1; i < num_point-1; i++) {
        anpha = dis_point[i]/(dis_point[i] + dis_point[i-1]);
        beta = dis_point[i-1]/(dis_point[i] + dis_point[i-1]);
        second_der[i].nx = anpha*(6*p1[i-1].x + 2*tangent[i-1].nx + 4*tangent[i].nx - 6*p1[i].x) + beta*(-6*p1[i].x - 4*tangent[i].nx - 2*tangent[i+1].nx + 6*p1[i+1].x);
        second_der[i].ny = anpha*(6*p1[i-1].y + 2*tangent[i-1].ny + 4*tangent[i].ny - 6*p1[i].y) + beta*(-6*p1[i].y - 4*tangent[i].ny - 2*tangent[i+1].ny + 6*p1[i+1].y);
    }
    for(int i = 0; i < num_point-1; i++){
        P0.x = p1[i].x;
        P0.y = p1[i].y;

        P5.x = p1[i+1].x;
        P5.y = p1[i+1].y;

        P1.x = tangent[i].nx/5 + P0.x;
        P1.y = tangent[i].ny/5 + P0.y;

        P2.x = second_der[i].nx/20 + 2*P1.x - P0.x;
        P2.y = second_der[i].ny/20 + 2*P1.y - P0.y;

        P4.x = P5.x - tangent[i+1].nx/5;
        P4.y = P5.y - tangent[i+1].ny/5;

        P3.x = second_der[i+1].nx/20 + 2*P4.x - P5.x;
        P3.y = second_der[i+1].ny/20 + 2*P4.y - P5.y;

        c.x = P0.x;
        c.y = P0.y;
        c0.push_back(c);

        c.x = -5*P0.x + 5*P1.x;
        c.y = -5*P0.y + 5*P1.y;
        c1.push_back(c);

        c.x = 10*P0.x - 20*P1.x + 10*P2.x;
        c.y = 10*P0.y - 20*P1.y + 10*P2.y;
        c2.push_back(c);

        c.x = -10*P0.x + 30*P1.x - 30*P2.x + 10*P3.x;
        c.y = -10*P0.y + 30*P1.y - 30*P2.y + 10*P3.y;
        c3.push_back(c);

        c.x = 5*P0.x - 20*P1.x + 30*P2.x - 20*P3.x + 5*P4.x;
        c.y = 5*P0.y - 20*P1.y + 30*P2.y - 20*P3.y + 5*P4.y;
        c4.push_back(c);

        c.x = -P0.x + 5*P1.x - 10*P2.x + 10*P3.x - 5*P4.x + P5.x;
        c.y = -P0.y + 5*P1.y - 10*P2.y + 10*P3.y - 5*P4.y + P5.y;
        c5.push_back(c);
    }
}

double Trajectory::func(double u, int idx) {
    double qx_dot, qy_dot;
    qx_dot = c1[idx].x + 2*c2[idx].x*u + 3*c3[idx].x*pow(u, 2) + 4*c4[idx].x*pow(u, 3) + 5*c5[idx].x*pow(u, 4);
    qy_dot = c1[idx].y + 2*c2[idx].y*u + 3*c3[idx].y*pow(u, 2) + 4*c4[idx].y*pow(u, 3) + 5*c5[idx].y*pow(u, 4);
    return sqrt(pow(qx_dot, 2) + pow(qy_dot, 2));
}

double Trajectory::func_curve(double u, int idx) {
    double qx_dot, qy_dot;
    double qx_2dot, qy_2dot;
    double curve;
    qx_dot = c1[idx].x + 2*c2[idx].x*u + 3*c3[idx].x*pow(u, 2) + 4*c4[idx].x*pow(u, 3) + 5*c5[idx].x*pow(u, 4);
    qy_dot = c1[idx].y + 2*c2[idx].y*u + 3*c3[idx].y*pow(u, 2) + 4*c4[idx].y*pow(u, 3) + 5*c5[idx].y*pow(u, 4);

    qx_2dot = 2*c2[idx].x + 6*c3[idx].x*u + 12*c4[idx].x*pow(u, 2) + 20*c5[idx].x*pow(u, 3);
    qy_2dot = 2*c2[idx].y + 6*c3[idx].y*u + 12*c4[idx].y*pow(u, 2) + 20*c5[idx].y*pow(u, 3);

    curve = fabs(qx_dot*qy_2dot - qy_dot*qx_2dot)/pow(pow(qx_dot, 2) + pow(qy_dot, 2), 1.5);
    return curve;
}

double Trajectory::inter_Romberg(double (*f)(double, int), double a, double b, int idx) {
    int max_step = 100;
    double acc = 1e-3;
    double c;
    double ep;
    vector<double> R1, R2;
    double h = (b - a);
    R1.push_back((f(a, idx) + f(b, idx))*h*0.5);
    for(int i = 1; i < max_step; i++) {
        h = 0.5*h;
        c = 0;
        ep = 1 << (i-1); // 2^(n-1)
        for(int j = 1; j <= ep; j++){
            c = c + f(a+(2*j-1)*h, idx);
        }
        R2.push_back(h*c + 0.5*R1[0]);
        for(int j = 1; j <= i; j++){
            R2.push_back((pow(4, j)*R2[j-1] - R1[j-1])/(pow(4, j) - 1));
        }
        if((i > 1) && (fabs(R1[i-1] - R2[i]) < acc)) {
            return R2[i];
        }
        R1.clear();
        for(int j = 0; j < R2.size(); j++) {
            R1.push_back(R2[j]);
        }
        R2.clear();
    }
    return R1[max_step-1];
}

void Trajectory::split_spline(double V_init, double V_end) {
    double L_h, L_;
    double del_s, del_length;
    double length[num_point-1];
    int k;
    double u, eps;
    double c;
    vector<double> v;
    double v_i;
    double t;
    double V_maxcurve, V_maxf;
    double u_left, u_right;
    T.clear();
    U.clear();
    arc_length.clear();
    path_planner(points, lz_pose);
    L_ = 0;
    for(int i = 0; i < num_point-1; i++) {
        length[i] = inter_Romberg(func,0,1,i);
        arc_length.push_back(length[i]);
        L_ += length[i];
    }
    del_s = L_;
    M = 1;
    while(del_s > 0.02) {
        M += 1;
        del_s = L_/M;
    }
    for(int i = 0; i <= M; i++) {
        L_h = 0;
        for(int j = 0; j < num_point-1; j++) {
            L_h += length[j];
            if(i*del_s < L_h) {
                k = j;
                break;
            }
        }
        if(k == 0) {
            del_length = i*del_s;
        } else {
            del_length = i*del_s;
            for(int j = 0; j < k; j++) {
                del_length = del_length - length[j];
            }
        }
        /* Newton - Raphson Method */
        // u = 0 + 0.1*del_length/inter_Romberg(func,0,0.1,k);
        // eps = fabs(inter_Romberg(func,0,u,k) - del_length);
        // while(eps > 0.001) {
        //     u = u - (inter_Romberg(func,0,u,k) - del_length)/func(u, k);
        //     eps = fabs(inter_Romberg(func,0,u,k) - del_length);
        // }
        /* Bisector Method */
        u_left = 0;
        u_right = 1;
        u = (u_left + u_right)/2;
        eps = inter_Romberg(func,0,u,k) - del_length;
        while(fabs(eps) > 10e-4) {
            if(eps < 0) {
                u_left = u;
            } else if (eps > 0) {
                u_right = u;
            }
            u = (u_left + u_right)/2;
            eps = inter_Romberg(func,0,u,k) - del_length;
        }
        U.push_back(u);
        c = func_curve(u, k);
        // Profile velocity phase I
        v_i = V_max;
        V_maxcurve = W_max/c;
        V_maxf = sqrt(F_max/(mass*c));
        if(v_i > V_maxcurve) {
            v_i = V_maxcurve;
        } else {
            v_i = V_max;
        }
        if(v_i > V_maxf) {
            v_i = V_maxf;
        }
        // Profile velocity phase II
        if(i == 0) {
            v_i = V_init;
        } else {
            v_i = (v_i > (sqrt(pow(v[i-1],2) + 2*del_s*at_max))) ? sqrt(pow(v[i-1],2) + 2*del_s*at_max) : v_i;
        }
        v.push_back(v_i);
    }
        // Profile velocity phase III
    reverse(v.begin(), v.end());
    for(int i = 0; i <= M; i++) {
        if(i == 0) {
            v[i] = V_end;
        } else {
            v[i] = (v[i] > (sqrt(pow(v[i-1],2) + 2*del_s*at_max))) ? sqrt(pow(v[i-1],2) + 2*del_s*at_max) : v[i];
        }
    }
    reverse(v.begin(), v.end());
    for(int i = 0; i <= M; i++) {
        if(i == 0) {
            t = 0;
        } else {
            t = T[i-1] + 2*del_s/(v[i] + v[i-1]);
        }
        T.push_back(t);
    }
}

State Trajectory::bezier_trajectory(double t) {
    double sum_length = 0;
    double u;
    double u_dot;
    State q;
    int idx;
    double t0;
    for(int i = 0; i < num_point-1; i++) {
        sum_length += arc_length[i];
    }
    double del_s = sum_length/M;
    for(int i = 0; i < M; i++) {
        if(t >= T[i] && t < T[i+1]) {
            double L = 0;
                for(int j = 0; j < num_point-1; j++) {
                    L += arc_length[j];
                    if(i*del_s < L) {
                        idx = j;
                        break;
                    }
                }
            if(U[i+1] - U[i] > 0) {
                u = U[i] + (t - T[i])*(U[i+1] - U[i])/(T[i+1] - T[i]);
                u_dot = (U[i+1] - U[i])/(T[i+1] - T[i]);
            } else if(U[i+1] - U[i] < 0) {
                t0 = T[i] + (T[i+1] - T[i])*(1 - U[i])/(1 - U[i] + U[i+1]);
                if(t <= t0) {
                    u = U[i] + (t - T[i])*(1 - U[i])/(t0 - T[i]);
                    u_dot = (1 - U[i])/(t0 - T[i]);
                } else  {
                    u = 0 + (t - t0)*(U[i+1] - 0)/(T[i+1] - t0);
                    u_dot = (U[i+1] - 0)/(T[i+1] - t0);
                    idx = idx + 1;
                }
            }
            q.x = c0[idx].x + c1[idx].x*u + c2[idx].x*pow(u, 2) + c3[idx].x*pow(u, 3) + c4[idx].x*pow(u, 4) + c5[idx].x*pow(u, 5);
            q.y = c0[idx].y + c1[idx].y*u + c2[idx].y*pow(u, 2) + c3[idx].y*pow(u, 3) + c4[idx].y*pow(u, 4) + c5[idx].y*pow(u, 5);
            q.theta = lz_pose.theta;
            
            q.vx = (c1[idx].x + 2*c2[idx].x*u + 3*c3[idx].x*pow(u, 2) + 4*c4[idx].x*pow(u, 3) + 5*c5[idx].x*pow(u, 4))*u_dot;
    	    q.vy = (c1[idx].y + 2*c2[idx].y*u + 3*c3[idx].y*pow(u, 2) + 4*c4[idx].y*pow(u, 3) + 5*c5[idx].y*pow(u, 4))*u_dot;
            q.w = 0;
        } else if(t > T[M]) {
            idx = num_point - 2;
            u = 1;
            u_dot = 0;
            q.x = c0[idx].x + c1[idx].x*u + c2[idx].x*pow(u, 2) + c3[idx].x*pow(u, 3) + c4[idx].x*pow(u, 4) + c5[idx].x*pow(u, 5);
            q.y = c0[idx].y + c1[idx].y*u + c2[idx].y*pow(u, 2) + c3[idx].y*pow(u, 3) + c4[idx].y*pow(u, 4) + c5[idx].y*pow(u, 5);
            q.theta = lz_pose.theta;
            
            q.vx = (c1[idx].x + 2*c2[idx].x*u + 3*c3[idx].x*pow(u, 2) + 4*c4[idx].x*pow(u, 3) + 5*c5[idx].x*pow(u, 4))*u_dot;
    	    q.vy = (c1[idx].y + 2*c2[idx].y*u + 3*c3[idx].y*pow(u, 2) + 4*c4[idx].y*pow(u, 3) + 5*c5[idx].y*pow(u, 4))*u_dot;
            q.w = 0;
        }
    }
    return q;
}