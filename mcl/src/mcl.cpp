#include "mcl/mcl.hpp"
int main(int argc, char **argv) {
    ros::init(argc, argv, "mcl");
    ROS_INFO("Running global localization node!");
    ros::NodeHandle nh;
    ros::Publisher particlecloud_pub = nh.advertise<geometry_msgs::PoseArray>("particlecloud", 10);
    ros::Publisher amcl_pose_pub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("amcl_pose", 10);
    ros::Publisher point_cloud_pub = nh.advertise<sensor_msgs::PointCloud>("point_cloud", 10);
    
    ros::Subscriber map_sub = nh.subscribe("map", 10, mapCallback);
    message_filters::Subscriber<nav_msgs::Odometry> odom_sub(nh, "odom", 10);
    message_filters::Subscriber<sensor_msgs::LaserScan> laser_scan_sub(nh, "scan1", 10);
    typedef sync_policies::ApproximateTime<nav_msgs::Odometry, sensor_msgs::LaserScan> MySyncPolicy;
    Synchronizer<MySyncPolicy> sync(MySyncPolicy(10),odom_sub, laser_scan_sub);
    sync.registerCallback(dataCallback);
    
    if(!ros::param::get("~anpha1", anpha1))
        anpha1 = 0.005;
    if(!ros::param::get("~anpha2", anpha2))
        anpha2 = 0.005;
    if(!ros::param::get("~anpha3", anpha3))
        anpha3 = 0.01;
    if(!ros::param::get("~anpha4", anpha4))
        anpha4 = 0.005;
    if(!ros::param::get("~anpha5", anpha5))
        anpha5 = 0.003;

    if(!ros::param::get("~sigma_hit", sigma_hit))
        sigma_hit = 0.2;
    if(!ros::param::get("~z_hit", z_hit))
        z_hit = 0.95;
    if(!ros::param::get("~z_rand", z_rand))
        z_rand = 0.05;
    if(!ros::param::get("~z_max", z_max))
        z_max = 0.0;
    if(!ros::param::get("~max_beams", max_beams))
        max_beams = 540;

    if(!ros::param::get("~anpha_slow", anpha_slow))
        anpha_slow = 0.001;
    if(!ros::param::get("~anpha_fast", anpha_fast))
        anpha_fast = 0.1;

    if(!ros::param::get("~dist_max", dist_max))
        dist_max = 2.0;
    if(!ros::param::get("~min_trans", min_trans))
        min_trans = 0.2;
    if(!ros::param::get("~min_rot", min_rot))
        min_rot = M_PI/6;

    if(!ros::param::get("~min_particles", min_particles))
        min_particles = 500;
    if(!ros::param::get("~max_particles", max_particles))
        max_particles = 5000;

    if(!ros::param::get("~throttle_scan", throttle_scan))
        throttle_scan = 5;
    if(!ros::param::get("~outlier_point", outlier_point))
        outlier_point = 3;
    if(!ros::param::get("~range_max", range_max))
        range_max = 25.0;
    
    if(!ros::param::get("~bin_size_x", bin_size_x))
        bin_size_x = 0.5;
    if(!ros::param::get("~bin_size_y", bin_size_y))
        bin_size_y = 0.5;
    if(!ros::param::get("~bin_size_theta", bin_size_theta))
        bin_size_theta = M_PI/12;
    
    if(!ros::param::get("~kld_eps", kld_eps))
        kld_eps = 0.05;
    if(!ros::param::get("~kld_delta", kld_delta))
        kld_delta = 2.326;

    if(!ros::param::get("~init_cov_x", init_cov_x))
        init_cov_x = 1.0;
    if(!ros::param::get("~init_cov_y", init_cov_y))
        init_cov_y = 1.0;
    if(!ros::param::get("~init_cov_theta", init_cov_theta))
        init_cov_theta = 2*M_PI;

    if(!ros::param::get("~frequency_publish", frequency_publish))
        frequency_publish = 20.0;
    
    if(!ros::param::get("~init_pose_x", init_pose_x))
        init_pose_x = 0.0;
    if(!ros::param::get("~init_pose_y", init_pose_y))
        init_pose_y = 0.0;
    if(!ros::param::get("~init_pose_theta", init_pose_theta))
        init_pose_theta = 0.0;

    if(!ros::param::get("~err_min", err_min))
        err_min = 0.05;
    
    if(!ros::param::get("~match_rate", match_rate))
        match_rate = 0.80;
    
    if(!ros::param::get("~model_type", model_type))
        model_type = 0;

    if(!ros::param::get("~lookup_table", lookup_table))
        lookup_table = true;
    if(!ros::param::get("~lookup_table_path", lookup_table_path))
        lookup_table_path = "";
    if(!ros::param::get("~last_pose_path", last_pose_path))
        last_pose_path = "";

    if(!ros::param::get("~uniform_pdf", uniform_pdf))
        uniform_pdf = true;
    
    if(!ros::param::get("~save_last_pose", save_last_pose))
        save_last_pose = true;
    if(!ros::param::get("~set_init_pose", set_init_pose))
        set_init_pose = false;

    if(!ros::param::get("~laser_pose_x", laser_pose_x))
        laser_pose_x = 0.0;
    if(!ros::param::get("~laser_pose_y", laser_pose_y))
        laser_pose_y = -0.0;
    if(!ros::param::get("~laser_pose_theta", laser_pose_theta))
        laser_pose_theta = 0.0;

    if(!ros::param::get("~inverse_laser", inverse_laser))
        inverse_laser = true;
    if(!ros::param::get("~agumented_mcl", agumented_mcl))
        agumented_mcl = true;

    if(!ros::param::get("~base_frame", base_frame))
        base_frame = "base_link";
    if(!ros::param::get("~map_frame", map_frame))
        map_frame = "map";

    if(!ros::param::get("~get_entropy", get_entropy))
        get_entropy = true;

    vector<Particle> set_particle_t_;
    Pose odom_t_1, odom_t;
    Pose u_t[2];
    geometry_msgs::PoseWithCovarianceStamped lz_pose;
    bool _init_sample = false;
    last_pose_file.open(last_pose_path);
    if(last_pose_file.fail()) {
        ROS_ERROR("Invalid last pose link!");
        return 1;
    }
    ros::Rate rate(frequency_publish);
    while(ros::ok()) {
        ros::spinOnce();
        if(first_time) {
            get_lookuptable();
            if(error_) break;
        }
        if(!first_time && _data) {
            odom_t.x = odom.pose.pose.position.x;
            odom_t.y = odom.pose.pose.position.y;
            odom_t.theta = tf::getYaw(odom.pose.pose.orientation);
            if(!_init_sample) {
                map_sub.shutdown();
                if(uniform_pdf) {
                    ROS_INFO("Sampling uniform distribution!");
                    uniform_sample(occ_map);
                }else {
                    ROS_INFO("Sampling normal distribution!");
                    normal_sample();
                }
                if(error_) break;
                set_particle_t_.clear();
                set_particle_t_ = set_particle_t;
                odom_t_1 = odom_t;
                u_t[0] = odom_t;
                _init_sample = true;
            }
            u_t[1] = odom_t;
            if(update_filter(u_t)) {
                if(!agumented_mcl) {
                    KLDSamplingMCL(set_particle_t_, u_t, laser_scan, occ_map);
                }else {
                    AugmentedMCL(set_particle_t_, u_t, laser_scan, occ_map);
                }
                set_particle_t_.clear();
                set_particle_t_ = set_particle_t;
                u_t[0] = odom_t;
            }else{
                Pose u_t_[2];
                u_t_[0] = odom_t_1; u_t_[1] = odom_t;
                update_motion(u_t_);
            }
            if(get_entropy) computeEntropy();
            lz_pose = MeanAndCovariance();
            amcl_pose_pub.publish(lz_pose);

            sensor_msgs::PointCloud PCL;
            PCL.points.clear();
            PCL = PointCloudExport(laser_scan, lz_pose, occ_map);
            point_cloud_pub.publish(PCL);

            geometry_msgs::PoseArray pose_arr;
            pose_arr.header.frame_id = map_frame;
            pose_arr.poses.clear();
            for(int i = 0; i < set_particle_t.size(); i++) {
                geometry_msgs::Pose p;
                p.position.x = set_particle_t[i].pose.x;
                p.position.y = set_particle_t[i].pose.y;
                p.position.z = 0;
                p.orientation = tf::createQuaternionMsgFromYaw(set_particle_t[i].pose.theta);
                pose_arr.poses.push_back(p);
            }
            particlecloud_pub.publish(pose_arr);
            odom_t_1 = odom_t;
        }
        rate.sleep();
    }
    last_pose_file.close();
    return 0;
}