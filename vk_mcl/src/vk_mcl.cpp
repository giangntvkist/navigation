#include "vk_mcl/vk_mcl.hpp"
#include "vk_mcl/motion_model.hpp"
#include "vk_mcl/sensor_model.hpp"
#include "vk_mcl/particle_filter.hpp"
#include "vk_mcl/map.hpp"
#include "vk_mcl/parameters.hpp"

int main(int argc, char **argv) {
    ros::init(argc, argv, "vk_mcl");
    ROS_INFO("Running global localization node!");
    ros::NodeHandle nh;
    ros::Publisher particlecloud_pub = nh.advertise<geometry_msgs::PoseArray>("particlecloud", 2);
    ros::Publisher amcl_pose_pub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("amcl_pose", 2);
    ros::Publisher point_cloud_pub = nh.advertise<sensor_msgs::PointCloud>("point_cloud", 2);
    
    ros::Subscriber map_sub = nh.subscribe("map", 1, mapCallback);
    ros::Subscriber init_pose_sub = nh.subscribe("initialpose", 1, init_poseCallback);
    message_filters::Subscriber<nav_msgs::Odometry> odom_sub(nh, "odom", 5);
    message_filters::Subscriber<sensor_msgs::LaserScan> laser_scan_sub(nh, "scan1", 2);
    typedef sync_policies::ApproximateTime<nav_msgs::Odometry, sensor_msgs::LaserScan> MySyncPolicy;
    Synchronizer<MySyncPolicy> sync(MySyncPolicy(10),odom_sub, laser_scan_sub);
    sync.registerCallback(dataCallback);

    if(!ros::param::get("~range_min", range_min)) range_min = 0.05;
    if(!ros::param::get("~range_max", range_max)) range_max = 25.0;
    if(!ros::param::get("~angle_min", angle_min)) angle_min = -M_PI/2;
    if(!ros::param::get("~angle_max", angle_max)) angle_max = M_PI/2;
    if(!ros::param::get("~angle_increment", angle_increment)) angle_increment = M_PI/(3*180);

    if(!ros::param::get("~laser_pose_x", laser_pose_x)) laser_pose_x = 0.289;
    if(!ros::param::get("~laser_pose_y", laser_pose_y)) laser_pose_y = -0.0;
    if(!ros::param::get("~laser_pose_theta", laser_pose_theta)) laser_pose_theta = 0.0;

    if(!ros::param::get("~model_type", model_type)) model_type = 0;
    if(!ros::param::get("~anpha1", anpha1)) anpha1 = 0.005;
    if(!ros::param::get("~anpha2", anpha2)) anpha2 = 0.005;
    if(!ros::param::get("~anpha3", anpha3)) anpha3 = 0.01;
    if(!ros::param::get("~anpha4", anpha4)) anpha4 = 0.005;
    if(!ros::param::get("~anpha5", anpha5)) anpha5 = 0.003;

    if(!ros::param::get("~sigma_hit", sigma_hit)) sigma_hit = 0.2;
    if(!ros::param::get("~z_hit", z_hit)) z_hit = 0.95;
    if(!ros::param::get("~z_rand", z_rand)) z_rand = 0.05;
    if(!ros::param::get("~z_max", z_max)) z_max = 0.0;
    if(!ros::param::get("~max_beams", max_beams)) max_beams = 540;

    if(!ros::param::get("~lamda_hit", lamda_hit)) lamda_hit = 0.7;
    if(!ros::param::get("~lamda_short", lamda_short)) lamda_short = 0.2;
    if(!ros::param::get("~lamda_rand", lamda_rand)) lamda_rand = 0.05;
    if(!ros::param::get("~lamda_max", lamda_max)) lamda_max = 0.05;
    if(!ros::param::get("~max_occ_dist", max_occ_dist)) max_occ_dist = 2.0;

    if(!ros::param::get("~anpha_slow", anpha_slow)) anpha_slow = 0.001;
    if(!ros::param::get("~anpha_fast", anpha_fast)) anpha_fast = 0.1;
    if(!ros::param::get("~min_trans", min_trans)) min_trans = 0.05;
    if(!ros::param::get("~min_rot", min_rot)) min_rot = M_PI/6;
    if(!ros::param::get("~min_particles", min_particles)) min_particles = 500;
    if(!ros::param::get("~max_particles", max_particles)) max_particles = 5000;
    if(!ros::param::get("~throttle_scan", throttle_scan)) throttle_scan = 9;

    if(!ros::param::get("~beam_skip_threshold", beam_skip_threshold)) beam_skip_threshold = 0.3;
    if(!ros::param::get("~beam_skip_distance", beam_skip_distance)) beam_skip_distance = 0.3;
    if(!ros::param::get("~beam_skip_error_threshold", beam_skip_error_threshold)) beam_skip_error_threshold = 0.9;
    if(!ros::param::get("~converged_distance", converged_distance)) converged_distance = 0.5;

    if(!ros::param::get("~bin_size_x", bin_size_x)) bin_size_x = 0.5;
    if(!ros::param::get("~bin_size_y", bin_size_y)) bin_size_y = 0.5;
    if(!ros::param::get("~bin_size_theta", bin_size_theta)) bin_size_theta = M_PI/12;
    
    if(!ros::param::get("~kld_eps", kld_eps)) kld_eps = 0.05;
    if(!ros::param::get("~kld_delta", kld_delta)) kld_delta = 2.326;

    if(!ros::param::get("~init_cov_x", init_cov_x)) init_cov_x = 0.5*0.5;
    if(!ros::param::get("~init_cov_y", init_cov_y)) init_cov_y = 0.5*0.5;
    if(!ros::param::get("~init_cov_theta", init_cov_theta)) init_cov_theta = M_PI/6*M_PI/6;

    if(!ros::param::get("~frequency_publish", frequency_publish)) frequency_publish = 20.0;
    
    if(!ros::param::get("~init_pose_x", init_pose_x)) init_pose_x = 0.0;
    if(!ros::param::get("~init_pose_y", init_pose_y)) init_pose_y = 0.0;
    if(!ros::param::get("~init_pose_theta", init_pose_theta)) init_pose_theta = 0.0;

    if(!ros::param::get("~err_min", err_min)) err_min = 0.05;
    if(!ros::param::get("~match_rate", match_rate)) match_rate = 0.80;

    if(!ros::param::get("~likelihoodfield", likelihoodfield)) likelihoodfield = true;
    if(!ros::param::get("~nor_sampling", nor_sampling)) nor_sampling = true;

    if(!ros::param::get("~lookup_table_path", lookup_table_path)) lookup_table_path = "";
    if(!ros::param::get("~last_pose_path", last_pose_path)) last_pose_path = "";
    if(!ros::param::get("~save_last_pose", save_last_pose)) save_last_pose = false;
    if(!ros::param::get("~set_init_pose", set_init_pose)) set_init_pose = false;

    if(!ros::param::get("~inverse_laser", inverse_laser)) inverse_laser = true;
    if(!ros::param::get("~agumented_mcl", agumented_mcl)) agumented_mcl = true;

    if(!ros::param::get("~base_frame", base_frame)) base_frame = "base_link";
    if(!ros::param::get("~map_frame", map_frame)) map_frame = "map";
    if(!ros::param::get("~base_scan_frame", base_scan_frame)) base_scan_frame = "base_scan";
    if(!ros::param::get("~entropy", entropy)) entropy = false;

    pf_set_sample_t S_t_1, S_t;
    pf_vector_t odom_t_1, odom_t;
    pf_vector_t u_t[2];
    pf_scan_t scan_w_t;

    geometry_msgs::PoseWithCovarianceStamped lz_pose;
    geometry_msgs::PoseArray hypoth_pose_t;
    sensor_msgs::PointCloud pcl_t;

    pf_vector_t laser_pose;
    laser_pose.v[0] = laser_pose_x;
    laser_pose.v[1] = laser_pose_y;
    laser_pose.v[2] = laser_pose_theta;

    ros::Rate rate(frequency_publish);
    while(ros::ok()) {
        ros::spinOnce();
        if(map_data && first_time) {
            lookuptable(map_t, map_dist_t);
            if(error) break;
            map_sub.shutdown();
            first_time = false;
        }
        if(sensor_data && !first_time) {
            odom_t = odom;
            if(init_sample) {
                if(nor_sampling) {
                    ROS_INFO("Sampling normal distribution ...");
                    normal_sample(map_t, S_t);
                }else {
                    ROS_INFO("Sampling uniform distribution ...");
                    uniform_sample(map_t, S_t);
                }
                ROS_INFO("Sampling done!");
                if(error) break;
                S_t_1 = S_t;
                odom_t_1 = odom_t;
                u_t[0] = odom_t;
                init_sample = false;
            }
            u_t[1] = odom_t;
            if(update_filter(u_t)) {
                if(agumented_mcl) {
                    AugmentedMCL(S_t_1, u_t, scan_t, map_t, S_t, scan_w_t, laser_pose, map_dist_t);
                }else {
                    KLDSamplingMCL(S_t_1, u_t, scan_t, map_t, S_t, scan_w_t, laser_pose, map_dist_t);
                }
                S_t_1.samples.clear();
                for(int i = 0; i < S_t.samples.size(); i++) {
                    S_t_1.samples.push_back(S_t.samples[i]);
                }
                S_t_1.mean = S_t.mean;
                S_t_1.cov = S_t.cov;
                u_t[0] = odom_t;
            }else{
                pf_vector_t u_t_w[2];
                u_t_w[0] = odom_t_1; u_t_w[1] = odom_t;
                update_motion(u_t_w, S_t);
            }
            if(entropy) computeEntropy(S_t);
            mcl_publisher(hypoth_pose_t, lz_pose, pcl_t, S_t, laser_pose, ls_scan_t, map_t);
            particlecloud_pub.publish(hypoth_pose_t);
            amcl_pose_pub.publish(lz_pose);
            point_cloud_pub.publish(pcl_t);

            odom_t_1 = odom_t;
        }
        rate.sleep();
    }
    return 0;
}