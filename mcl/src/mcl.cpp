#include "mcl/mcl.hpp"
#include "mcl/motion_model.hpp"
#include "mcl/sensor_model.hpp"
#include "mcl/particle_filter.hpp"
#include "mcl/map.hpp"
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

    if(!ros::param::get("~range_min", range_min)) range_min = 0.05;
    if(!ros::param::get("~range_max", range_max)) range_max = 25.0;
    if(!ros::param::get("~angle_min", angle_min)) angle_min = -M_PI/2;
    if(!ros::param::get("~angle_max", angle_max)) angle_max = M_PI/2;
    if(!ros::param::get("~angle_increment", angle_increment)) angle_increment = M_PI/(3*180);

    if(!ros::param::get("~laser_pose_x", laser_pose_x)) laser_pose_x = 0.0;
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
    if(!ros::param::get("~init_cov_theta", init_cov_theta)) init_cov_theta = M_PI*M_PI;

    if(!ros::param::get("~frequency_publish", frequency_publish)) frequency_publish = 20.0;
    
    if(!ros::param::get("~init_pose_x", init_pose_x)) init_pose_x = 0.0;
    if(!ros::param::get("~init_pose_y", init_pose_y)) init_pose_y = 0.0;
    if(!ros::param::get("~init_pose_theta", init_pose_theta)) init_pose_theta = 0.0;
    if(!ros::param::get("~range_sample", range_sample)) range_sample = 5.0;

    if(!ros::param::get("~err_min", err_min)) err_min = 0.05;
    if(!ros::param::get("~match_rate", match_rate)) match_rate = 0.80;

    if(!ros::param::get("~likelihoodfield", likelihoodfield)) likelihoodfield = true;
    if(!ros::param::get("~lookup_table", lookup_table)) lookup_table = true;
    if(!ros::param::get("~lookup_table_path", lookup_table_path)) lookup_table_path = "";
    if(!ros::param::get("~last_pose_path", last_pose_path)) last_pose_path = "";

    if(!ros::param::get("~uniform_pdf_submap", uniform_pdf_submap)) uniform_pdf_submap = true;
    if(!ros::param::get("~uniform_pdf_allmap", uniform_pdf_allmap)) uniform_pdf_allmap = false;
    if(!ros::param::get("~save_last_pose", save_last_pose)) save_last_pose = false;
    if(!ros::param::get("~set_init_pose", set_init_pose)) set_init_pose = false;

    if(!ros::param::get("~inverse_laser", inverse_laser)) inverse_laser = true;
    if(!ros::param::get("~agumented_mcl", agumented_mcl)) agumented_mcl = true;

    if(!ros::param::get("~base_frame", base_frame)) base_frame = "base_link";
    if(!ros::param::get("~map_frame", map_frame)) map_frame = "map";
    if(!ros::param::get("~entropy", entropy)) entropy = false;

    pf_set_sample_t set_particle_t_1;
    pf_vector_t odom_t_1, odom_t;
    pf_vector_t u_t[2];
    bool _init_sample = true;

    geometry_msgs::PoseWithCovarianceStamped lz_pose;
    geometry_msgs::PoseArray hypoth_pose;
    sensor_msgs::PointCloud pcl;

    ros::Rate rate(frequency_publish);
    while(ros::ok()) {
        ros::spinOnce();
        if(_map && first_time) {
            get_lookuptable(occ_map);
            if(error_) break;
            map_sub.shutdown();
            first_time = false;
        }
        if(!first_time && _data) {
            odom_t.v[0] = odom.pose.pose.position.x;
            odom_t.v[1] = odom.pose.pose.position.y;
            odom_t.v[2] = tf::getYaw(odom.pose.pose.orientation);
            if(_init_sample) {
                if(uniform_pdf_allmap) {
                    ROS_INFO("Sampling uniform distribution!");
                    uniform_sample_allmap(occ_map);
                }else if(uniform_pdf_submap) {
                    ROS_INFO("Sampling uniform distribution submap!");
                    uniform_sample_submap(occ_map);
                }else {
                    ROS_INFO("Sampling normal distribution!");
                    normal_sample(occ_map);
                }
                if(error_) break;
                set_particle_t_1.samples.clear();
                set_particle_t_1 = set_particle_t;
                odom_t_1 = odom_t;
                u_t[0] = odom_t;
                _init_sample = false;
            }
            u_t[1] = odom_t;
            if(update_filter(u_t)) {
                if(agumented_mcl) {
                    AugmentedMCL(set_particle_t_1, u_t, laser_scan, occ_map);
                }else {
                    KLDSamplingMCL(set_particle_t_1, u_t, laser_scan, occ_map);
                }
                set_particle_t_1.samples.clear();
                set_particle_t_1 = set_particle_t;
                u_t[0] = odom_t;
            }else{
                pf_vector_t u_t_[2];
                u_t_[0] = odom_t_1; u_t_[1] = odom_t;
                update_motion(u_t_);
            }
            if(entropy) computeEntropy();
            mcl_publisher(hypoth_pose, lz_pose, pcl, set_particle_t, set_scan_t, occ_map);
            particlecloud_pub.publish(hypoth_pose);
            amcl_pose_pub.publish(lz_pose);
            point_cloud_pub.publish(pcl);
            odom_t_1 = odom_t;
        }
        rate.sleep();
    }
    return 0;
}