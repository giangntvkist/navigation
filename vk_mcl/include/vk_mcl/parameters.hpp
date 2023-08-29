#pragma once
#include "vk_mcl/vk_mcl.hpp"

/* Laser params */
double range_min, range_max, angle_increment, angle_min, angle_max;
double laser_pose_x, laser_pose_y, laser_pose_theta;
int throttle_scan;

/* Motion model params */
int model_type;
double anpha1, anpha2, anpha3, anpha4, anpha5;
double sigma_hit;

/* Sensor model params */
double z_hit, z_rand, z_max;
double max_beams, max_occ_dist;
double lamda_hit, lamda_short, lamda_max, lamda_rand;

/* Particles filter params */
int min_particles, max_particles;
double anpha_slow, anpha_fast;
double bin_size_x, bin_size_y, bin_size_theta;
double kld_eps, kld_delta;


/* Common params */
double min_trans, min_rot;
double init_cov_x, init_cov_y, init_cov_theta;
double init_pose_x, init_pose_y, init_pose_theta;

double converged_distance;
double beam_skip_threshold, beam_skip_distance, beam_skip_error_threshold;

double err_min, match_rate;
double frequency_publish;
bool save_last_pose, set_init_pose;

bool nor_sampling;
bool inverse_laser;
bool likelihoodfield;
bool agumented_mcl;
bool entropy;

string lookup_table_path, last_pose_path;
string base_frame, map_frame, base_scan_frame;