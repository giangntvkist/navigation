#pragma once
#include <iostream>
struct pf_vector_t {
    double v[3];
};

struct pf_matrix_t {
    double m[3][3];
};

struct pf_sample_t {
    pf_vector_t pose;
    double weight;
};

struct pf_set_sample_t {
    int sample_count;
    pf_sample_t *samples;
    pf_vector_t mean;
    pf_matrix_t cov;
};

pf_vector_t pf_vector_sub(pf_vector_t a, pf_vector_t b);

inline double normalize (double z);
inline double angle_diff (double a, double b);

inline double normal_pdf(double mean, double sigma);
inline double uniform_pdf(double a, double b);

struct map_cell_t {
    int occ_state;
    double occ_dist;
};

struct map_t {
    double origin_x, origin_y;
    double resolution;
    int width, height;
    map_cell_t *cells;
    double max_occ_dist;
};

#define MAP_VALID(map, i, j) ((i >= 0) && (i < map->height) && (j >= 0) && (j < map->width))
#define MAP_IDEX(map, i, j) (i*map->width + j)

#define MAP_WXGX(map, j) (map->origin_x + j*map->resolution)
#define MAP_WYGY(map, i) (map->origin_y + i*map->resolution)

#define MAP_GXWX(map, x) ((x - map->origin_x)/map->resolution)
#define MAP_GYWY(map, y) ((y - map->origin_y)/map->resolution)


