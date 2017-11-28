/**
    Author: Thang Vu
    Created date: 25/Nov/2017
    Discription: low-level function, control two wheels of a robots
*/

#ifndef _BASE_LAYER_H
#define _BASE_LAYER_H

#include <array>

#include "pid_ctrler.hpp"

namespace aim {

class base_layer {
public:
    base_layer(bool is_debug = false);
    ~base_layer();

	double d2r(double deg);
	
	double r2d(double rad);

    double get_distance(const double x1, const double x2, const double y1, const double y2);

    double get_distance(const std::array<double, 2> current, 
                        const std::array<double, 2> target);

    double get_distance(const std::array<double, 3> cur_posture, 
                        const std::array<double, 3> tar_posture);

    bool theta_equal_with_tolerance(const double cur_th, const double tar_th,
                                      const double tolerance);

    double compute_static_theta(const std::array<double, 2> current, 
                                const std::array<double, 2> target);

    double compute_theta_to_target(const std::array<double, 3> cur_posture,
                                   const std::array<double, 2> target);

    std::array<double, 2> spin_to_theta(double cur_th, double tar_th);

    std::array<double, 2> move_to_target(const std::array<double, 3> cur_posture,
                                         const std::array<double, 2> target,
                                         const double damping = 0.45);

    std::array<double, 2> three_phase_move_to_target(const std::array<double, 3> cur_posture,
                                                     const std::array<double, 3> tar_posture,
                                                     const double damping = 0.45);

    // getters and setters
    std::size_t base_layer::get_cur_phase();

    void base_layer::set_cur_phase(std::size_t cur_phase);



private:
    bool is_debug;
    pid_ctrler *spin_ctrl;
    std::size_t cur_phase; 

    // Constants
    static const double PI;
    static const double TH_TOLERANCE;
    static const double DIST_TOLERANCE;
};
}

#endif //_BASE_LAYER_H