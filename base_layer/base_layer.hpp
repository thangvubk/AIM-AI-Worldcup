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

	std::array<double, 3> get_velocity(const std::array<double, 3> curr_posture, const std::array<double, 3> past_posture);

	double get_bivariate_normal_pdf(const std::array<double, 2> x, const std::array<double, 2> mean, const std::array<double, 2> sigma);

	std::array<double, 2> get_avoid_point(const std::array<double, 3> curr_posture, const std::array<double, 2> tar_dest, const std::array<double, 3> oppn_posture, const std::array<double, 3> oppn_vel);

	std::array<double, 2> get_avoid_point(const std::array<double, 3> curr_posture, const std::array<double, 2> tar_dest,
		const std::array<std::array<double, 3>, 5> opnt_posture, const std::array<std::array<double, 3>, 5> opnt_vel);

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