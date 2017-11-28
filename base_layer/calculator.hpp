/**
    Author: Thang Vu
    Created date: 25/Nov/2017
    Discription: low-level function, control two wheels of a robots
*/

#ifndef _CALCULATOR_H
#define _CALCULATOR_H

#include <array>

namespace aim {

class calculator {
public:
    calculator(bool is_debug = false);
    ~calculator();

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

	std::array<double, 3> compute_desired_posture(std::array<double, 2> ball_pstn,
                                                  std::array<double, 2> goal_pstn);

    bool is_desired_posture(std::array<double, 3> cur_posture, 
                            std::array<double, 3> tar_posture);

    // Constants
    static const double PI;
    static const double TH_TOLERANCE;
    static const double DIST_TOLERANCE;

private:
    bool is_debug; 
};
}

#endif //_CALCULATOR_H