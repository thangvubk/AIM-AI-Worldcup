#include <iostream>
#include <array>

#include "calculator.hpp"

namespace aim {

// Constants implementation
const double calculator::PI = 3.1415926535;
const double calculator::TH_TOLERANCE = 0.05; //radian
const double calculator::DIST_TOLERANCE = 0.05;

calculator::calculator(bool is_debug) {
    // Contructer
    this->is_debug = is_debug;
    if (is_debug) {
        std::cout << "calculator constructed" << std::endl;
    }
}

calculator::~calculator() {
    //Deconstructor

    if (this->is_debug) {
        std::cout << "calculator distroyed" << std::endl;
    }
}

// util function
double calculator::d2r(double deg) {
    return deg * calculator::PI / 180;
}

double calculator::r2d(double rad) {
    return rad * 180 / calculator::PI;
}

double calculator::get_distance(const double x1, const double y1, const double x2, const double y2) {
    const double dx = x1 - x2;
    const double dy = y1 - y2;
    return std::sqrt(dx * dx + dy * dy);
}

double calculator::get_distance(const std::array<double, 2> current, const std::array<double, 2> target) {
    const double x1 = current[0];
    const double y1 = current[1];
    const double x2 = target[0];
    const double y2 = target[1];
    return this->get_distance(x1, y1, x2, y2);
}

double calculator::get_distance(const std::array<double, 3> cur_posture, const std::array<double, 3> tar_posture) {
    const std::array<double, 2> target = {tar_posture[0], tar_posture[1]};
    const std::array<double, 2> current = {cur_posture[0], cur_posture[1]};
    return this->get_distance(current, target);
}

/**
    check the different between cur_th and tar_th then normalize and compare with tolerance
    return true if cur_th approxly equals tar_th else false
*/
bool calculator::theta_equal_with_tolerance(const double cur_th, const double tar_th, const double tolerance) {
    double d_th = cur_th - tar_th;
    while (d_th > PI) d_th -= 2 * PI;
    while (d_th < -PI) d_th += 2 * PI;

    if (this->is_debug) {
        std::cout << __func__ << ": (d_th, tolerance)" << d_th << " " << tolerance << std::endl;
    }

    if (std::abs(d_th) < tolerance) {
        return true;
    }
    return false;
}

/**
    Compute static theta which base on coordinates only
    Args:
    - current (x, y)
    - target (x, y)

    return static_th in radian
*/
double calculator::compute_static_theta(const std::array<double, 2> current,
                                        const std::array<double, 2> target) {
    const double dx = target[0] - current[0];
    const double dy = target[1] - current[1];

    // compute desired theta, and handle conner cases
    double static_th;
    if (dx == 0 && dy == 0) {
        static_th = 0;
    } else if (dx == 0 && dy > 0) {
        static_th = calculator::PI / 2;
    } else if (dx == 0 && dy < 0) {
        static_th = - calculator::PI / 2;
    } else {
        static_th = std::atan2(dy, dx);
    }

    return static_th;
}

/**
    Compute theta to the target with is summation of cur_posture theta and theta computed
    from current (x, y) and target (x, y)

    args:
    - cur_posture: current (x, y, th)
    - target: target(x, y)

    return d_th in radian
*/
double calculator::compute_theta_to_target(const std::array<double, 3> cur_posture,
        const std::array<double, 2> target) {

    const std::array<double, 2> current = {cur_posture[0], cur_posture[1]};

    // compute desired theta, and handle conner cases
    const double static_th = this->compute_static_theta(current, target);

    double d_th = static_th - cur_posture[2];

    while (d_th > PI) d_th -= 2 * PI;
    while (d_th < -PI) d_th += 2 * PI;

    if (this->is_debug) {
        std::cout << __func__ << ": static theta, target theta " << static_th << " " << d_th << std::endl;
    }

    return d_th;
}

std::array<double, 3> calculator::compute_desired_posture(std::array<double, 2> ball_pstn, std::array<double, 2> goal_pstn){
    const double pad = 0.1; //the target for player is a litte bit behind the ball

    const double static_th = this->compute_static_theta(ball_pstn, goal_pstn);

    // Compute desired posture
    const double x = ball_pstn[0] - pad*std::cos(static_th); 
    const double y = ball_pstn[1] - pad*std::sin(static_th);
    const double th = static_th;

    return {x, y, th};
}

/**
    Check if the cur_posture is similar to the target posture with some tolerance
    args:
    - cur_posture: (x, y, th)
    - tar_posture: (x, y, th)

    return: true if similar else false
*/
bool calculator::is_desired_posture(std::array<double, 3> cur_posture, std::array<double, 3> tar_posture){
    if (this->get_distance(cur_posture, tar_posture) < calculator::DIST_TOLERANCE &&
        this->theta_equal_with_tolerance(cur_posture[2], tar_posture[2], calculator::TH_TOLERANCE) == true){
        return true;
    }
    return false;
}

} //aim namespace