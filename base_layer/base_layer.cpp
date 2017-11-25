#include <iostream>
#include <array>

#include "base_layer.hpp"

namespace aim {
static constexpr double PI = 3.1415926535;

base_layer::base_layer(bool is_debug) {
    // Contructer

    this->is_debug = is_debug;
    this->spin_ctrl = new pid_ctrler(0.25, 0.005, 0.01, 0.05, true);

    if (is_debug) {
        std::cout << "base_layer constructed" << std::endl;
    }
}

base_layer::~base_layer() {
    //Deconstructor

    delete this->spin_ctrl;

    if (this->is_debug) {
        std::cout << "base_layer distroyed" << std::endl;
    }
}

// util function
double d2r(double deg) {
    return deg * PI / 180;
}

double r2d(double rad) {
    return rad * 180 / PI;
}

std::array<double, 2> base_layer::spin_to_theta(const double cur_th, const double tar_th) {
    /**
        spin the robot to theta, pid controller is used to control the velocity

        args:
        - cur_th: current theta in radian
        - tar_th: target theta in radian

        return: linear velocity of left and right wheels
    */
    std::array<double, 2> wheel_velos;

    // compute error and normalize from -PI to PI
    double err = tar_th - cur_th;
    while (err > PI) err -= 2 * PI;
    while (err < -PI) err += 2 * PI;

    double velo = this->spin_ctrl->control(err);

    wheel_velos = { -velo, velo}; //left, right

    return wheel_velos;
}

std::array<double, 2> base_layer::move_to_target(const std::array<double, 3> cur_posture,
        const std::array<double, 2> target,
        const double damping) {
    /**
        move the robot to target

        args:
        - cur_posture: current x, y, theta
        - target: target x, y

        return: linear velocity of left and right wheels
    */

    std::array<double, 2> wheel_velos;

    const double mult_lin = 2;
    const double mult_ang = 0.2;

    const double dx = target[0] - cur_posture[0];
    const double dy = target[1] - cur_posture[1];

    const double d_e = std::sqrt(dx * dx + dy * dy);

    const double desired_th = (dx == 0 && dy == 0) ? (PI / 2) : std::atan2(dy, dx);

    double d_th = desired_th - cur_posture[2];
    while (d_th > PI) d_th -= 2 * PI;
    while (d_th < -PI) d_th += 2 * PI;

    double ka;
    if (d_e > 1) {        ka = 17; }
    else if (d_e > 0.5) { ka = 19; }
    else if (d_e > 0.3) { ka = 21; }
    else if (d_e > 0.2) { ka = 23; }
    else               { ka = 25; }
    ka /= 90;

    int sign = 1;

    if (d_th > d2r(95)) {
        d_th -= PI;
        sign = -1;
    }
    else if (d_th < d2r(-95)) {
        d_th += PI;
        sign = -1;
    }

    if (std::abs(d_th) > d2r(85)) {
        wheel_velos = { -mult_ang * d_th, mult_ang * d_th};
    }
    else {
        if (d_e < 5.0 && abs(d_th) < d2r(40)) {
            ka = 0.1;
        }
        ka *= 4;
        double left_velo = sign * (mult_lin * (1 / (1 + std::exp(-3 * d_e)) - damping) - mult_ang * ka * d_th);
        double right_velo = sign * (mult_lin * (1 / (1 + std::exp(-3 * d_e)) - damping) + mult_ang * ka * d_th);
        wheel_velos = {left_velo, right_velo};
    }

    return wheel_velos;
}
} //aim namespace