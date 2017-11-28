#include <iostream>
#include <array>

#include "base_layer.hpp"

namespace aim {

// Constants implementation
const double base_layer::PI = 3.1415926535;
const double base_layer::TH_TOLERANCE = 0.05; //radian
const double base_layer::DIST_TOLERANCE = 0.05;

base_layer::base_layer(bool is_debug) {
    // Contructer

    this->is_debug = is_debug;
    this->spin_ctrl = new pid_ctrler(0.25, 0.005, 0.01, 0.05, true);
    this->cur_phase = 0;

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
double base_layer::d2r(double deg) {
    return deg * base_layer::PI / 180;
}

double base_layer::r2d(double rad) {
    return rad * 180 / base_layer::PI;
}

double base_layer::get_distance(const double x1, const double y1, const double x2, const double y2) {
    const double dx = x1 - x2;
    const double dy = y1 - y2;
    return std::sqrt(dx * dx + dy * dy);
}

double base_layer::get_distance(const std::array<double, 2> current, const std::array<double, 2> target) {
    const double x1 = current[0];
    const double y1 = current[1];
    const double x2 = target[0];
    const double y2 = target[1];
    return this->get_distance(x1, y1, x2, y2);
}

double base_layer::get_distance(const std::array<double, 3> cur_posture, const std::array<double, 3> tar_posture) {
    const std::array<double, 2> target = {tar_posture[0], tar_posture[1]};
    const std::array<double, 2> current = {cur_posture[0], cur_posture[1]};
    return this->get_distance(current, target);
}


/**
    check the different between cur_th and tar_th then normalize and compare with tolerance
    return true if cur_th approxly equals tar_th else false
*/
bool base_layer::theta_equal_with_tolerance(const double cur_th, const double tar_th, const double tolerance) {
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
    compute static theta which base on coordinates only
    Args:
    - current (x, y)
    - target (x, y)

    return static_th in radian
*/
double base_layer::compute_static_theta(const std::array<double, 2> current,
                                        const std::array<double, 2> target) {
    const double dx = target[0] - current[0];
    const double dy = target[1] - current[1];

    // compute desired theta, and handle conner cases
    double static_th;
    if (dx == 0 && dy == 0) {
        static_th = 0;
    } else if (dx == 0 && dy > 0) {
        static_th = base_layer::PI / 2;
    } else if (dx == 0 && dy < 0) {
        static_th = - base_layer::PI / 2;
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
double base_layer::compute_theta_to_target(const std::array<double, 3> cur_posture,
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

/**
    spin the robot to theta, pid controller is used to control the velocity

    args:
    - cur_th: current theta in radian
    - tar_th: target theta in radian

    return: linear velocity of left and right wheels
*/
std::array<double, 2> base_layer::spin_to_theta(const double cur_th, const double tar_th) {

    std::array<double, 2> wheel_velos;

    // compute error and normalize from -PI to PI
    double err = tar_th - cur_th;
    while (err > PI) err -= 2 * PI;
    while (err < -PI) err += 2 * PI;

    double velo = this->spin_ctrl->control(err);

    wheel_velos = { -velo, velo}; //left, right

    return wheel_velos;
}

/**
    move the robot to target

    args:
    - cur_posture: current x, y, theta
    - target: target x, y

    return: linear velocity of left and right wheels
*/
std::array<double, 2> base_layer::move_to_target(const std::array<double, 3> cur_posture,
        const std::array<double, 2> target,
        const double damping) {

    std::array<double, 2> wheel_velos;

    const double mult_lin = 2;
    const double mult_ang = 0.2;

    const std::array<double, 2> current = { cur_posture[0], cur_posture[1] };
    const double d_e = this->get_distance(current, target);
    double d_th = this->compute_theta_to_target(cur_posture, target);

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

/**
    Move to target (x, y, th) with three phase (spin, move, spin)
    An FSM is used with this->cur_phase being the current state
*/
std::array<double, 2> base_layer::three_phase_move_to_target(std::array<double, 3> cur_posture,
        std::array<double, 3> tar_posture, double damping) {

    std::array<double, 2> wheel_velos = {0, 0};

    if (this->is_debug) {
        std::cout << __func__ << ": cur posture " << cur_posture[0] << " " << cur_posture[1] << " " << cur_posture[2] << std::endl;
        std::cout << __func__ << ": tar posture " << tar_posture[0] << " " << tar_posture[1] << " " << tar_posture[2] << std::endl;
    }

    //slice positions from  postures
    const std::array<double, 2> target = {tar_posture[0], tar_posture[1]};
    const std::array<double, 2> current = {cur_posture[0], cur_posture[1]};

    // FSM for 3-phase movement
    if (this->cur_phase == 0) { // phase1: spin by static theta
        const double static_th = this->compute_static_theta(current, target);

        if (this->is_debug) {
            std::cout << __func__ << ": Phase 0, tar theta " << static_th << std::endl;
        }

        const double cur_th = cur_posture[2];
        // spin if current theta is different from target theta else switch to phase 2
        if (this->theta_equal_with_tolerance(cur_th, static_th, base_layer::TH_TOLERANCE) == false) {
            if (this->is_debug) {
                std::cout << __func__ << ": Phase 0, spin" << std::endl;
            }
            wheel_velos = this->spin_to_theta(cur_th, static_th);
        } else {
            if (this->is_debug) {
                std::cout << __func__ << ": move to phase 1" << std::endl;
            }
            this->cur_phase = 1;
        }
    }

    if (this->cur_phase == 1) {
        const double dist = this->get_distance(current, target);

        if (this->is_debug) {
            std::cout << __func__ << ": Phase 1, distance " << dist << std::endl;
        }

        if (dist > base_layer::DIST_TOLERANCE) {
            if (this->is_debug) {
                std::cout << __func__ << ": Phase 1, move to target" << std::endl;
            }
            wheel_velos = this->move_to_target(cur_posture, target, damping);
        } else {
            if (this->is_debug) {
                std::cout << __func__ << ": move to phase 2" << std::endl;
            }
            this->cur_phase = 2;
        }
    }

    if (this->cur_phase == 2) {
        double cur_th = cur_posture[2];
        double tar_th = tar_posture[2];
        if (this->theta_equal_with_tolerance(cur_th, tar_th, base_layer::TH_TOLERANCE) == false) {
            if (this->is_debug) {
                std::cout << __func__ << ": Phase 2, spin" << std::endl;
            }
            wheel_velos = this->spin_to_theta(cur_th, tar_th);
        } else {
            if (this->is_debug) {
                std::cout << __func__ << ": Finish 3 phase movement" << std::endl;
            }
        }
    }

    return wheel_velos;
}

// getters setters
std::size_t base_layer::get_cur_phase(){
    return this->cur_phase;
}

void base_layer::set_cur_phase(std::size_t cur_phase){
    this->cur_phase = cur_phase;
}

} //aim namespace