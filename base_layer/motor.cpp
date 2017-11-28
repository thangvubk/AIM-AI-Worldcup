#include <iostream>
#include <array>
#include <string>
#include "motor.hpp"



namespace aim {

// cóntructor
motor::motor(std::string name, bool is_debug) {
    this->spin_ctrl = new pid_ctrler(0.25, 0.005, 0.01, 0.05, false); //args: kP, kI, kD, dt, is_debug
    this->cal = new calculator(false); // arg: is_debug
    this->cur_phase = 0;
    this->is_debug = is_debug;
    this->name = name;
    if (is_debug) {
        std::cout << this->name << " " << "motor constructed" << std::endl;
    }
}

// deconstructor
motor::~motor() {
    delete this->spin_ctrl;
    delete this->cal;
    if (this->is_debug) {
        std::cout << this->name << " " << "motor distroyed" << std::endl;
    }
}
/**
    spin the robot to theta, pid controller is used to control the velocity

    args:
    - cur_th: current theta in radian
    - tar_th: target theta in radian

    return: linear velocity of left and right wheels
*/
std::array<double, 2> motor::spin_to_theta(const double cur_th, const double tar_th) {

    std::array<double, 2> wheel_velos;

    // compute error and normalize from -PI to PI
    double err = tar_th - cur_th;
    while (err > calculator::PI) err -= 2 * calculator::PI;
    while (err < -calculator::PI) err += 2 * calculator::PI;

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
std::array<double, 2> motor::move_to_target(const std::array<double, 3> cur_posture,
        const std::array<double, 2> target,
        const double damping) {

    std::array<double, 2> wheel_velos;

    const double mult_lin = 2;
    const double mult_ang = 0.2;

    const std::array<double, 2> current = { cur_posture[0], cur_posture[1] };
    const double d_e = this->cal->get_distance(current, target);
    double d_th = this->cal->compute_theta_to_target(cur_posture, target);

    double ka;
    if (d_e > 1) {        ka = 17; }
    else if (d_e > 0.5) { ka = 19; }
    else if (d_e > 0.3) { ka = 21; }
    else if (d_e > 0.2) { ka = 23; }
    else               { ka = 25; }
    ka /= 90;

    int sign = 1;

    if (d_th > this->cal->d2r(95)) {
        d_th -= calculator::PI;
        sign = -1;
    }
    else if (d_th < this->cal->d2r(-95)) {
        d_th += calculator::PI;
        sign = -1;
    }

    if (std::abs(d_th) > this->cal->d2r(85)) {
        wheel_velos = { -mult_ang * d_th, mult_ang * d_th};
    }
    else {
        if (d_e < 5.0 && abs(d_th) < this->cal->d2r(40)) {
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
std::array<double, 2> motor::three_phase_move_to_target(std::array<double, 3> cur_posture,
        std::array<double, 3> tar_posture, double damping) {

    std::array<double, 2> wheel_velos = {0, 0};

    if (this->is_debug) {
        std::cout << this->name << " " << __func__ << ": cur posture " << cur_posture[0]
                  << " " << cur_posture[1] << " " << cur_posture[2] << std::endl;
        std::cout << this->name << " " << __func__ << ": tar posture " << tar_posture[0]
                  << " " << tar_posture[1] << " " << tar_posture[2] << std::endl;
    }

    //slice positions from  postures
    const std::array<double, 2> target = {tar_posture[0], tar_posture[1]};
    const std::array<double, 2> current = {cur_posture[0], cur_posture[1]};

    // FSM for 3-phase movement
    if (this->cur_phase == 0) { // phase1: spin by static theta
        const double static_th = this->cal->compute_static_theta(current, target);

        if (this->is_debug) {
            std::cout << this->name << " " << __func__ << ": Phase 0, tar theta " << static_th << std::endl;
        }

        const double cur_th = cur_posture[2];
        // spin if current theta is different from target theta else switch to phase 2
        if (this->cal->theta_equal_with_tolerance(cur_th, static_th, calculator::TH_TOLERANCE) == false) {
            if (this->is_debug) {
                std::cout << this->name << " " << __func__ << ": Phase 0, spin" << std::endl;
            }
            wheel_velos = this->spin_to_theta(cur_th, static_th);
        } else {
            if (this->is_debug) {
                std::cout << this->name << " " << __func__ << ": move to phase 1" << std::endl;
            }
            this->cur_phase = 1;
        }
    }

    if (this->cur_phase == 1) {
        const double dist = this->cal->get_distance(current, target);

        if (this->is_debug) {
            std::cout << this->name << " " << __func__ << ": Phase 1, distance " << dist << std::endl;
        }

        if (dist > calculator::DIST_TOLERANCE) {
            if (this->is_debug) {
                std::cout << this->name << " " << __func__ << ": Phase 1, move to target" << std::endl;
            }
            wheel_velos = this->move_to_target(cur_posture, target, damping);
        } else {
            if (this->is_debug) {
                std::cout << this->name << " " << __func__ << ": move to phase 2" << std::endl;
            }
            this->cur_phase = 2;
        }
    }

    if (this->cur_phase == 2) {
        double cur_th = cur_posture[2];
        double tar_th = tar_posture[2];
        if (this->cal->theta_equal_with_tolerance(cur_th, tar_th, calculator::TH_TOLERANCE) == false) {
            if (this->is_debug) {
                std::cout << this->name << " " << __func__ << ": Phase 2, spin" << std::endl;
            }
            wheel_velos = this->spin_to_theta(cur_th, tar_th);
        } else {
            if (this->is_debug) {
                this->cur_phase = 0;
                std::cout << this->name << " " << __func__ << ": Finish 3 phase movement, reset" << std::endl;
            }
        }
    }

    return wheel_velos;
}

// getters setters
std::size_t motor::get_cur_phase() {
    return this->cur_phase;
}

void motor::set_cur_phase(std::size_t cur_phase) {
    this->cur_phase = cur_phase;
}
}