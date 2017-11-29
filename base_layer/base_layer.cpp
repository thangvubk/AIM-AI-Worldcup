#include <iostream>
#include <array>
#include <random>
#include <valarray>

#include "base_layer.hpp"

namespace aim {

// Constants implementation
const double base_layer::PI = 3.1415926535;
const double base_layer::TH_TOLERANCE = 0.05; //radian
const double base_layer::DIST_TOLERANCE = 0.05;

base_layer::base_layer(bool is_debug) {
    // Contructer

    this->is_debug = is_debug;
    this->spin_ctrl = new pid_ctrler(0.25, 0.005, 0.01);
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

// code by jykim
std::array<double, 3> base_layer::get_velocity(const std::array<double, 3> curr_posture, const std::array<double, 3> past_posture) {
	std::array<double, 3> velocity;

	const double dt = 0.05;
	const double x_curr = curr_posture[0];
	const double y_curr = curr_posture[1];
	const double x_past = past_posture[0];
	const double y_past = past_posture[1];
	const double x_vel = (x_curr - x_past) / dt;
	const double y_vel = (y_curr - y_past) / dt;

	velocity = { x_vel, y_vel, std::sqrt(x_vel*x_vel + y_vel * y_vel) };
	return velocity;
}

double base_layer::get_bivariate_normal_pdf(const std::array<double, 2> x, const std::array<double, 2> mean, const std::array<double, 2> sigma) {
	return 1 / (2 * base_layer::PI * sigma[0] * sigma[1]) * std::exp(-0.5 * ((x[0] - mean[0])*(x[0] - mean[0])/(sigma[0] * sigma[0]) + (x[1] - mean[1])*(x[1] - mean[1])/(sigma[1] * sigma[1])));
}

std::array<double, 2> base_layer::get_avoid_point(const std::array<double, 3> curr_posture, const std::array<double, 2> tar_dest, const std::array<double, 3> oppn_posture, const std::array<double, 3> oppn_vel) {
	const double dt = 0.05; // time between adjacent frames
	const double threshold = 0.05;
	const std::array<double, 2> sigma = { 0.2, 0.2 };
	
	const std::array<double, 2> pred_oppn_next_posture = { oppn_posture[0] + oppn_vel[0] * dt, oppn_posture[1] + oppn_vel[1] * dt };

	const double dx = tar_dest[0] - curr_posture[0];
	const double dy = tar_dest[1] - curr_posture[1];
	
	std::array<double, 50> path_potential;

	for (double i = 1.0; i < 51.0; i++) {
		path_potential[i - 1] = get_bivariate_normal_pdf({ curr_posture[0] + i * dx / 51.0, curr_posture[1] + i * dy / 51.0 }, pred_oppn_next_posture, sigma);
		if (path_potential[i - 1] > 1) {
		}
	}
	double max = -1.0;
	int max_index;
	for (int i = 0; i < 50; i++) {
		if (path_potential[i] > max) {
			max = path_potential[i];
			max_index = i;
		}
	}
	std::cout << "max is    " << max << "      max index is     " << max_index << std::endl;

	std::array<double, 50> normal_potential;
	if (max < threshold) {
		return { -100, -100 };
	} else {
		for (int i = -25; i < 25; i++) {
			normal_potential[i + 25] = get_bivariate_normal_pdf({ curr_posture[0] + max_index * dx / 51.0 + i * dy / 51.0, curr_posture[1] + max_index * dy / 51.0 - i * dx / 51.0 }, pred_oppn_next_posture, sigma);
		}
	}
	double max_normal = -1;
	int max_index_normal = -1;
	for (int i = 0; i < 50; i++) {
		if ((normal_potential[i] > max_normal) && (normal_potential[i] < threshold)) {
			max_normal = normal_potential[i];
			max_index_normal = i;
		}
	}
	if (max_index_normal == -1) {
		return { -1.3, 0 };
	}
	else {
		return { curr_posture[0] + max_index * dx / 51.0 + max_index_normal * dy / 51.0, curr_posture[1] + max_index * dy / 51.0 - max_index_normal * dx / 51.0 };
	}
}

std::array<double, 2> base_layer::get_avoid_point(const std::array<double, 3> curr_posture, const std::array<double, 2> tar_dest,
	const std::array<std::array<double, 3>, 5> opnt_posture, const std::array<std::array<double, 3>, 5> opnt_vel) {
	const double dt = 0.05; // time between adjacent frames
	const double threshold = 0.05;
	const double sigma = 0.2;
	//const std::array<double, 2> sigma_0 = { sigma * std::abs(opnt_vel[0][0]), sigma * std::abs(opnt_vel[0][1]) };
	//const std::array<double, 2> sigma_1 = { sigma * std::abs(opnt_vel[1][0]), sigma * std::abs(opnt_vel[1][1]) };
	//const std::array<double, 2> sigma_2 = { sigma * std::abs(opnt_vel[2][0]), sigma * std::abs(opnt_vel[2][1]) };
	//const std::array<double, 2> sigma_3 = { sigma * std::abs(opnt_vel[3][0]), sigma * std::abs(opnt_vel[3][1]) };
	//const std::array<double, 2> sigma_4 = { sigma * std::abs(opnt_vel[4][0]), sigma * std::abs(opnt_vel[4][1]) };
	const std::array<double, 2> sigma_0 = { 0.2, 0.2 };
	const std::array<double, 2> sigma_1 = { 0.2, 0.2 };
	const std::array<double, 2> sigma_2 = { 0.2, 0.2 };
	const std::array<double, 2> sigma_3 = { 0.2, 0.2 };
	const std::array<double, 2> sigma_4 = { 0.2, 0.2 };

	const std::array<double, 2> mean_0 = { opnt_posture[0][0] + opnt_vel[0][0] * dt, opnt_posture[0][1] + opnt_vel[0][1] * dt };
	const std::array<double, 2> mean_1 = { opnt_posture[1][0] + opnt_vel[1][0] * dt, opnt_posture[1][1] + opnt_vel[1][1] * dt };
	const std::array<double, 2> mean_2 = { opnt_posture[2][0] + opnt_vel[2][0] * dt, opnt_posture[2][1] + opnt_vel[2][1] * dt };
	const std::array<double, 2> mean_3 = { opnt_posture[3][0] + opnt_vel[3][0] * dt, opnt_posture[3][1] + opnt_vel[3][1] * dt };
	const std::array<double, 2> mean_4 = { opnt_posture[4][0] + opnt_vel[4][0] * dt, opnt_posture[4][1] + opnt_vel[4][1] * dt };

	const double dx = tar_dest[0] - curr_posture[0];
	const double dy = tar_dest[1] - curr_posture[1];

	std::array<double, 50> path_potential;

	for (double i = 1.0; i < 51.0; i++) {
		path_potential[i - 1] = 0.2 * get_bivariate_normal_pdf({ curr_posture[0] + i * dx / 51.0, curr_posture[1] + i * dy / 51.0 }, mean_0, sigma_0)
			+ 0.2 * get_bivariate_normal_pdf({ curr_posture[0] + i * dx / 51.0, curr_posture[1] + i * dy / 51.0 }, mean_1, sigma_1)
			+ 0.2 * get_bivariate_normal_pdf({ curr_posture[0] + i * dx / 51.0, curr_posture[1] + i * dy / 51.0 }, mean_2, sigma_2)
			+ 0.2 * get_bivariate_normal_pdf({ curr_posture[0] + i * dx / 51.0, curr_posture[1] + i * dy / 51.0 }, mean_3, sigma_3)
			+ 0.2 * get_bivariate_normal_pdf({ curr_posture[0] + i * dx / 51.0, curr_posture[1] + i * dy / 51.0 }, mean_4, sigma_4); 
	}

	double max = -1.0;
	int max_index = -1;
	for (int i = 0; i < 50; i++) {
		if (path_potential[i] > max) {
			max = path_potential[i];
			max_index = i;
		}
	}

	std::array<double, 50> normal_potential;
	if (max < threshold) {
		return { -100, -100 };
	}
	else {
		for (int i = -25; i < 25; i++) {
			normal_potential[i + 25] = 0.2 * get_bivariate_normal_pdf({ curr_posture[0] + max_index * dx / 51.0 + i * dy / 51.0, curr_posture[1] + max_index * dy / 51.0 - i * dx / 51.0 }, mean_0, sigma_0)
				+ 0.2 * get_bivariate_normal_pdf({ curr_posture[0] + max_index * dx / 51.0 + i * dy / 51.0, curr_posture[1] + max_index * dy / 51.0 - i * dx / 51.0 }, mean_1, sigma_1)
				+ 0.2 * get_bivariate_normal_pdf({ curr_posture[0] + max_index * dx / 51.0 + i * dy / 51.0, curr_posture[1] + max_index * dy / 51.0 - i * dx / 51.0 }, mean_2, sigma_2)
				+ 0.2 * get_bivariate_normal_pdf({ curr_posture[0] + max_index * dx / 51.0 + i * dy / 51.0, curr_posture[1] + max_index * dy / 51.0 - i * dx / 51.0 }, mean_3, sigma_3)
				+ 0.2 * get_bivariate_normal_pdf({ curr_posture[0] + max_index * dx / 51.0 + i * dy / 51.0, curr_posture[1] + max_index * dy / 51.0 - i * dx / 51.0 }, mean_4, sigma_4);
		}
	}
	double max_normal = -1;
	int max_index_normal = -1;
	for (int i = 0; i < 50; i++) {
		if ((normal_potential[i] > max_normal) && (normal_potential[i] < threshold)) {
			max_normal = normal_potential[i];
			max_index_normal = i;
		}
	}
	if (max_index_normal == -1) {
		return { -1.3, 0 };
	}
	else {
		return { curr_posture[0] + max_index * dx / 51.0 + max_index_normal * dy / 51.0, curr_posture[1] + max_index * dy / 51.0 - max_index_normal * dx / 51.0 };
	}
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
        if (std::abs(cur_th - static_th) > base_layer::TH_TOLERANCE) {
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
        if (std::abs(cur_th - tar_th) > base_layer::TH_TOLERANCE) {
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

} //aim namespace