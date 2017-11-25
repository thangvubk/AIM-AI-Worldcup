#include <iostream>
#include <vector>
#include "pid_ctrler.hpp"

namespace aim {

	// methods implementation

	pid_ctrler::pid_ctrler(double kP, double kI, double kD, double dt, bool is_debug) {
		if (is_debug) {
			std::cout << "PID constructed." << std::endl;
			std::cout << "kP: " << kP << " kI: " << kI << " kD " << kD << std::endl;
		}
		this->kP = kP;
		this->kI = kI;
		this->kD = kD;
		this->dt = dt;
		this->is_debug = is_debug;
	}

	pid_ctrler::~pid_ctrler() {
		if (this->is_debug) {
			std::cout << "PID distroyed" << std::endl;
		}
	}

	double pid_ctrler::control(double err) {
		// Based on the err return the control signal

		this->sum_err += err;

		double P = this->kP * err;
		double I = this->kI * this->sum_err * this->dt;
		double D = this->kD * (err - this->prev_err) / this->dt;
		double PID = P + I + D;

		if (this->is_debug) {
			std::cout << "P: " << P << std::endl;
			std::cout << "I: " << I << std::endl;
			std::cout << "D: " << D << std::endl;
			std::cout << "PID: " << PID << std::endl;
		}

		this->prev_err = err;

		return PID;

	}
}