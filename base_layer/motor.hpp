/**
    Author: Thang Vu
    Created date: 25/Nov/2017
    Discription: low-level function, control two wheels of a robots
*/

#ifndef _MOTOR_H
#define _MOTOR_H

#include <array>

#include "pid_ctrler.hpp"
#include "calculator.hpp"

namespace aim {

class motor {
public:
    motor(std::string name, bool is_debug = false);
    ~motor();

    std::array<double, 2> spin_to_theta(double cur_th, double tar_th);

    std::array<double, 2> move_to_target(const std::array<double, 3> cur_posture,
                                         const std::array<double, 2> target,
                                         const double damping = 0.45);

    std::array<double, 2> three_phase_move_to_target(const std::array<double, 3> cur_posture,
                                                     const std::array<double, 3> tar_posture,
                                                     const double damping = 0.45);

    // getters and setters
    std::size_t get_cur_phase();

    void set_cur_phase(std::size_t cur_phase);

private:
    bool is_debug;
    std::string name;
    pid_ctrler *spin_ctrl;
    calculator *cal;

    std::size_t cur_phase; 
};
}

#endif //_MOTOR_H