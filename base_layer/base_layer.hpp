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
    std::array<double, 2> spin_to_theta(double cur_th, double tar_th);
    std::array<double, 2> move_to_target(const std::array<double, 3> cur_posture,
                                         const std::array<double, 2> target,
                                         const double damping = 0.45);



private:
    bool is_debug;
    pid_ctrler *spin_ctrl;
    //std::array<double, 2> wheel_velos; // left and right velocity
};
}

#endif //_BASE_LAYER_H