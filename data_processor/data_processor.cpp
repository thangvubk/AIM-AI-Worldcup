#include <iostream>
#include "data_processor.hpp"

namespace aim {

data_processor::data_processor(bool is_debug) {
    this->is_debug = is_debug;
    if (is_debug) {
        std::cout << "data processor constructed" << std::endl;
    }
}

data_processor::~data_processor() {
    if (this->is_debug) {
        std::cout << "data processor distroyed" << std::endl;
    }
}

const std::array<std::array<double, 3>, 5> data_processor::get_cur_postures(const aiwc::frame& f, std::size_t team) {
    // get current postures including x, y, th of team (ours or opponent)
    // based on received frame

    std::array<std::array<double, 3>, 5> cur_postures;
    for (std::size_t id = 0; id < 5; ++id) {
        cur_postures[id][0]  = (*f.opt_coordinates).robots[team][id].x;
        cur_postures[id][1]  = (*f.opt_coordinates).robots[team][id].y;
        cur_postures[id][2] = (*f.opt_coordinates).robots[team][id].th;
    }

    return cur_postures;
}

const std::array<std::array<double, 3>, 5> data_processor::get_my_team_postures(const aiwc::frame& f) {
    return this->get_cur_postures(f, 0);
}

const std::array<std::array<double, 3>, 5> data_processor::get_opponent_postures(const aiwc::frame& f) {
    return this->get_cur_postures(f, 1);
}
const std::array<double, 2> data_processor::get_cur_ball_position(const aiwc::frame& f) {
    const std::array<double, 2> pos_ball = { (*f.opt_coordinates).ball.x, (*f.opt_coordinates).ball.y };
    return pos_ball;
}
}