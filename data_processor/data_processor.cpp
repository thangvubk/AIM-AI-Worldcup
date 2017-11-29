#include <iostream>
#include "data_processor.hpp"

namespace aim {

data_processor::data_processor(bool is_debug) {
    this->is_debug = is_debug;
    this->is_first_frame_received = false;
    if (is_debug) {
        std::cout << "data processor constructed" << std::endl;
    }
}

data_processor::~data_processor() {
    if (this->is_debug) {
        std::cout << "data processor distroyed" << std::endl;
    }
}

void data_processor::update_cur_frame(const aiwc::frame& f){
    this->cur_frame = f;
}

const std::array<std::array<double, 3>, 5> data_processor::get_cur_postures(std::size_t team) {
    // get current postures including x, y, th of team (ours or opponent)
    // based on received frame

    std::array<std::array<double, 3>, 5> cur_postures;
    for (std::size_t id = 0; id < 5; ++id) {
        cur_postures[id][0]  = (*this->cur_frame.opt_coordinates).robots[team][id].x;
        cur_postures[id][1]  = (*this->cur_frame.opt_coordinates).robots[team][id].y;
        cur_postures[id][2] = (*this->cur_frame.opt_coordinates).robots[team][id].th;
    }

    return cur_postures;
}

const std::array<std::array<double, 3>, 5> data_processor::get_my_team_postures() {
    return this->get_cur_postures(0);
}

const std::array<std::array<double, 3>, 5> data_processor::get_opponent_postures() {
    return this->get_cur_postures(1);
}

const std::array<double, 2> data_processor::get_cur_ball_position() {
    const std::array<double, 2> pos_ball = { (*this->cur_frame.opt_coordinates).ball.x, (*this->cur_frame.opt_coordinates).ball.y };
    return pos_ball;
}

const std::array<double, 2> data_processor::get_ball_position(const aiwc::frame& f) {
    const std::array<double, 2> pos_ball = { (*f.opt_coordinates).ball.x, (*f.opt_coordinates).ball.y };
    return pos_ball;
}

const std::array<double, 2> data_processor::get_cur_ball_transition() {
    std::array<double, 2> transition;
    if (this->is_first_frame_received == false) {
        transition = {0, 0};
        this->is_first_frame_received = true;
    } else {
        const std::array<double, 2> cur_ball = this->get_ball_position(this->cur_frame);
        const std::array<double, 2> prev_ball = this->get_ball_position(this->prev_frame);
        transition = {cur_ball[0] - prev_ball[0], cur_ball[1] - prev_ball[1]};
    }

    this->prev_frame = this->cur_frame;

    if (this->is_debug) {
        std::cout << __func__ << ": transition (x, y) " << transition[0] << " " << transition[1] << std::endl;
    }
    return transition;
}
}