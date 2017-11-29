
#ifndef _DATA_PROCESSOR_H
#define _DATA_PROCESSOR_H
#include <iostream>
#include "ai_base.hpp"

namespace aim{
class data_processor{
public:
    data_processor(bool is_debug);
    ~data_processor();

    void update_cur_frame(const aiwc::frame& f);
    
    const std::array<std::array<double, 3>, 5> get_my_team_postures();

    const std::array<std::array<double, 3>, 5> get_opponent_postures();

    const std::array<double, 2> get_cur_ball_position();

    const std::array<double, 2> get_cur_ball_transition();

private:
    const std::array<std::array<double, 3>, 5> get_cur_postures(std::size_t team);

    const std::array<double, 2> get_ball_position(const aiwc::frame& f);

private:
    aiwc::frame cur_frame;
    aiwc::frame prev_frame; // use previous frame to caculate the velocity (transition/frame)
    bool is_first_frame_received;

    bool is_debug;
};
}

#endif //_DATA_PROCESSOR_H