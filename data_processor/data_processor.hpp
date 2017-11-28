
#ifndef _DATA_PROCESSOR_H
#define _DATA_PROCESSOR_H
#include <iostream>
#include "ai_base.hpp"

namespace aim{
class data_processor{
public:
    data_processor(bool is_debug);
    ~data_processor();

    
    const std::array<std::array<double, 3>, 5> get_my_team_postures(const aiwc::frame& f);

    const std::array<std::array<double, 3>, 5> get_opponent_postures(const aiwc::frame& f);

    const std::array<double, 2> get_cur_ball_position(const aiwc::frame& f);

    //const std::array<double, 2> get_cur_ball_transition(const aiwc::frame &f);

private:
    const std::array<std::array<double, 3>, 5> get_cur_postures(const aiwc::frame& f, std::size_t team);

private:
    aiwc::frame previous_frame; // use previous frame to caculate the velocity (transition/frame)
    bool is_debug;
};
}

#endif //_DATA_PROCESSOR_H