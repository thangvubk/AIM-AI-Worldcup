#ifndef _STRATEGY_H
#define _STRATEGY_H

#include <array>
#include "data_processor/data_processor.hpp"
#include "base_layer/motor.hpp"
#include "base_layer/calculator.hpp"


namespace aim{

class strategy{
public:
    strategy(data_processor* data_proc, bool is_debug=false);
    ~strategy();

    std::array<std::array<double, 2>, 5> perform();

    std::array<double, 2> attack(std::size_t id, std::size_t closest_id, std::array<double, 3> default_posture);
    
    std::array<double, 2> upper_size_attack(std::size_t id, std::size_t closest_id);
    
    std::array<double, 2> lower_size_attack(std::size_t id, std::size_t closest_id);

    std::array<double, 2> defense(std::size_t id, std::size_t closest_id, std::array<double, 3> default_posture);
    
    std::array<double, 2> upper_size_defense(std::size_t id, std::size_t closest_id);
    
    std::array<double, 2> lower_size_defense(std::size_t id, std::size_t closest_id);

    std::array<double, 2> keep_goal(std::size_t id);

    std::size_t find_closest_robot();


private:
    bool is_debug;
    //std::array<array<double, 2>, 5> robot_wheels;
    std::array<motor*, 5> motors;
    calculator *cal;
    data_processor *data_proc;

    std::size_t goal_keeper, upper_defender, lower_defender, upper_attacker, lower_attacker;
};

}

#endif // _STRATEGY_H