#include <iostream>
#include <string>
#include "strategy.hpp"

namespace aim {

// Dimensions
constexpr double FIELD_LENGTH = 2.2;
constexpr double FIELD_WIDTH  = 1.8;
constexpr double GOAL_DEPTH   = 0.15;
constexpr double GOAL_WIDTH   = 0.4;
constexpr double PENALTY_AREA_DEPTH = 0.35;
constexpr double PENALTY_AREA_WIDTH = 0.8;

constexpr double BALL_RADIUS = 0.02135;
constexpr double ROBOT_SIZE = 0.075;
constexpr double AXLE_LENGTH = 0.07;
constexpr double WHEEL_RADIUS = 0.03;

strategy::strategy(data_processor* data_proc, bool is_debug) {
    this->is_debug = is_debug;
    if (this->is_debug) {
        std::cout << "strategy constructed" << std::endl;
    }

    this->cal = new calculator(false);
    this->data_proc = data_proc;

    motors[0] = new motor("motor 0", false);
    motors[1] = new motor("motor 1", false);
    motors[2] = new motor("motor 2", false);
    motors[3] = new motor("motor 3", false);
    motors[4] = new motor("motor 4", false);

    this->goal_keeper = 4;
    this->upper_attacker = 1;
    this->lower_attacker = 0;
    this->upper_defender = 3;
    this->lower_defender = 2;
}

strategy::~strategy() {
    if (this->is_debug) {
        std::cout << "strategy distroyed" << std::endl;
    }
}

std::array<std::array<double, 2>, 5> strategy::perform(){
    std::array<std::array<double, 2>, 5> robot_wheels;
    std::size_t closest_id = this->find_closest_robot();
    robot_wheels[this->upper_attacker] = this->upper_size_attack(this->upper_attacker, closest_id);
    robot_wheels[this->lower_attacker] = this->lower_size_attack(this->lower_attacker, closest_id);
    robot_wheels[this->upper_defender] = this->upper_size_defense(this->upper_defender, closest_id);
    robot_wheels[this->lower_defender] = this->lower_size_defense(this->lower_defender, closest_id);
    robot_wheels[this->goal_keeper] = this->keep_goal(this->goal_keeper);
    return robot_wheels;
}

std::array<double, 2> strategy::attack(std::size_t id, std::size_t closest_id, std::array<double, 3> default_posture) {

    std::array<double, 2> wheel_velos;

    std::array<std::array<double, 3>, 5> our_postures = this->data_proc->get_my_team_postures();
    std::array<double, 3> my_posture = our_postures[id];
    std::array<double, 2> cur_ball = this->data_proc->get_cur_ball_position();

    // if (id == closest_id) {
    //     std::array<double, 2> cur_trans = this->data_proc->get_cur_ball_transition();
    //     std::array<double, 2> goal_pstn = {1.1 + 0.75, 0}; // middle point of the net
    //     std::array<double, 3> tar_posture = this->cal->compute_desired_posture(cur_ball, cur_trans, goal_pstn);
    //     if (this->cal->is_desired_posture(my_posture, tar_posture) == false) {
    //         wheel_velos = this->motors[id]->three_phase_move_to_target(my_posture, tar_posture);
    //     } else {
    //         wheel_velos = this->motors[id]->move_to_target(my_posture, goal_pstn, 0);
    //     }
    // } else {
    //     wheel_velos = this->motors[id]->three_phase_move_to_target(my_posture, default_posture);
    // }

    if (cur_ball[0] > -0.1){ // little margin
        double ox = 0.2;
		double oy = 0.2;
        std::array<double, 2> cur_trans = this->data_proc->get_cur_ball_transition();
        std::array<double, 2> goal_pstn = {1.1 + 0.75, 0}; // middle point of the net
        std::array<double, 3> tar_posture = this->cal->compute_desired_posture(cur_ball, cur_trans, goal_pstn);
        //if(my_posture[0] < cur_ball[0] && my_posture[0] > cur_ball[0] - ox &&
        //   std::abs(my_posture[1] - cur_ball[1]) < oy){
        if (this->cal->is_desired_posture(my_posture, tar_posture) == true) {
            wheel_velos = this->motors[id]->move_to_target(my_posture, goal_pstn, 0);
        } else{
            wheel_velos = this->motors[id]->three_phase_move_to_target(my_posture, tar_posture, 0.35);
        }
    }else {
        wheel_velos = this->motors[id]->three_phase_move_to_target(my_posture, default_posture);
    }

    return wheel_velos;
}

std::array<double, 2> strategy::upper_size_attack(std::size_t id, std::size_t closest_id) {
    // optimize default posture if needed
    std::array<double, 3> default_posture = {0.55, 0.45, 0};
    return this->attack(id, closest_id, default_posture);
}

std::array<double, 2> strategy::lower_size_attack(std::size_t id, std::size_t closest_id) {
    // optimize default posture if needed
    std::array<double, 3> default_posture = {0.55, -0.45, 0};
    return this->attack(id, closest_id, default_posture);
}

std::array<double, 2> strategy::defense(std::size_t id, std::size_t closest_id, std::array<double, 3> default_posture) {
    std::array<double, 2> wheel_velos;

    std::array<std::array<double, 3>, 5> our_postures = this->data_proc->get_my_team_postures();
    std::array<double, 3> my_posture = our_postures[id];
    std::array<double, 2> cur_ball = this->data_proc->get_cur_ball_position();

    // if (id == closest_id) {
    //     std::array<double, 2> cur_trans = this->data_proc->get_cur_ball_transition();
    //     std::array<double, 2> goal_pstn = {1.1 + 0.75, 0}; // middle point of the net
    //     std::array<double, 3> tar_posture = this->cal->compute_desired_posture(cur_ball, cur_trans, goal_pstn);
    //     if (this->cal->is_desired_posture(my_posture, tar_posture) == false) {
    //         wheel_velos = this->motors[id]->three_phase_move_to_target(my_posture, tar_posture);
    //     } else {
    //         wheel_velos = this->motors[id]->move_to_target(my_posture, goal_pstn, 0);
    //     }
    // } else {
    //     wheel_velos = this->motors[id]->three_phase_move_to_target(my_posture, default_posture);
    // }

    if (cur_ball[0] < -0.1){ // little margin
        double ox = 0.05;
        std::array<double, 2> cur_trans = this->data_proc->get_cur_ball_transition();
        std::array<double, 2> goal_pstn = {1.1 + 0.75, 0}; // middle point of the net
        std::array<double, 3> tar_posture = this->cal->compute_desired_posture(cur_ball, cur_trans, goal_pstn);
        if(my_posture[0] < cur_ball[0] - ox){
            wheel_velos = this->motors[id]->move_to_target(my_posture, cur_ball, 0.2); //0.2 damping
        } else{
            wheel_velos = this->motors[id]->three_phase_move_to_target(my_posture, tar_posture);
        }
    }else {
        wheel_velos = this->motors[id]->three_phase_move_to_target(my_posture, default_posture);
    }

    return wheel_velos;
}

std::array<double, 2> strategy::upper_size_defense(std::size_t id, std::size_t closest_id) {
    // optimize default posture if needed
    std::array<double, 2> cur_ball = this->data_proc->get_cur_ball_position();
    double y = std::max(0.0, cur_ball[1]);
    std::array<double, 3> default_posture = {-0.55, y, 0};
    return this->defense(id, closest_id, default_posture);
}

std::array<double, 2> strategy::lower_size_defense(std::size_t id, std::size_t closest_id) {
    // optimize default posture if needed
    std::array<double, 2> cur_ball = this->data_proc->get_cur_ball_position();
    double y = std::min(0.0, cur_ball[1]);
    std::array<double, 3> default_posture = {-0.55, y, 0};
    return this->defense(id, closest_id, default_posture);
}


std::array<double, 2> strategy::keep_goal(std::size_t id){
    std::array<double, 2> wheel_velos;

    std::array<std::array<double, 3>, 5> our_postures = this->data_proc->get_my_team_postures();
    std::array<double, 3> my_posture = our_postures[id];
    std::array<double, 2> cur_ball = this->data_proc->get_cur_ball_position();

    const double x = -FIELD_LENGTH/2 + ROBOT_SIZE/2;
    const double y = std::max(std::min(cur_ball[1], GOAL_WIDTH/2 - ROBOT_SIZE/2),
                              -GOAL_WIDTH/2 + ROBOT_SIZE/2);
    if (cur_ball[0] < -FIELD_LENGTH/2 + PENALTY_AREA_DEPTH && std::abs(cur_ball[1]) < PENALTY_AREA_WIDTH/2){
        wheel_velos =this->motors[id]->move_to_target(my_posture, cur_ball, 0.35);
    }else{
        wheel_velos = this->motors[id]->move_to_target(my_posture, {x, y});
    }
    return wheel_velos;
}

std::size_t strategy::find_closest_robot(void)
{
    //Check for distances between each non-goallie robot and the ball, return the index of robot closest to the ball.
    std::size_t num_of_robots = 5;
    std::size_t min_idx = 0;
    double min_dist = 9999.99;
    std::array<std::array<double, 3>, 5> our_postures = this->data_proc->get_my_team_postures();
    std::array<double, 2> cur_ball = this->data_proc->get_cur_ball_position();

    for (std::size_t i = 0; i < num_of_robots - 1; i++) {
        std::array<double, 2> cur_pstn = {our_postures[i][0], our_postures[i][1]};
        double measured_dist = this->cal->get_distance(cur_pstn, cur_ball);
        if (measured_dist < min_dist) { //if this robot's distance is shorter, replace. (robot with smaller idx has priority over one with larger idx when two dists are equal)
            min_dist = measured_dist;
            min_idx = i;
        }
    }
    if (this->is_debug){
        std::cout << __func__ << ": closest robot index " << min_idx << std::endl;
    }
    return min_idx;
}

}