#include "ai_base.hpp"

#include <boost/lexical_cast.hpp>

#include <fstream>
#include <iostream>
#include <random>
#include <vector>

#include "base_layer/base_layer.hpp"

class aim_ai : public aiwc::ai_base {
    static constexpr double PI = 3.1415926535;

public:
    // Constructor
    aim_ai(std::string server_ip, std::size_t port, std::string realm, std::string key, std::string datapath, bool is_debug=false)
        : aiwc::ai_base(std::move(server_ip), port, std::move(realm), std::move(key), std::move(datapath))
        , robot_wheels{}
    {
        this->is_debug = is_debug;
		layer = new aim::base_layer(is_debug=true);
		if (is_debug) {
			std::cout << "I am ready." << std::endl;
		}

    }

private:
	// implement parent virtual methods
	void init() {
		if (is_debug) {
			std::cout << "AIM AI has been initilized" << std::endl;
		}
	}

	// member methods
    const auto get_cur_postures(const aiwc::frame& f, std::size_t team){
		// get current postures including x, y, th of team (ours or opponent)
		// based on received frame

        std::array<std::array<double, 3>, 5> cur_postures;
        for (std::size_t id = 0; id < 5; ++id) {
            cur_postures[id][X]  = (*f.opt_coordinates).robots[team][id].x;
            cur_postures[id][Y]  = (*f.opt_coordinates).robots[team][id].y;
            cur_postures[id][TH] = (*f.opt_coordinates).robots[team][id].th;
        }

		return cur_postures;
    }

    const auto get_my_team_postures(const aiwc::frame& f){
        return this->get_cur_postures(f, MYTEAM);
    }

	const auto get_opponent_postures(const aiwc::frame& f) {
		return this->get_cur_postures(f, OPPONENT);
	}

	const auto get_cur_ball_position(const aiwc::frame& f) {
		const std::array<double, 2> pos_ball = { (*f.opt_coordinates).ball.x, (*f.opt_coordinates).ball.y };
		return pos_ball;
	}

    void update(const aiwc::frame& f)
    {
        if (f.reset_reason == aiwc::GAME_START) {
            std::cout << "Game started : " << f.time << std::endl;
        }
        if (f.reset_reason == aiwc::SCORE_MYTEAM) {
            // yay! my team scored!
            std::cout << "Myteam scored : " << f.time << std::endl;
        }
        else if (f.reset_reason == aiwc::SCORE_OPPONENT) {
            // T_T what have you done
            std::cout << "Opponent scored : " << f.time << std::endl;
        }
        else if (f.reset_reason == aiwc::GAME_END) {
            // game is finished. finish() will be called after you return.
            // now you have about 30 seconds before this process is killed.
            std::cout << "Game ended : " << f.time << std::endl;
            return;
        }

        if (f.opt_coordinates) { // if the optional coordinates are given,
            // const auto& myteam   = f.opt_coordinates->robots[MYTEAM];
            // const auto& opponent = f.opt_coordinates->robots[OPPONENT];
            // const auto& ball     = f.opt_coordinates->ball;

            // const auto& myteam0_x      = (*f.opt_coordinates).robots[MYTEAM][0].x;
            // const auto& myteam0_y      = (*f.opt_coordinates).robots[MYTEAM][0].y;
            // const auto& myteam0_th     = (*f.opt_coordinates).robots[MYTEAM][0].th;
            // const auto& myteam0_active = (*f.opt_coordinates).robots[MYTEAM][0].is_active;
            

			// Get infos
			this->our_postures = this->get_my_team_postures(f);
			this->opnt_postures = this->get_opponent_postures(f);



            /*****************************************
             * OUR ALGORITHM HERE
             ****************************************/

            std::array<double, 2> wheel_velos;
            std::array<double, 3> tar_posture = {0, -0.4, PI/3};
            
            wheel_velos = layer->three_phase_move_to_target(this->our_postures[1], tar_posture);
            robot_wheels[1] = wheel_velos;
            std::cout << this->count << "theta " << this->our_postures[1][TH] << std::endl;

            


            /*****************************************
             * FINISH OUR ALGORITHM
             ****************************************/


            std::array<double, 10> ws;

            for (std::size_t id = 0; id < 5; ++id) {
                //std::cout << "Robot " << id << ":[" << robot_wheels[id][0] << "," << robot_wheels[id][1] << "]" << std::endl; //print robots info
                ws[2 * id    ] = robot_wheels[id][0]; // left
                ws[2 * id + 1] = robot_wheels[id][1]; // right
            }
            set_wheel(ws);
        }
        else { // given no coordinates, you need to get coordinates from image
        }

        // std::array<double, 10> wheels;
        // for(auto& w : wheels) {
        //   w = info.max_linear_velocity;
        // }
        // set_wheel(wheels); // every robot will go ahead with maximum velocity

    }

    void finish()
    {
        // You have less than 30 seconds before it's killed.
        std::ofstream ofs(datapath + "/result.txt");

        // print header
        ofs << "time, a.score, b.score, ";
        ofs << "ball.x, ball.y" << std::endl;

        // print data
        for (const auto& f : frames) {
            std::array<double, 2> ball;

            ball = { (*f.opt_coordinates).ball.x, (*f.opt_coordinates).ball.y };

            ofs << f.time << ", "
                << f.score[0] << ", " << f.score[1] << ", ";

            ofs << ball[X] << ", " << ball[Y] << std::endl;
        }
    }

private: // member variable
    aiwc::frame previous_frame;

	// cur postures of our team and opponent w.r.t cur frame
    std::array<std::array<double, 3>, 5> our_postures;
	std::array<std::array<double, 3>, 5> opnt_postures;
	

    std::array<double, 2> prev_ball;
    std::array<double, 2> cur_ball;

	// robot_wheels to set velocity for the next frame
    std::array<std::array<double, 2>, 5> robot_wheels;


    std::vector<aiwc::frame> frames;

    // thangvu
    int count = 0; //test
    double last = 0;
	bool is_debug = true;
    aim::base_layer *layer;
    std::size_t phase = 1;
};

int main(int argc, char *argv[])
{
    if (argc < 6) {
        std::cerr << "Usage " << argv[0] << " server_ip port realm key datapath" << std::endl;
        return -1;
    }

    const auto& server_ip = std::string(argv[1]);
    const auto& port      = boost::lexical_cast<std::size_t>(argv[2]);
    const auto& realm     = std::string(argv[3]);
    const auto& key       = std::string(argv[4]);
    const auto& datapath  = std::string(argv[5]);


    aim_ai ai(server_ip, port, realm, key, datapath, true);

    ai.run();

    return 0;
}
