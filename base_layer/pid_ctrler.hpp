#ifndef _AIM_PID_H
#define _AIM_PID_H

namespace aim {

class pid_ctrler {
public:
	pid_ctrler(double kP, double kI, double kD, double dt = 0.05, bool is_debug = false);
    ~pid_ctrler();

    double control(double err);
private:
    // parameters for PID controller
    double kP;
    double kI;
    double kD;

    // previous error and sum error
    double prev_err = 0;
    double sum_err = 0;

    // sampling time
    double dt;

    // use for debug
    bool is_debug;
};
}
#endif //_AIM_PID_H