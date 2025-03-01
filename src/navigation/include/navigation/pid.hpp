#ifndef PID_H
#define PID_H

class PID
{
public:
    PID(double p, double i, double d, double max_p, double max_i, double max_d, double dt);
    ~PID() = default;
    void calculate(double dist);
private:
    double kp;
    double ki;
    double kd;
    double dt;
    double max_p;
    double max_i;
    double max_d;
    double integral;
    double p_out;
    double i_out;
    double d_out;
    double total_out;
};

#endif